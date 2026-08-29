mod ble_hid;
mod button;
mod imu;
mod mapping;
mod status_led;

use ble_hid::{BleMouse, MouseReport};
use button::{Button, Repeater};
use esp_idf_svc::hal::{
    delay::FreeRtos,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::Hertz,
};
use mapping::{CursorMapper, SCROLL_REPEAT_DELAY_MS, SCROLL_REPEAT_MS, SCROLL_SIGN};
use status_led::{Status, StatusLed};

/// 메인 루프 주기 (ms). IMU 샘플링/HID 리포트 주기이기도 하다.
const LOOP_PERIOD_MS: u32 = 10;

/// HID 리포트의 버튼 비트
const BTN_LEFT: u8 = 0x01;
const BTN_RIGHT: u8 = 0x02;

fn main() -> anyhow::Result<()> {
    // ESP-IDF 링커 패치 적용 (esp-idf 프로젝트 필수 보일러플레이트)
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("esp32c3-airmouse: boot OK");

    let p = Peripherals::take()?;

    // 상태 표시등 (WS2812, GPIO2).
    // channel0은 ws2812 드라이버가 요구하는 구 RMT API — status_led.rs 주석 참고.
    #[allow(deprecated)]
    let mut led = StatusLed::new(p.rmt.channel0, p.pins.gpio2)?;
    led.set(Status::Advertising, true);

    // BLE HID 마우스 초기화 및 광고 시작
    let mut mouse = BleMouse::new("ESP32C3 AirMouse")?;

    // I2C0: SDA=GPIO7, SCL=GPIO8, IMU 주소 0x68.
    // 주의: RUST-2 보드는 RUST-1과 달리 유저 LED가 GPIO10을 쓰면서
    // SDA가 GPIO10 → GPIO7로 이동했다 (실보드 스캔으로 확인, 공식 문서의
    // 핀 테이블은 RUST-1 값을 그대로 실은 오류가 있음).
    let i2c = I2cDriver::new(
        p.i2c0,
        p.pins.gpio7,
        p.pins.gpio8,
        &I2cConfig::new().baudrate(Hertz(400_000)),
    )?;
    let mut imu = imu::Imu::new(i2c)?;
    log::info!("ICM-42670-P 초기화 완료");

    // 외부 택트 스위치. 각각 GND로 당기는 배선이며 내부 풀업을 쓴다.
    let mut btn_left = Button::new(p.pins.gpio3)?;
    let mut btn_right = Button::new(p.pins.gpio4)?;
    let mut btn_scroll_up = Button::new(p.pins.gpio5)?;
    let mut btn_scroll_down = Button::new(p.pins.gpio6)?;
    // 온보드 BOOT 버튼은 현장 재캘리브레이션 트리거로 쓴다.
    let mut btn_recal = Button::new(p.pins.gpio9)?;

    let mut up_repeat = Repeater::new(SCROLL_REPEAT_DELAY_MS, SCROLL_REPEAT_MS, LOOP_PERIOD_MS);
    let mut down_repeat = Repeater::new(SCROLL_REPEAT_DELAY_MS, SCROLL_REPEAT_MS, LOOP_PERIOD_MS);

    // 부팅 시 정지 상태 가정하고 자이로 바이어스 캘리브레이션 (실측 약 2.6초)
    log::info!("자이로 캘리브레이션 중... 보드를 움직이지 마세요");
    led.set(Status::Calibrating, true);
    let mut bias = imu.calibrate_gyro(60, 200)?;
    log::info!(
        "캘리브레이션 완료: bias[dps] x={:+.2} y={:+.2} z={:+.2}",
        bias.x,
        bias.y,
        bias.z
    );

    let mut mapper = CursorMapper::default();
    let mut prev_buttons = 0u8;
    let mut tick: u32 = 0;
    loop {
        btn_left.update();
        btn_right.update();
        btn_scroll_up.update();
        btn_scroll_down.update();
        btn_recal.update();

        // BOOT 버튼: 커서가 흐를 때 재부팅 없이 영점을 다시 잡는다.
        // 자이로는 이미 워밍업된 상태라 폐기 샘플은 적게 잡는다.
        if btn_recal.just_pressed() {
            log::info!("재캘리브레이션 요청 — 보드를 내려놓고 기다리세요");
            led.set(Status::Calibrating, true);
            match imu.calibrate_gyro(10, 200) {
                Ok(new_bias) => {
                    bias = new_bias;
                    log::info!(
                        "재캘리브레이션 완료: bias[dps] x={:+.2} y={:+.2} z={:+.2}",
                        bias.x,
                        bias.y,
                        bias.z
                    );
                }
                // 여기서 앱을 끝내면 마우스가 죽는다. 기존 값을 유지하고 계속 간다.
                Err(e) => log::warn!("{e} — 기존 바이어스를 유지합니다"),
            }
            // 캘리브레이션 동안 쌓였을 잔여 이동량을 버린다
            mapper = CursorMapper::default();
            continue;
        }

        let gyro = imu.gyro_dps()?;

        // 축 매핑: yaw=Z(좌우 회전) → dx, pitch=X(상하 기울임) → dy
        let (dx, dy) = mapper.update(gyro.z - bias.z, gyro.x - bias.x);

        let mut buttons = 0u8;
        if btn_left.pressed() {
            buttons |= BTN_LEFT;
        }
        if btn_right.pressed() {
            buttons |= BTN_RIGHT;
        }

        // 휠은 상대값이라 움직인 틱에만 실어 보낸다
        let mut wheel = 0i8;
        if up_repeat.tick(&btn_scroll_up) {
            wheel += SCROLL_SIGN;
        }
        if down_repeat.tick(&btn_scroll_down) {
            wheel -= SCROLL_SIGN;
        }

        // 상태 표시: 연결되면 초록 상시등, 광고 중이면 파랑 느린 깜빡임
        if mouse.connected() {
            led.set(Status::Connected, true);
        } else {
            // 1초 주기로 깜빡임 (100틱 = 1초, 앞 절반만 점등)
            led.set(Status::Advertising, tick % 100 < 50);
        }

        if mouse.connected() {
            let report = MouseReport {
                buttons,
                dx,
                dy,
                wheel,
            };
            // 버튼 상태가 바뀐 리포트는 이동량이 0이어도 반드시 보낸다
            // (누름/뗌이 모두 전달되어야 클릭과 드래그가 성립한다)
            if !report.is_zero() || buttons != prev_buttons {
                mouse.send(&report);
            }
        }
        prev_buttons = buttons;

        // 5초에 한 번 상태 로그 (시리얼이 밀리지 않게)
        if tick % 500 == 0 {
            log::info!(
                "connected={} | gyro-bias[dps] yaw(z)={:+7.2} pitch(x)={:+7.2}",
                mouse.connected(),
                gyro.z - bias.z,
                gyro.x - bias.x
            );
        }

        tick = tick.wrapping_add(1);
        FreeRtos::delay_ms(LOOP_PERIOD_MS);
    }
}
