mod ble_hid;
mod button;
mod imu;
mod mapping;
mod orientation;
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
use orientation::Orientation;
use status_led::{Status, StatusLed};
use std::time::Instant;

/// 메인 루프 주기 (ms). IMU 샘플링/HID 리포트 주기이기도 하다.
const LOOP_PERIOD_MS: u32 = 10;

/// 실측 루프 주기의 상한(초). 재캘리브레이션처럼 루프가 오래 멈췄던 직후
/// 한 틱의 적분이 튀지 않도록 막는다.
const MAX_DT_S: f32 = 0.05;

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
    let mut orient = Orientation::default();
    let mut last_tick = Instant::now();
    let mut prev_buttons = 0u8;
    let mut prev_btn_state = 0u8;
    let mut tick: u32 = 0;
    loop {
        btn_left.update();
        btn_right.update();
        btn_scroll_up.update();
        btn_scroll_down.update();
        btn_recal.update();

        // 버튼 상태가 바뀔 때만 로그 (배선 확인용, 사람 손 속도라 시리얼을 밀지 않는다)
        let btn_state = btn_left.pressed() as u8
            | (btn_right.pressed() as u8) << 1
            | (btn_scroll_up.pressed() as u8) << 2
            | (btn_scroll_down.pressed() as u8) << 3
            | (btn_recal.pressed() as u8) << 4;
        if btn_state != prev_btn_state {
            log::info!(
                "버튼: 좌={} 우={} 스크롤↑={} 스크롤↓={} BOOT={}",
                btn_left.pressed(),
                btn_right.pressed(),
                btn_scroll_up.pressed(),
                btn_scroll_down.pressed(),
                btn_recal.pressed()
            );
            prev_btn_state = btn_state;
        }

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
            last_tick = Instant::now();
            continue;
        }

        // 루프 주기는 10ms 대기 + 처리 시간이라 고정값이 아니다. 실측한다.
        let now = Instant::now();
        let dt = now.duration_since(last_tick).as_secs_f32().min(MAX_DT_S);
        last_tick = now;

        let gyro = imu.gyro_dps()? - bias;
        orient.update(gyro, imu.accel_g()?, dt);

        // 기울기 보정: 보드를 옆으로 기울여 쥐어도 실제 좌우·상하로 나눈다.
        // 수평이면 기존 축(yaw=Z, pitch=X)과 같다.
        let (yaw_rate, pitch_rate) = orient.world_rates(gyro);
        let (dx, dy) = mapper.update(yaw_rate, pitch_rate);

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
            // 움직임·휠이 있거나 버튼 상태가 바뀐 틱에만 보낸다.
            // 버튼 변화는 이동량이 0이어도 반드시 보낸다 (누름/뗌이 모두 전달되어야
            // 클릭과 드래그가 성립한다). 반대로 누르고만 있는 동안은 보내지 않는다 —
            // 호스트가 버튼 상태를 기억하므로 같은 리포트를 100Hz로 반복할 필요가 없다.
            if dx != 0 || dy != 0 || wheel != 0 || buttons != prev_buttons {
                mouse.send(&report);
            }
        }
        prev_buttons = buttons;

        // 5초에 한 번 상태 로그 (시리얼이 밀리지 않게)
        if tick % 500 == 0 {
            log::info!(
                "connected={} | yaw={:+7.2} pitch={:+7.2} dps | 기울기={:+5.1}° | dt={:.1}ms",
                mouse.connected(),
                yaw_rate,
                pitch_rate,
                orient.roll_deg(),
                dt * 1000.0
            );
        }

        tick = tick.wrapping_add(1);
        FreeRtos::delay_ms(LOOP_PERIOD_MS);
    }
}
