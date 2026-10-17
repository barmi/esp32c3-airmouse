mod ble_hid;
mod button;
mod imu;
mod mapping;
mod orientation;
mod pointer;
mod status_led;

use ble_hid::{BleMouse, MouseReport};
use button::{Button, Gesture, HoldGesture, Repeater};
use esp_idf_svc::hal::{
    delay::FreeRtos,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::Hertz,
};
use mapping::{
    CursorMapper, ABS_FILTER_BETA, ABS_FILTER_D_CUTOFF_HZ, ABS_FILTER_MIN_CUTOFF_HZ,
    ABS_YAW_GATE_DPS, SCROLL_REPEAT_DELAY_MS, SCROLL_REPEAT_MS, SCROLL_SIGN,
};
use orientation::{BiasTracker, Orientation};
use pointer::{AngleTracker, ScreenCal, TrackerConfig};
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

/// BOOT 버튼을 이만큼 누르고 있으면 길게 누름 (모드 전환)
const BOOT_LONG_MS: u32 = 1500;

/// 버튼이 눌린 것으로 인식된 순간보다 이만큼 앞선 각도를 쓴다 (틱). 디바운스로
/// 인식이 늦는 데다 누르는 순간 보드가 흔들리므로, 그 직전에 가리키던 곳을 쓴다.
const PRESS_LOOKBACK_TICKS: usize = 5;

/// 절대 좌표가 이만큼 이상 바뀌어야 보낸다 (0..=32767 단위). 필터가 수렴한 뒤
/// 미세한 흔들림으로 매 틱 전송되는 것을 막는다. 2면 1920px 화면에서 0.1px 수준.
const ABS_SEND_THRESHOLD: u16 = 2;

/// 포인팅 모드
#[derive(Clone, Copy)]
enum Mode {
    /// 상대 이동 (기본). 회전 속도를 커서 이동량으로 보낸다.
    Relative,
    /// 화면 캘리브레이션 1단계: 왼쪽 위 모서리를 가리키고 좌클릭
    CalTopLeft,
    /// 화면 캘리브레이션 2단계: 오른쪽 아래 모서리를 가리키고 좌클릭
    CalBottomRight { top_left: (f32, f32) },
    /// 절대 좌표. 가리키는 각도를 화면 위치로 보낸다.
    Absolute(ScreenCal),
}

impl Mode {
    fn name(&self) -> &'static str {
        match self {
            Mode::Relative => "상대",
            Mode::CalTopLeft | Mode::CalBottomRight { .. } => "화면캘리브레이션",
            Mode::Absolute(_) => "절대",
        }
    }
}

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
    // 온보드 BOOT 버튼: 짧게 = 재캘리브레이션(상대) / 리센터(절대), 길게 = 모드 전환
    let mut btn_boot = Button::new(p.pins.gpio9)?;
    let mut boot_gesture = HoldGesture::new(BOOT_LONG_MS, LOOP_PERIOD_MS);

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
    let mut bias_tracker = BiasTracker::default();
    let mut angles = AngleTracker::new(TrackerConfig {
        yaw_gate_dps: ABS_YAW_GATE_DPS,
        min_cutoff_hz: ABS_FILTER_MIN_CUTOFF_HZ,
        beta: ABS_FILTER_BETA,
        d_cutoff_hz: ABS_FILTER_D_CUTOFF_HZ,
    });
    let mut mode = Mode::Relative;
    // BOOT를 누르기 직전에 가리키던 각도 (리센터 기준)
    let mut boot_press_angles = (0.0f32, 0.0f32);
    // 캘리브레이션 단계에서 쓴 좌클릭이 호스트로 새지 않도록 뗄 때까지 막는다
    let mut swallow_left = false;
    let mut last_abs = (u16::MAX, u16::MAX);
    let mut last_tick = Instant::now();
    let mut prev_buttons = 0u8;
    let mut prev_btn_state = 0u8;
    let mut tick: u32 = 0;
    loop {
        // ------------------------------------------------------------ 입력
        btn_left.update();
        btn_right.update();
        btn_scroll_up.update();
        btn_scroll_down.update();
        btn_boot.update();

        // 버튼 상태가 바뀔 때만 로그 (배선 확인용, 사람 손 속도라 시리얼을 밀지 않는다)
        let btn_state = btn_left.pressed() as u8
            | (btn_right.pressed() as u8) << 1
            | (btn_scroll_up.pressed() as u8) << 2
            | (btn_scroll_down.pressed() as u8) << 3
            | (btn_boot.pressed() as u8) << 4;
        if btn_state != prev_btn_state {
            log::info!(
                "버튼: 좌={} 우={} 스크롤↑={} 스크롤↓={} BOOT={}",
                btn_left.pressed(),
                btn_right.pressed(),
                btn_scroll_up.pressed(),
                btn_scroll_down.pressed(),
                btn_boot.pressed()
            );
            prev_btn_state = btn_state;
        }

        // ------------------------------------------------------------ 센서
        // 루프 주기는 10ms 대기 + 처리 시간이라 고정값이 아니다. 실측한다.
        let now = Instant::now();
        let dt = now.duration_since(last_tick).as_secs_f32().min(MAX_DT_S);
        last_tick = now;

        let raw = imu.gyro_dps()?;
        let accel = imu.accel_g()?;
        bias_tracker.update(raw, &mut bias, accel);
        let gyro = raw - bias;
        orient.update(gyro, accel, dt);

        // 기울기 보정: 보드를 옆으로 기울여 쥐어도 실제 좌우·상하로 나눈다.
        // 수평이면 기존 축(yaw=Z, pitch=X)과 같다.
        let (yaw_rate, pitch_rate) = orient.world_rates(gyro);
        // 절대 모드는 방위각의 정확한 미분을 적분한다. 좌우 각속도를 그대로 적분하면
        // 위·아래를 가리킨 채 움직인 경로에 따라 어긋난다.
        angles.update(orient.azimuth_rate(gyro), orient.elevation_deg(), dt);

        // ------------------------------------------------------------ BOOT 버튼
        if btn_boot.just_pressed() {
            boot_press_angles = angles.angles_before(PRESS_LOOKBACK_TICKS);
        }
        match boot_gesture.tick(&btn_boot) {
            Gesture::Short => match &mut mode {
                // 커서가 흐를 때 재부팅 없이 자이로 영점을 다시 잡는다.
                // 자이로는 이미 워밍업된 상태라 폐기 샘플은 적게 잡는다.
                Mode::Relative => {
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
                    // 캘리브레이션 동안 쌓였을 잔여 이동량과 필터 상태를 버린다
                    mapper = CursorMapper::default();
                    angles.reset_filters();
                    last_tick = Instant::now();
                    continue;
                }
                // 좌우 각도는 자력계가 없어 서서히 흐른다. 화면 중앙을 가리키고 눌러 바로잡는다.
                Mode::Absolute(cal) => {
                    cal.recenter(boot_press_angles);
                    log::info!("리센터: 누르기 직전에 가리키던 곳을 화면 중앙으로 맞춤");
                }
                Mode::CalTopLeft | Mode::CalBottomRight { .. } => {}
            },
            Gesture::Long => {
                if mouse.connected() {
                    match &mode {
                        // 버튼을 누른 채 모드가 바뀌면 이전 리포트 쪽에 눌림이 남으므로 뗀다
                        Mode::Relative if prev_buttons != 0 => mouse.send(&MouseReport::default()),
                        // 펜이 근접 범위를 벗어났다고 알린다. 스위치도 함께 풀린다.
                        Mode::Absolute(cal) => {
                            let (x, y) = cal.position(angles.angles());
                            mouse.send_pen(0, x, y, false);
                        }
                        _ => {}
                    }
                }
                prev_buttons = 0;
                swallow_left = btn_left.pressed();
                mode = match mode {
                    Mode::Relative => {
                        if mouse.connected() && !mouse.abs_subscribed() {
                            log::warn!(
                                "호스트가 절대 좌표 리포트를 구독하지 않았습니다. Mac에서 기기를 지우고 다시 페어링하세요"
                            );
                        }
                        log::info!(
                            "화면 캘리브레이션: 화면 왼쪽 위 모서리를 가리키고 좌클릭하세요"
                        );
                        Mode::CalTopLeft
                    }
                    _ => {
                        log::info!("상대 이동 모드로 전환");
                        Mode::Relative
                    }
                };
            }
            Gesture::None => {}
        }

        // ------------------------------------------------------------ 화면 캘리브레이션
        // 좌클릭으로 단계를 진행한다. 누르는 순간의 흔들림을 피해 직전 각도를 쓴다.
        if btn_left.just_pressed() {
            let pointed = angles.angles_before(PRESS_LOOKBACK_TICKS);
            match mode {
                Mode::CalTopLeft => {
                    swallow_left = true;
                    log::info!(
                        "왼쪽 위 기록 (방위각={:+.1}° 고도각={:+.1}°). 이제 오른쪽 아래 모서리를 가리키고 좌클릭하세요",
                        pointed.0,
                        pointed.1
                    );
                    mode = Mode::CalBottomRight { top_left: pointed };
                }
                Mode::CalBottomRight { top_left } => {
                    swallow_left = true;
                    match ScreenCal::from_corners(top_left, pointed) {
                        Some(cal) => {
                            let (w, h) = cal.span_deg();
                            log::info!(
                                "화면 캘리브레이션 완료: 오른쪽 아래 (방위각={:+.1}° 고도각={:+.1}°), 화면이 가로 {w:.1}°, 세로 {h:.1}°를 차지합니다. 절대 좌표 모드 시작",
                                pointed.0,
                                pointed.1
                            );
                            last_abs = (u16::MAX, u16::MAX);
                            mode = Mode::Absolute(cal);
                        }
                        None => {
                            log::warn!(
                                "두 모서리가 너무 가깝습니다. 왼쪽 위 모서리부터 다시 가리키고 좌클릭하세요"
                            );
                            mode = Mode::CalTopLeft;
                        }
                    }
                }
                Mode::Relative | Mode::Absolute(_) => {}
            }
        }
        if swallow_left && !btn_left.pressed() {
            swallow_left = false;
        }

        // ------------------------------------------------------------ 버튼/휠
        let mut buttons = 0u8;
        if btn_left.pressed() && !swallow_left {
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

        // ------------------------------------------------------------ 상태 LED
        // 캘리브레이션은 빠른 깜빡임(0.3초), 광고 중은 느린 깜빡임(1초)
        match &mode {
            Mode::CalTopLeft => led.set(Status::ScreenCalTopLeft, tick % 30 < 15),
            Mode::CalBottomRight { .. } => led.set(Status::ScreenCalBottomRight, tick % 30 < 15),
            _ if !mouse.connected() => led.set(Status::Advertising, tick % 100 < 50),
            Mode::Relative => led.set(Status::Connected, true),
            Mode::Absolute(_) => led.set(Status::ConnectedAbsolute, true),
        }

        // ------------------------------------------------------------ 전송
        if mouse.connected() {
            match &mode {
                Mode::Relative => {
                    let (dx, dy) = mapper.update(yaw_rate, pitch_rate);
                    // 움직임·휠이 있거나 버튼 상태가 바뀐 틱에만 보낸다.
                    // 버튼 변화는 이동량이 0이어도 반드시 보낸다 (누름/뗌이 모두 전달되어야
                    // 클릭과 드래그가 성립한다). 반대로 누르고만 있는 동안은 보내지 않는다 —
                    // 호스트가 버튼 상태를 기억하므로 같은 리포트를 100Hz로 반복할 필요가 없다.
                    if dx != 0 || dy != 0 || wheel != 0 || buttons != prev_buttons {
                        mouse.send(&MouseReport {
                            buttons,
                            dx,
                            dy,
                            wheel,
                        });
                    }
                }
                Mode::Absolute(cal) => {
                    let (x, y) = cal.position(angles.angles());
                    let moved = x.abs_diff(last_abs.0) >= ABS_SEND_THRESHOLD
                        || y.abs_diff(last_abs.1) >= ABS_SEND_THRESHOLD;
                    if moved || buttons != prev_buttons {
                        mouse.send_pen(buttons, x, y, true);
                        last_abs = (x, y);
                    }
                    // 펜 리포트에는 휠이 없어 상대 리포트로 보낸다. 다만 상대 리포트의
                    // 버튼 0이 펜의 눌림과 충돌해 드래그를 끊을 수 있으므로, 버튼을
                    // 누르지 않았을 때만 보낸다.
                    if wheel != 0 && buttons == 0 {
                        mouse.send(&MouseReport {
                            wheel,
                            ..Default::default()
                        });
                    }
                }
                // 캘리브레이션 중에는 아무것도 보내지 않는다 (좌클릭이 호스트로 가지 않도록)
                Mode::CalTopLeft | Mode::CalBottomRight { .. } => {}
            }
        }
        prev_buttons = buttons;

        // 절대 모드 진단 (0.5초마다): 가리키는 각도와 실제로 보내는 좌표.
        // 커서 위치가 이상할 때 펌웨어 쪽 값인지 호스트 쪽 해석인지 가른다.
        if let Mode::Absolute(cal) = &mode {
            if tick % 50 == 0 {
                let a = angles.angles();
                let (x, y) = cal.position(a);
                log::info!(
                    "절대: 방위각={:+6.1}° 고도각={:+5.1}° → x={:5} ({:3.0}%) y={:5} ({:3.0}%) 구독={}",
                    a.0,
                    a.1,
                    x,
                    x as f32 / 327.67,
                    y,
                    y as f32 / 327.67,
                    mouse.abs_subscribed()
                );
            }
        }

        // 5초에 한 번 상태 로그 (시리얼이 밀리지 않게)
        if tick % 500 == 0 {
            let (yaw_deg, el_deg) = angles.angles();
            log::info!(
                "connected={} 모드={} | yaw={:+6.2} pitch={:+6.2} dps | 기울기={:+5.1}° | 각도 yaw={:+6.1}° el={:+5.1}° | bias=({:+.2},{:+.2},{:+.2})",
                mouse.connected(),
                mode.name(),
                yaw_rate,
                pitch_rate,
                orient.roll_deg(),
                yaw_deg,
                el_deg,
                bias.x,
                bias.y,
                bias.z
            );
        }

        tick = tick.wrapping_add(1);
        FreeRtos::delay_ms(LOOP_PERIOD_MS);
    }
}
