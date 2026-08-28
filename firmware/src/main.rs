mod ble_hid;
mod imu;

use ble_hid::{BleMouse, MouseReport};
use esp_idf_svc::hal::{
    delay::FreeRtos,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::Hertz,
};

/// 메인 루프 주기 (ms). IMU 샘플링/HID 리포트 주기이기도 하다.
const LOOP_PERIOD_MS: u32 = 10;

/// 이슈 #3 검증용: 연결되면 커서로 원을 그리는 테스트 패턴.
/// 커서 매핑(#4)이 들어오면 false로 바꾼다.
const CURSOR_TEST_PATTERN: bool = true;

fn main() -> anyhow::Result<()> {
    // ESP-IDF 링커 패치 적용 (esp-idf 프로젝트 필수 보일러플레이트)
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("esp32c3-airmouse: boot OK");

    let p = Peripherals::take()?;

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

    let mut tick: u32 = 0;
    loop {
        let gyro = imu.gyro_dps()?;
        let accel = imu.accel_g()?;

        if mouse.connected() && CURSOR_TEST_PATTERN {
            // 약 3초에 한 바퀴 도는 원 그리기
            let theta = tick as f32 * 0.02;
            let report = MouseReport {
                dx: (5.0 * theta.cos()) as i8,
                dy: (5.0 * theta.sin()) as i8,
                ..Default::default()
            };
            mouse.send(&report);
        }

        // 5초에 한 번 상태 로그 (시리얼이 밀리지 않게)
        if tick % 500 == 0 {
            log::info!(
                "connected={} | gyro[dps] x={:+8.2} y={:+8.2} z={:+8.2} | accel[g] x={:+6.3} y={:+6.3} z={:+6.3}",
                mouse.connected(),
                gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z
            );
        }

        tick = tick.wrapping_add(1);
        FreeRtos::delay_ms(LOOP_PERIOD_MS);
    }
}
