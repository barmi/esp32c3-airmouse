mod ble_hid;
mod imu;
mod mapping;

use ble_hid::{BleMouse, MouseReport};
use esp_idf_svc::hal::{
    delay::FreeRtos,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::Hertz,
};
use mapping::CursorMapper;

/// 메인 루프 주기 (ms). IMU 샘플링/HID 리포트 주기이기도 하다.
const LOOP_PERIOD_MS: u32 = 10;

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

    // 부팅 시 정지 상태 가정하고 자이로 바이어스 캘리브레이션 (약 1.3초)
    log::info!("자이로 캘리브레이션 중... 보드를 움직이지 마세요");
    let bias = imu.calibrate_gyro(60, 200)?;
    log::info!(
        "캘리브레이션 완료: bias[dps] x={:+.2} y={:+.2} z={:+.2}",
        bias.x, bias.y, bias.z
    );

    let mut mapper = CursorMapper::default();
    let mut tick: u32 = 0;
    loop {
        let gyro = imu.gyro_dps()?;

        // 축 매핑: yaw=Z(좌우 회전) → dx, pitch=X(상하 기울임) → dy
        let (dx, dy) = mapper.update(gyro.z - bias.z, gyro.x - bias.x);

        if mouse.connected() {
            let report = MouseReport {
                dx,
                dy,
                ..Default::default()
            };
            // 변화 없는 0 리포트는 보내지 않는다 (BLE 대역폭/전력 절약)
            if !report.is_zero() {
                mouse.send(&report);
            }
        }

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
