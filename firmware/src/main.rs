mod imu;

use esp_idf_svc::hal::{
    delay::FreeRtos,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::Hertz,
};

/// 메인 루프 주기 (ms). IMU 샘플링 주기이기도 하다.
const LOOP_PERIOD_MS: u32 = 10;

fn main() -> anyhow::Result<()> {
    // ESP-IDF 링커 패치 적용 (esp-idf 프로젝트 필수 보일러플레이트)
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("esp32c3-airmouse: boot OK");

    let p = Peripherals::take()?;

    // I2C0: SDA=GPIO10, SCL=GPIO8 (esp-rust-board 온보드 배선, IMU 주소 0x68)
    let i2c = I2cDriver::new(
        p.i2c0,
        p.pins.gpio10,
        p.pins.gpio8,
        &I2cConfig::new().baudrate(Hertz(400_000)),
    )?;
    let mut imu = imu::Imu::new(i2c)?;
    log::info!("ICM-42670-P 초기화 완료");

    let mut tick: u32 = 0;
    loop {
        let gyro = imu.gyro_dps()?;
        let accel = imu.accel_g()?;

        // 0.5초에 한 번만 로그 (100Hz 전체를 찍으면 시리얼이 밀린다)
        if tick % 50 == 0 {
            log::info!(
                "gyro[dps] x={:+8.2} y={:+8.2} z={:+8.2} | accel[g] x={:+6.3} y={:+6.3} z={:+6.3}",
                gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z
            );
        }

        tick = tick.wrapping_add(1);
        FreeRtos::delay_ms(LOOP_PERIOD_MS);
    }
}
