use esp_idf_svc::hal::i2c::I2cDriver;
use icm42670::{
    accelerometer::{vector::F32x3, Accelerometer},
    Address, GyroOdr, GyroRange, Icm42670, PowerMode,
};

/// ICM-42670-P 래퍼.
///
/// 에어마우스 용도 기본 설정:
/// - 6축 low-noise 모드
/// - 자이로 ±2000dps — 빠른 손동작에도 포화되지 않도록 최대 범위
/// - 자이로 ODR 200Hz (메인 루프 100Hz보다 높게 잡아 최신 값 보장)
pub struct Imu<'d> {
    drv: Icm42670<I2cDriver<'d>>,
}

impl<'d> Imu<'d> {
    pub fn new(i2c: I2cDriver<'d>) -> anyhow::Result<Self> {
        let mut drv = Icm42670::new(i2c, Address::Primary).map_err(err)?;
        drv.set_power_mode(PowerMode::SixAxisLowNoise).map_err(err)?;
        drv.set_gyro_range(GyroRange::Deg2000).map_err(err)?;
        drv.set_gyro_odr(GyroOdr::Hz200).map_err(err)?;
        Ok(Self { drv })
    }

    /// 자이로 각속도 (deg/sec)
    pub fn gyro_dps(&mut self) -> anyhow::Result<F32x3> {
        self.drv.gyro_norm().map_err(err)
    }

    /// 정지 상태를 가정하고 자이로 바이어스를 측정한다.
    ///
    /// 전원 인가 직후에는 자이로 스핀업 때문에 포화값이 나오므로
    /// 앞쪽 `discard`개 샘플은 버리고 `samples`개 평균을 낸다.
    pub fn calibrate_gyro(&mut self, discard: usize, samples: usize) -> anyhow::Result<F32x3> {
        use esp_idf_svc::hal::delay::FreeRtos;

        for _ in 0..discard {
            let _ = self.drv.gyro_norm();
            FreeRtos::delay_ms(5);
        }
        let (mut sx, mut sy, mut sz) = (0.0f32, 0.0f32, 0.0f32);
        for _ in 0..samples {
            let g = self.gyro_dps()?;
            sx += g.x;
            sy += g.y;
            sz += g.z;
            FreeRtos::delay_ms(5);
        }
        let n = samples as f32;
        Ok(F32x3::new(sx / n, sy / n, sz / n))
    }

    /// 가속도 (g)
    pub fn accel_g(&mut self) -> anyhow::Result<F32x3> {
        self.drv.accel_norm().map_err(err)
    }
}

fn err<E: core::fmt::Debug>(e: E) -> anyhow::Error {
    anyhow::anyhow!("IMU 오류: {e:?}")
}
