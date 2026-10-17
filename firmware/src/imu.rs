use crate::orientation::V3;
use esp_idf_svc::hal::i2c::I2cDriver;
use icm42670::{
    accelerometer::Accelerometer, AccelOdr, AccelRange, Address, GyroOdr, GyroRange, Icm42670,
    PowerMode,
};

/// ICM-42670-P 래퍼.
///
/// 에어마우스 용도 기본 설정:
/// - 6축 low-noise 모드
/// - 자이로 ±2000dps — 빠른 손동작에도 포화되지 않도록 최대 범위
/// - 자이로 ODR 200Hz (메인 루프 100Hz보다 높게 잡아 최신 값 보장)
/// - 가속도 ±4g, ODR 200Hz — 중력 방향 추정용. 손을 휘두를 때의 가속까지 담는다
pub struct Imu<'d> {
    drv: Icm42670<I2cDriver<'d>>,
}

impl<'d> Imu<'d> {
    pub fn new(i2c: I2cDriver<'d>) -> anyhow::Result<Self> {
        let mut drv = Icm42670::new(i2c, Address::Primary).map_err(err)?;
        drv.set_power_mode(PowerMode::SixAxisLowNoise)
            .map_err(err)?;
        drv.set_gyro_range(GyroRange::Deg2000).map_err(err)?;
        drv.set_gyro_odr(GyroOdr::Hz200).map_err(err)?;
        drv.set_accel_range(AccelRange::G4).map_err(err)?;
        drv.set_accel_odr(AccelOdr::Hz200).map_err(err)?;
        Ok(Self { drv })
    }

    /// 자이로 각속도 (deg/sec)
    pub fn gyro_dps(&mut self) -> anyhow::Result<V3> {
        let g = self.drv.gyro_norm().map_err(err)?;
        Ok(V3::new(g.x, g.y, g.z))
    }

    /// 가속도 (g). 정지 시 "위" 방향을 가리킨다 (수평으로 두면 z ≈ +1).
    pub fn accel_g(&mut self) -> anyhow::Result<V3> {
        let a = self.drv.accel_norm().map_err(err)?;
        Ok(V3::new(a.x, a.y, a.z))
    }

    /// 정지 상태를 가정하고 자이로 바이어스를 측정한다.
    ///
    /// 전원 인가 직후에는 자이로 스핀업 때문에 풀스케일 포화값(±2000dps)이
    /// 나온다. 포화 샘플이 하나만 평균에 섞여도 바이어스가 10dps 단위로
    /// 틀어져 커서가 흐르므로, 앞쪽 `discard`개를 버리는 것에 더해
    /// 정지 상태로 볼 수 없는 샘플은 평균에서 제외한다.
    ///
    /// 받아들인 샘플의 흔들림 폭(최대−최소)이 어느 축이든 `max_spread_dps`를 넘으면
    /// 보드가 움직인 것으로 보고 실패한다. 손에 든 채 재캘리브레이션하면 20dps 문턱
    /// 아래의 움직임이 평균에 섞여 영점이 4dps씩 틀어진 적이 있다.
    pub fn calibrate_gyro(
        &mut self,
        discard: usize,
        samples: usize,
        max_spread_dps: f32,
    ) -> anyhow::Result<V3> {
        use esp_idf_svc::hal::delay::FreeRtos;

        /// 정지 상태로 인정하는 각속도 상한(dps). 이 값을 넘는 샘플은
        /// 스핀업 잔재이거나 사용자가 보드를 건드린 것이다.
        const REST_LIMIT_DPS: f32 = 20.0;
        /// 유효 샘플을 모으기 위한 최대 시도 횟수
        const MAX_ATTEMPTS: usize = 10;

        for _ in 0..discard {
            let _ = self.drv.gyro_norm();
            FreeRtos::delay_ms(5);
        }

        let (mut sx, mut sy, mut sz) = (0.0f32, 0.0f32, 0.0f32);
        let mut lo = V3::new(f32::MAX, f32::MAX, f32::MAX);
        let mut hi = V3::new(f32::MIN, f32::MIN, f32::MIN);
        let (mut kept, mut rejected) = (0usize, 0usize);
        for _ in 0..samples * MAX_ATTEMPTS {
            if kept >= samples {
                break;
            }
            let g = self.gyro_dps()?;
            if g.x.abs() > REST_LIMIT_DPS
                || g.y.abs() > REST_LIMIT_DPS
                || g.z.abs() > REST_LIMIT_DPS
            {
                rejected += 1;
            } else {
                sx += g.x;
                sy += g.y;
                sz += g.z;
                lo = V3::new(lo.x.min(g.x), lo.y.min(g.y), lo.z.min(g.z));
                hi = V3::new(hi.x.max(g.x), hi.y.max(g.y), hi.z.max(g.z));
                kept += 1;
            }
            FreeRtos::delay_ms(5);
        }

        if kept == 0 {
            anyhow::bail!("캘리브레이션 실패: 정지 샘플을 얻지 못함 (보드를 움직이지 마세요)");
        }
        if rejected > 0 {
            log::info!("캘리브레이션: {rejected}개 샘플 제외 (움직임/스핀업)");
        }
        let spread = hi - lo;
        if spread.x > max_spread_dps || spread.y > max_spread_dps || spread.z > max_spread_dps {
            anyhow::bail!(
                "캘리브레이션 실패: 보드가 움직였습니다 (흔들림 폭 {:.1}/{:.1}/{:.1}dps)",
                spread.x,
                spread.y,
                spread.z
            );
        }
        let n = kept as f32;
        Ok(V3::new(sx / n, sy / n, sz / n))
    }
}

fn err<E: core::fmt::Debug>(e: E) -> anyhow::Error {
    anyhow::anyhow!("IMU 오류: {e:?}")
}
