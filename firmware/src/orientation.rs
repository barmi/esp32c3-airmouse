//! 중력 방향 추정과 기울기 보정.
//!
//! 자이로는 보드 기준 축으로 회전을 잰다. 손목이 보드를 옆으로 기울인 채(roll)
//! 좌우로 돌리면 실제 좌우 회전이 보드의 Z축과 X축에 나눠 담겨, 커서가 기울인
//! 각도만큼 비스듬히 간다. 가속도계로 중력 방향을 알아내 회전 속도를 실제
//! 좌우·상하 기준으로 다시 나누면 쥔 각도와 무관하게 방향이 맞는다.
//!
//! ESP-IDF에 의존하지 않는 순수 계산이라 호스트에서도 검증할 수 있다.

use core::f32::consts::PI;
use core::ops::{Add, Mul, Sub};

const DEG2RAD: f32 = PI / 180.0;
const RAD2DEG: f32 = 180.0 / PI;

/// 보드가 가리키는 축 (USB 커넥터 방향). roll 회전축으로, 실측으로 확정된 Y축.
const POINT_AXIS: V3 = V3::new(0.0, 1.0, 0.0);

/// 가속도계 쪽으로 끌어당기는 강도 (틱당). 100Hz에서 0.02면 시상수 약 0.5초.
const ACCEL_GAIN: f32 = 0.02;

/// 가속도 크기가 1g에서 이만큼 벗어나면 손 움직임의 선형 가속이 섞인 것으로
/// 보고 그 틱은 가속도계 보정을 건너뛴다.
const ACCEL_GATE_G: f32 = 0.15;

/// 위나 아래를 똑바로 가리키면 상하 회전축이 정의되지 않는다(가리키는 축과
/// "위"가 평행). 그 근처에서는 보드 X축으로 대신한다.
const DEGENERATE_LIMIT: f32 = 0.2;

/// 바이어스 추적: 보정 후 각속도가 모든 축에서 이보다 작아야 정지로 본다(dps).
/// 손에 쥔 채 아주 천천히 돌리는 동작까지 정지로 오인하면 그 회전이 바이어스로
/// 흡수되어 커서가 덜 움직이므로 낮게 잡는다.
const STILL_DPS: f32 = 0.8;

/// 바이어스 추적: 가속도 크기가 1g에서 이만큼 안이어야 정지로 본다.
const STILL_ACCEL_G: f32 = 0.05;

/// 바이어스 추적: 정지 조건이 이만큼 연속돼야 갱신을 시작한다 (틱, 0.5초).
const STILL_TICKS: u32 = 50;

/// 바이어스 추적: 정지 중 틱당 갱신 비율. 100Hz에서 0.002면 시상수 약 5초.
const BIAS_RATE: f32 = 0.002;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct V3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl V3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn dot(self, o: V3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: V3) -> V3 {
        V3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn norm(self) -> f32 {
        self.dot(self).sqrt()
    }

    fn normalized(self) -> V3 {
        let n = self.norm();
        if n > 0.0 {
            self * (1.0 / n)
        } else {
            self
        }
    }
}

impl Add for V3 {
    type Output = V3;
    fn add(self, o: V3) -> V3 {
        V3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for V3 {
    type Output = V3;
    fn sub(self, o: V3) -> V3 {
        V3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f32> for V3 {
    type Output = V3;
    fn mul(self, k: f32) -> V3 {
        V3::new(self.x * k, self.y * k, self.z * k)
    }
}

/// 중력 방향 추정기 (상보 필터).
///
/// 자이로로 "위" 벡터를 매 틱 회전시켜 빠른 움직임을 따라가고, 가속도계로
/// 천천히 끌어당겨 자이로 적분 오차를 지운다. 가속도계만 쓰면 손을 휘두를 때의
/// 선형 가속에 흔들리고, 자이로만 쓰면 오차가 쌓인다.
pub struct Orientation {
    /// 보드 기준 "위" 방향 단위벡터. 정지 시 가속도계가 가리키는 방향과 같다.
    up: V3,
    initialized: bool,
}

impl Default for Orientation {
    fn default() -> Self {
        Self {
            up: V3::new(0.0, 0.0, 1.0),
            initialized: false,
        }
    }
}

impl Orientation {
    /// 매 틱 호출한다. `gyro_dps`는 바이어스를 뺀 각속도, `accel_g`는 가속도,
    /// `dt`는 지난 호출 이후 경과 시간(초).
    pub fn update(&mut self, gyro_dps: V3, accel_g: V3, dt: f32) {
        let a_norm = accel_g.norm();
        if !self.initialized {
            // 첫 샘플로 "위"를 바로 잡는다. (0,0,1)에서 천천히 수렴시키면 기울여
            // 든 채 시작했을 때 몇 초간 방향이 틀린다.
            if a_norm > 0.5 {
                self.up = accel_g * (1.0 / a_norm);
                self.initialized = true;
            }
            return;
        }

        // 보드가 ω로 돌면, 보드 기준으로 본 고정된 "위"는 반대로 돈다: du/dt = u × ω
        let w = gyro_dps * DEG2RAD;
        self.up = (self.up + self.up.cross(w) * dt).normalized();

        // 자이로 적분 오차를 가속도계로 천천히 지운다. 손을 휘두르는 중이면 건너뛴다.
        if (a_norm - 1.0).abs() < ACCEL_GATE_G {
            let a_dir = accel_g * (1.0 / a_norm);
            self.up = (self.up * (1.0 - ACCEL_GAIN) + a_dir * ACCEL_GAIN).normalized();
        }
    }

    /// 실제 좌우(yaw)·상하(pitch) 회전 속도(dps).
    ///
    /// 좌우는 "위" 축 둘레의 회전, 상하는 수평이면서 가리키는 방향에 수직인 축
    /// 둘레의 회전이다. 보드가 수평이면 기존 축(보드 Z, X)과 같고, 기울여 쥐면
    /// 방향만 바로잡힌다. 크기는 보존되므로 감도·데드존 동작은 그대로다.
    pub fn world_rates(&self, gyro_dps: V3) -> (f32, f32) {
        let yaw = gyro_dps.dot(self.up);
        let h = POINT_AXIS.cross(self.up);
        let hn = h.norm();
        let pitch = if hn > DEGENERATE_LIMIT {
            gyro_dps.dot(h * (1.0 / hn))
        } else {
            gyro_dps.x
        };
        (yaw, pitch)
    }

    /// 보드를 옆으로 기울인 각도(도). 수평이면 0. 상태 확인용.
    pub fn roll_deg(&self) -> f32 {
        self.up.x.atan2(self.up.z) * RAD2DEG
    }

    /// 가리키는 축의 고도각(도). 위를 가리킬수록 한쪽으로 커진다. 중력 기준이라
    /// 시간이 지나도 흐르지 않는다. 부호는 보드 방향에 따라 다를 수 있지만 2점
    /// 화면 캘리브레이션이 흡수한다.
    pub fn elevation_deg(&self) -> f32 {
        POINT_AXIS.dot(self.up).clamp(-1.0, 1.0).asin() * RAD2DEG
    }
}

/// 정지 상태에서 자이로 바이어스를 계속 추적한다.
///
/// 부팅 캘리브레이션 뒤에도 수 0.1°/s의 바이어스가 남고, 보드가 데워지면 조금씩
/// 바뀐다. 상대 모드에선 데드존이 가려주지만, 각도를 적분하는 절대 모드에선 이게
/// 곧 좌우 드리프트 속도다. 보드가 가만히 있는 동안 남은 값을 천천히 흡수한다.
#[derive(Default)]
pub struct BiasTracker {
    still_ticks: u32,
}

impl BiasTracker {
    /// `raw_dps`는 바이어스를 빼기 전 자이로 값. 정지가 이어지면 `bias`를 갱신한다.
    pub fn update(&mut self, raw_dps: V3, bias: &mut V3, accel_g: V3) {
        let c = raw_dps - *bias;
        let still = c.x.abs() < STILL_DPS
            && c.y.abs() < STILL_DPS
            && c.z.abs() < STILL_DPS
            && (accel_g.norm() - 1.0).abs() < STILL_ACCEL_G;
        self.still_ticks = if still {
            self.still_ticks.saturating_add(1)
        } else {
            0
        };
        if self.still_ticks >= STILL_TICKS {
            *bias = *bias + c * BIAS_RATE;
        }
    }
}
