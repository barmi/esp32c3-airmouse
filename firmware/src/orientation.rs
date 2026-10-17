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

/// 방위각 변화율은 좌우 각속도를 cos(고도각)으로 나눈 값이라 위·아래를 크게
/// 가리킬수록 커진다(천정에서 발산). 화면을 가리킬 일이 없는 고도각 60° 이상에서
/// 잡음이 증폭되지 않도록 cos를 이 값 아래로 내리지 않는다.
const MIN_COS_ELEVATION: f32 = 0.5;

/// 바이어스 추적: 정지 판정에 쓰는 최근 샘플 수 (100Hz에서 약 1초).
const STILL_WINDOW: usize = 100;

/// 바이어스 추적: 창 안에서 자이로 원시값의 흔들림 폭(최대−최소)이 모든 축에서
/// 이보다 작아야 정지로 본다(dps). 책상 위 잡음은 0.3~0.5, 손에 쥔 떨림은 1~3 수준.
/// 보정한 값의 크기가 아니라 원시값의 흔들림으로 판정하므로, 영점이 크게 틀어져
/// 있어도 정지를 알아보고 바로잡을 수 있다.
const STILL_GYRO_SPREAD_DPS: f32 = 1.0;

/// 바이어스 추적: 창 안에서 가속도 각 축의 흔들림 폭이 이보다 작아야 정지로 본다(g).
/// 보드가 천천히 돌면 중력 방향이 바뀌므로, 자이로만으로는 영점 오차와 구분되지 않는
/// 느린 회전을 걸러낸다 (단, 수직축 둘레 회전은 중력 방향이 변하지 않아 못 거른다).
const STILL_ACCEL_SPREAD_G: f32 = 0.02;

/// 바이어스 추적: 가속도 크기가 1g에서 이만큼 안이어야 정지로 본다.
const STILL_ACCEL_NORM_G: f32 = 0.05;

/// 바이어스 추적: 정지 중 틱당 창 평균 쪽으로 옮기는 비율. 0.01이면 시상수 약 1초.
const BIAS_RATE: f32 = 0.01;

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

    /// 가리키는 방향이 움직이는 각속도를 좌우·상하 성분으로 나눈다(dps).
    ///
    /// 두 축 모두 가리키는 축에 수직이다. 좌우축은 "위"에서 가리키는 축 성분을 뺀
    /// 방향, 상하축은 수평이면서 가리키는 방향에 수직인 방향이다. 그래서 보드를
    /// 옆으로 기울여 쥐어도 방향이 맞고, 위·아래를 가리킨 채 손목을 비틀어도(가리키는
    /// 축 둘레 회전) 커서가 움직이지 않는다. 보드가 수평이면 기존 축(보드 Z, X)과 같다.
    ///
    /// 좌우축을 "위" 그대로 쓰면 위를 가리킨 채 비틀 때 회전의 sin(고도각)만큼이
    /// 좌우로 새어 든다. 고도각 30°면 절반이다.
    pub fn world_rates(&self, gyro_dps: V3) -> (f32, f32) {
        match self.pointing_frame() {
            Some((h, v, _)) => (gyro_dps.dot(h), gyro_dps.dot(v)),
            None => (gyro_dps.z, gyro_dps.x),
        }
    }

    /// 가리키는 방향의 방위각 변화율(dps).
    ///
    /// 좌우 각속도를 cos(고도각)으로 나눈 값으로, 방위각의 정확한 시간 미분이다.
    /// 그래서 적분해도 움직인 경로에 따라 어긋나지 않는다 — 같은 곳을 가리키면 같은
    /// 각도가 나온다. 수직인 화면에서 가로 위치는 방위각으로 정해지므로 절대 좌표
    /// 모드가 이 값을 적분한다.
    pub fn azimuth_rate(&self, gyro_dps: V3) -> f32 {
        match self.pointing_frame() {
            Some((h, _, c)) => gyro_dps.dot(h) / c.max(MIN_COS_ELEVATION),
            None => 0.0,
        }
    }

    /// (좌우축, 상하축, cos(고도각)). 가리키는 축이 수직에 가까우면 None.
    fn pointing_frame(&self) -> Option<(V3, V3, f32)> {
        let up_perp = self.up - POINT_AXIS * POINT_AXIS.dot(self.up);
        let c = up_perp.norm();
        if c < DEGENERATE_LIMIT {
            return None;
        }
        let inv = 1.0 / c;
        Some((up_perp * inv, POINT_AXIS.cross(self.up) * inv, c))
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
/// 바뀐다. 각도를 적분하는 절대 모드에서는 이게 곧 좌우 드리프트 속도다. 잘못된
/// 재캘리브레이션으로 영점이 크게 틀어진 경우도 보드를 내려놓으면 스스로 바로잡는다.
///
/// 처음에는 "보정한 각속도가 작으면 정지"로 판정했는데, 그러면 영점이 판정 문턱보다
/// 크게 틀어졌을 때 정지해 있어도 움직이는 것으로 보여 영영 보정하지 않았다. 그래서
/// 원시값이 거의 변하지 않는지(흔들림 폭)로 판정하고, 그 구간의 평균으로 영점을 옮긴다.
pub struct BiasTracker {
    gyro: [V3; STILL_WINDOW],
    accel: [V3; STILL_WINDOW],
    len: usize,
    head: usize,
}

impl Default for BiasTracker {
    fn default() -> Self {
        let zero = V3::new(0.0, 0.0, 0.0);
        Self {
            gyro: [zero; STILL_WINDOW],
            accel: [zero; STILL_WINDOW],
            len: 0,
            head: 0,
        }
    }
}

impl BiasTracker {
    /// `raw_dps`는 바이어스를 빼기 전 자이로 값. 최근 창이 정지 상태면 `bias`를 창의
    /// 평균 쪽으로 옮기고 true를 돌려준다.
    pub fn update(&mut self, raw_dps: V3, bias: &mut V3, accel_g: V3) -> bool {
        self.gyro[self.head] = raw_dps;
        self.accel[self.head] = accel_g;
        self.head = (self.head + 1) % STILL_WINDOW;
        self.len = (self.len + 1).min(STILL_WINDOW);
        if self.len < STILL_WINDOW || (accel_g.norm() - 1.0).abs() >= STILL_ACCEL_NORM_G {
            return false;
        }
        let (g_lo, g_hi, g_sum) = span(&self.gyro);
        let (a_lo, a_hi, _) = span(&self.accel);
        let g = g_hi - g_lo;
        let a = a_hi - a_lo;
        let still = g.x < STILL_GYRO_SPREAD_DPS
            && g.y < STILL_GYRO_SPREAD_DPS
            && g.z < STILL_GYRO_SPREAD_DPS
            && a.x < STILL_ACCEL_SPREAD_G
            && a.y < STILL_ACCEL_SPREAD_G
            && a.z < STILL_ACCEL_SPREAD_G;
        if still {
            let mean = g_sum * (1.0 / STILL_WINDOW as f32);
            *bias = *bias + (mean - *bias) * BIAS_RATE;
        }
        still
    }
}

/// 축별 최솟값, 최댓값, 합
fn span(v: &[V3]) -> (V3, V3, V3) {
    let mut lo = v[0];
    let mut hi = v[0];
    let mut sum = V3::new(0.0, 0.0, 0.0);
    for s in v {
        lo = V3::new(lo.x.min(s.x), lo.y.min(s.y), lo.z.min(s.z));
        hi = V3::new(hi.x.max(s.x), hi.y.max(s.y), hi.z.max(s.z));
        sum = sum + *s;
    }
    (lo, hi, sum)
}
