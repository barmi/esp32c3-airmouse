//! 절대 좌표 포인팅: 보드가 가리키는 각도를 화면 위치로 바꾼다.
//!
//! 상대 이동 모드는 OS 포인터 가속과 화면 끝 클램프 때문에 각도와 커서 위치의
//! 대응이 원리적으로 유지되지 않는다. 여기서는 가리키는 각도를 추적해 사용자가
//! 직접 지정한 화면 모서리 기준으로 HID 절대 좌표를 만든다.
//!
//! ESP-IDF에 의존하지 않는 순수 계산이라 호스트에서도 검증할 수 있다.

use core::f32::consts::PI;

/// HID 절대 좌표 최댓값. 리포트 디스크립터의 LOGICAL_MAXIMUM과 같아야 한다.
pub const ABS_MAX: u16 = 32767;

/// 화면 캘리브레이션에서 받아들이는 최소 범위(도). 두 모서리를 거의 같은 곳에서
/// 찍으면 아주 작은 손떨림이 화면 전체로 증폭되므로 거부한다.
const MIN_SPAN_DEG: f32 = 2.0;

/// 최근 각도 기록 길이 (틱). 버튼을 누르는 순간의 흔들림을 피해 그 직전 값을
/// 꺼내 쓰기 위한 것이라 수십 ms면 충분하다.
const HISTORY: usize = 16;

/// 1€ 필터 (Casiez, Roussel, Vogel, CHI 2012).
///
/// 느리게 움직일 때는 강하게 걸러 손떨림을 없애고, 빠르게 움직일 때는 약하게
/// 걸러 지연을 줄인다. 차단 주파수가 움직임 속도에 비례해 올라간다.
pub struct OneEuro {
    min_cutoff_hz: f32,
    beta: f32,
    d_cutoff_hz: f32,
    prev: Option<f32>,
    d_prev: f32,
}

impl OneEuro {
    /// - `min_cutoff_hz`: 정지 시 차단 주파수. 낮출수록 떨림이 줄고 느린 동작이 굼떠진다.
    /// - `beta`: 속도에 따라 차단 주파수를 올리는 비율 (입력 단위/초 당 Hz).
    ///   높일수록 빠른 동작의 지연이 줄어든다.
    /// - `d_cutoff_hz`: 속도 추정 자체를 거르는 차단 주파수.
    pub fn new(min_cutoff_hz: f32, beta: f32, d_cutoff_hz: f32) -> Self {
        Self {
            min_cutoff_hz,
            beta,
            d_cutoff_hz,
            prev: None,
            d_prev: 0.0,
        }
    }

    fn alpha(cutoff_hz: f32, dt: f32) -> f32 {
        let tau = 1.0 / (2.0 * PI * cutoff_hz);
        1.0 / (1.0 + tau / dt)
    }

    pub fn filter(&mut self, x: f32, dt: f32) -> f32 {
        let Some(prev) = self.prev else {
            self.prev = Some(x);
            return x;
        };
        if dt <= 0.0 {
            return prev;
        }
        let a_d = Self::alpha(self.d_cutoff_hz, dt);
        let d = a_d * ((x - prev) / dt) + (1.0 - a_d) * self.d_prev;
        let a = Self::alpha(self.min_cutoff_hz + self.beta * d.abs(), dt);
        let out = a * x + (1.0 - a) * prev;
        self.prev = Some(out);
        self.d_prev = d;
        out
    }

    /// 다음 입력을 그대로 받아들이게 한다 (큰 불연속 뒤에 지연이 끌리지 않도록).
    pub fn reset(&mut self) {
        self.prev = None;
        self.d_prev = 0.0;
    }
}

/// 2점 화면 캘리브레이션: 사용자가 화면 왼쪽 위와 오른쪽 아래 모서리를 가리킨
/// 각도. 각도의 부호 방향과 무관하게 동작한다 (오른쪽으로 돌릴 때 각도가
/// 줄어드는 보드 방향이어도 비율 계산이 알아서 맞춘다).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScreenCal {
    yaw_left: f32,
    yaw_right: f32,
    el_top: f32,
    el_bottom: f32,
}

impl ScreenCal {
    /// (yaw, 고도각) 두 점으로 만든다. 범위가 너무 좁으면 None.
    pub fn from_corners(top_left: (f32, f32), bottom_right: (f32, f32)) -> Option<Self> {
        let width = (bottom_right.0 - top_left.0).abs();
        let height = (bottom_right.1 - top_left.1).abs();
        if width < MIN_SPAN_DEG || height < MIN_SPAN_DEG {
            return None;
        }
        Some(Self {
            yaw_left: top_left.0,
            yaw_right: bottom_right.0,
            el_top: top_left.1,
            el_bottom: bottom_right.1,
        })
    }

    /// 화면이 차지하는 각도 (가로, 세로).
    pub fn span_deg(&self) -> (f32, f32) {
        (
            (self.yaw_right - self.yaw_left).abs(),
            (self.el_bottom - self.el_top).abs(),
        )
    }

    /// 각도를 HID 절대 좌표(0..=ABS_MAX)로. 화면 밖을 가리키면 가장자리에 붙는다.
    pub fn position(&self, angles: (f32, f32)) -> (u16, u16) {
        let nx = (angles.0 - self.yaw_left) / (self.yaw_right - self.yaw_left);
        let ny = (angles.1 - self.el_top) / (self.el_bottom - self.el_top);
        (to_abs_axis(nx), to_abs_axis(ny))
    }

    /// 지금 가리키는 방향이 화면 중앙이 되도록 기준을 옮긴다. 범위(크기)는 유지.
    /// 좌우 각도는 자력계가 없어 서서히 흐르므로 이걸로 바로잡는다.
    pub fn recenter(&mut self, angles: (f32, f32)) {
        let d_yaw = angles.0 - (self.yaw_left + self.yaw_right) / 2.0;
        self.yaw_left += d_yaw;
        self.yaw_right += d_yaw;
        let d_el = angles.1 - (self.el_top + self.el_bottom) / 2.0;
        self.el_top += d_el;
        self.el_bottom += d_el;
    }
}

fn to_abs_axis(n: f32) -> u16 {
    (n.clamp(0.0, 1.0) * ABS_MAX as f32).round() as u16
}

/// 각도 추적 설정. 값은 mapping.rs의 튜닝 상수에서 온다.
pub struct TrackerConfig {
    /// 이보다 느린 좌우 회전은 적분하지 않는다 (정지 시 바이어스 잔여로 흐르는 것 방지)
    pub yaw_gate_dps: f32,
    pub min_cutoff_hz: f32,
    pub beta: f32,
    pub d_cutoff_hz: f32,
}

/// 가리키는 각도 추적기.
///
/// - 좌우: 기울기 보정된 좌우 회전 속도를 적분한다. 절대 기준(자력계)이 없어
///   서서히 흐르므로 리센터로 바로잡는다.
/// - 상하: 중력 기준 고도각을 그대로 쓴다. 흐르지 않는다.
///
/// 두 값 모두 1€ 필터로 손떨림을 거른다.
pub struct AngleTracker {
    yaw_gate_dps: f32,
    yaw: f32,
    f_yaw: OneEuro,
    f_el: OneEuro,
    current: (f32, f32),
    history: [(f32, f32); HISTORY],
    head: usize,
}

impl AngleTracker {
    pub fn new(cfg: TrackerConfig) -> Self {
        Self {
            yaw_gate_dps: cfg.yaw_gate_dps,
            yaw: 0.0,
            f_yaw: OneEuro::new(cfg.min_cutoff_hz, cfg.beta, cfg.d_cutoff_hz),
            f_el: OneEuro::new(cfg.min_cutoff_hz, cfg.beta, cfg.d_cutoff_hz),
            current: (0.0, 0.0),
            history: [(0.0, 0.0); HISTORY],
            head: 0,
        }
    }

    /// 매 틱 호출. `yaw_rate_dps`는 기울기 보정된 좌우 회전 속도, `elevation_deg`는
    /// 중력 기준 고도각.
    pub fn update(&mut self, yaw_rate_dps: f32, elevation_deg: f32, dt: f32) {
        if yaw_rate_dps.abs() >= self.yaw_gate_dps {
            self.yaw += yaw_rate_dps * dt;
        }
        self.current = (
            self.f_yaw.filter(self.yaw, dt),
            self.f_el.filter(elevation_deg, dt),
        );
        self.head = (self.head + 1) % HISTORY;
        self.history[self.head] = self.current;
    }

    /// 걸러진 현재 각도 (yaw, 고도각).
    pub fn angles(&self) -> (f32, f32) {
        self.current
    }

    /// `ticks` 틱 전의 각도. 버튼을 누르는 순간 보드가 흔들리므로, 누르기 직전에
    /// 가리키던 곳을 알고 싶을 때 쓴다.
    pub fn angles_before(&self, ticks: usize) -> (f32, f32) {
        let back = ticks.min(HISTORY - 1);
        self.history[(self.head + HISTORY - back) % HISTORY]
    }

    /// 루프가 오래 멈췄던 직후(재캘리브레이션 등) 필터가 옛 값에 끌리지 않게 한다.
    pub fn reset_filters(&mut self) {
        self.f_yaw.reset();
        self.f_el.reset();
    }
}
