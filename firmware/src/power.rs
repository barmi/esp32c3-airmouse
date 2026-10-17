//! 절전 정책: 언제 대기 모드와 딥슬립에 들어갈지 정한다.
//!
//! ESP-IDF에 의존하지 않는 순수 계산이라 호스트에서 테스트한다. 실제 슬립 진입은
//! `sleep.rs`가 맡는다.
//!
//! - **활성**: 연결돼 있고 최근 `IDLE_AFTER_MS` 안에 움직이거나 버튼을 눌렀다.
//! - **대기**: 연결돼 있지만 그동안 움직임이 없거나, 아직 연결되지 않았다. 연결 중이면
//!   LED를 끄고, 상대 모드에서는 루프를 늦춘다. 움직이거나 버튼을 누르면 바로 활성이다.
//! - **딥슬립**: 오래 쓰지 않았거나 연결하지 못한 채 오래 지났다. 클릭 버튼으로 깨운다.
//!
//! 틱 사이의 자동 라이트 슬립은 상태와 상관없이 ESP-IDF가 알아서 한다.

/// 이 시간 동안 움직임·버튼이 없으면 대기 모드 (ms)
pub const IDLE_AFTER_MS: u64 = 5_000;

/// 마지막으로 움직이거나 버튼을 누른 뒤 이만큼 지나면 딥슬립 (ms). BLE 연결이 끊긴다.
pub const DEEP_SLEEP_AFTER_MS: u64 = 10 * 60_000;

/// 호스트에 연결되지 않은 채 이만큼 지나면 딥슬립 (ms). 광고를 계속하는 전력을 아낀다.
pub const UNCONNECTED_SLEEP_AFTER_MS: u64 = 5 * 60_000;

/// 바이어스를 뺀 각속도 크기가 이보다 크면 움직인 것으로 본다 (dps).
/// 커서 데드존과 같은 값이라, 커서가 움직일 만한 회전이면 대기에서 깬다.
pub const MOTION_DPS: f32 = 2.0;

/// 대기 모드의 루프 주기 (ms). 상대 모드에서만 쓴다 — 절대 모드는 각도를 적분하므로
/// 샘플을 성기게 읽으면 움직이기 시작하는 순간의 각도가 어긋난다.
pub const IDLE_LOOP_PERIOD_MS: u32 = 50;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PowerState {
    Active,
    Idle,
    Sleep,
}

impl PowerState {
    pub fn name(self) -> &'static str {
        match self {
            PowerState::Active => "활성",
            PowerState::Idle => "대기",
            PowerState::Sleep => "딥슬립",
        }
    }
}

pub struct PowerPolicy {
    last_activity_ms: u64,
    unconnected_since_ms: Option<u64>,
}

impl PowerPolicy {
    pub fn new(now_ms: u64) -> Self {
        Self {
            last_activity_ms: now_ms,
            unconnected_since_ms: None,
        }
    }

    /// 매 틱 호출한다.
    ///
    /// - `active`: 이번 틱에 움직였거나 버튼을 눌렀는지
    /// - `connected`: BLE 호스트와 연결돼 있는지
    /// - `may_sleep`: 딥슬립을 허용하는지 (USB로 컴퓨터에 연결돼 있거나 화면
    ///   캘리브레이션 중이면 false)
    pub fn update(
        &mut self,
        now_ms: u64,
        active: bool,
        connected: bool,
        may_sleep: bool,
    ) -> PowerState {
        if active {
            self.last_activity_ms = now_ms;
        }
        if connected {
            self.unconnected_since_ms = None;
        } else if self.unconnected_since_ms.is_none() {
            self.unconnected_since_ms = Some(now_ms);
        }

        let idle_for = now_ms.saturating_sub(self.last_activity_ms);
        if may_sleep {
            let unconnected_for = self
                .unconnected_since_ms
                .map_or(0, |t| now_ms.saturating_sub(t));
            if idle_for >= DEEP_SLEEP_AFTER_MS || unconnected_for >= UNCONNECTED_SLEEP_AFTER_MS {
                return PowerState::Sleep;
            }
        }
        if connected && idle_for < IDLE_AFTER_MS {
            PowerState::Active
        } else {
            PowerState::Idle
        }
    }

    /// 마지막으로 움직이거나 버튼을 누른 뒤 지난 시간 (ms)
    pub fn idle_for_ms(&self, now_ms: u64) -> u64 {
        now_ms.saturating_sub(self.last_activity_ms)
    }
}
