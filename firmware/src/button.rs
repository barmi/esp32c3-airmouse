use esp_idf_svc::hal::delay::Ets;
use esp_idf_svc::hal::gpio::{Input, InputPin, PinDriver, Pull};

/// 디바운스에 필요한 연속 동일 샘플 수. 10ms 루프 기준 3개 = 30ms.
const DEBOUNCE_SAMPLES: u8 = 3;

/// active-low 택트 스위치 입력.
///
/// 배선은 `GPIO ──[스위치]── GND`이고 내부 풀업을 켜므로 외부 저항이 필요 없다.
/// `PinDriver`가 핀 타입을 지우기 때문에 어느 GPIO를 쓰든 같은 타입이 된다.
pub struct Button<'d> {
    pin: PinDriver<'d, Input>,
    /// 디바운스 전 마지막 핀 상태
    raw: bool,
    pressed: bool,
    prev_pressed: bool,
    counter: u8,
}

impl<'d> Button<'d> {
    pub fn new(pin: impl InputPin + 'd) -> anyhow::Result<Self> {
        let pin = PinDriver::input(pin, Pull::Up)?;
        // 풀업이 핀을 끌어올릴 시간을 준 뒤 읽는다
        Ets::delay_us(50);
        // 이미 눌려 있는 버튼은 눌린 상태로 시작해 '눌린 순간'이 생기지 않게 한다.
        // 딥슬립을 깨운 클릭을 부팅 후 계속 누르고 있어도 새 클릭으로 처리되지 않는다.
        let raw = pin.is_low();
        Ok(Self {
            pin,
            raw,
            pressed: raw,
            prev_pressed: raw,
            counter: 0,
        })
    }

    /// 루프 주기마다 한 번 호출해 디바운스 상태를 갱신한다.
    pub fn update(&mut self) {
        let raw = self.pin.is_low();
        self.raw = raw;
        self.prev_pressed = self.pressed;
        if raw == self.pressed {
            self.counter = 0;
        } else {
            self.counter += 1;
            if self.counter >= DEBOUNCE_SAMPLES {
                self.pressed = raw;
                self.counter = 0;
            }
        }
    }

    /// 디바운스된 눌림 상태. 누르고 있는 동안 계속 true (드래그에 사용).
    pub fn pressed(&self) -> bool {
        self.pressed
    }

    /// 디바운스 전 핀 상태. 대기 모드에서 누르는 즉시 깨어나는 데 쓴다.
    pub fn raw_pressed(&self) -> bool {
        self.raw
    }

    /// 방금 눌린 순간에만 true. 한 번만 실행할 동작에 사용.
    pub fn just_pressed(&self) -> bool {
        self.pressed && !self.prev_pressed
    }
}

/// 버튼을 누르고 있는 동안 키보드 키처럼 반복 신호를 만든다.
///
/// 스크롤 버튼용이다. 톡 누르면 한 칸만 움직이고, 계속 누르고 있으면
/// 잠시 후부터 연속으로 스크롤된다.
pub struct Repeater {
    delay_ticks: u32,
    interval_ticks: u32,
    held_ticks: u32,
    next_tick: u32,
}

impl Repeater {
    /// `loop_period_ms`는 메인 루프 주기. ms 설정을 루프 틱으로 환산한다.
    pub fn new(delay_ms: u32, interval_ms: u32, loop_period_ms: u32) -> Self {
        Self {
            delay_ticks: delay_ms.div_ceil(loop_period_ms),
            interval_ticks: interval_ms.div_ceil(loop_period_ms).max(1),
            held_ticks: 0,
            next_tick: 0,
        }
    }

    /// 이번 틱에 한 칸 움직여야 하면 true.
    pub fn tick(&mut self, button: &Button) -> bool {
        if !button.pressed() {
            self.held_ticks = 0;
            return false;
        }
        if button.just_pressed() {
            // 누르는 즉시 한 칸 (톡 누름에 바로 반응)
            self.held_ticks = 0;
            self.next_tick = self.delay_ticks;
            return true;
        }
        self.held_ticks += 1;
        if self.held_ticks >= self.next_tick {
            self.next_tick += self.interval_ticks;
            return true;
        }
        false
    }
}

/// 짧게/길게 누름 구분 결과.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Gesture {
    None,
    /// 기준 시간 전에 손을 뗐다 (뗄 때 발생)
    Short,
    /// 기준 시간만큼 누르고 있었다 (손을 떼기 전에 바로 발생)
    Long,
}

/// 한 버튼에 두 가지 동작을 싣기 위한 누름 시간 판별기.
///
/// 길게 누름은 기준 시간에 닿는 순간 바로 발생해 손을 떼기 전에 반응이 온다.
/// 짧게 누름은 뗄 때 판별되므로 누르는 즉시 반응해야 하는 클릭에는 쓰지 않는다.
pub struct HoldGesture {
    long_ticks: u32,
    held_ticks: u32,
    long_fired: bool,
}

impl HoldGesture {
    pub fn new(long_ms: u32, loop_period_ms: u32) -> Self {
        Self {
            long_ticks: long_ms.div_ceil(loop_period_ms),
            held_ticks: 0,
            long_fired: false,
        }
    }

    pub fn tick(&mut self, button: &Button) -> Gesture {
        if button.pressed() {
            self.held_ticks += 1;
            if !self.long_fired && self.held_ticks >= self.long_ticks {
                self.long_fired = true;
                return Gesture::Long;
            }
            return Gesture::None;
        }
        let gesture = if self.held_ticks > 0 && !self.long_fired {
            Gesture::Short
        } else {
            Gesture::None
        };
        self.held_ticks = 0;
        self.long_fired = false;
        gesture
    }
}
