use esp_idf_svc::hal::gpio::{Input, InputPin, PinDriver, Pull};

/// 디바운스에 필요한 연속 동일 샘플 수. 10ms 루프 기준 3개 = 30ms.
const DEBOUNCE_SAMPLES: u8 = 3;

/// active-low 택트 스위치 입력.
///
/// 배선은 `GPIO ──[스위치]── GND`이고 내부 풀업을 켜므로 외부 저항이 필요 없다.
/// `PinDriver`가 핀 타입을 지우기 때문에 어느 GPIO를 쓰든 같은 타입이 된다.
pub struct Button<'d> {
    pin: PinDriver<'d, Input>,
    pressed: bool,
    prev_pressed: bool,
    counter: u8,
}

impl<'d> Button<'d> {
    pub fn new(pin: impl InputPin + 'd) -> anyhow::Result<Self> {
        Ok(Self {
            pin: PinDriver::input(pin, Pull::Up)?,
            pressed: false,
            prev_pressed: false,
            counter: 0,
        })
    }

    /// 루프 주기마다 한 번 호출해 디바운스 상태를 갱신한다.
    pub fn update(&mut self) {
        let raw = self.pin.is_low();
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
