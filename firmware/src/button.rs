use esp_idf_svc::hal::gpio::{Gpio9, Input, PinDriver, Pull};

/// 디바운스에 필요한 연속 동일 샘플 수. 10ms 루프 기준 3개 = 30ms.
const DEBOUNCE_SAMPLES: u8 = 3;

/// BOOT 버튼(GPIO9, active-low)을 마우스 좌클릭으로 쓰기 위한 래퍼.
///
/// GPIO9는 부팅 스트래핑 핀이라 리셋 중 누르고 있으면 다운로드 모드로
/// 들어가지만, 동작 중 입력으로 쓰는 데는 문제가 없다 (보드에 풀업 있음).
pub struct Button<'d> {
    pin: PinDriver<'d, Input>,
    stable_pressed: bool,
    counter: u8,
}

impl<'d> Button<'d> {
    pub fn new(gpio9: Gpio9<'d>) -> anyhow::Result<Self> {
        let pin = PinDriver::input(gpio9, Pull::Up)?;
        Ok(Self {
            pin,
            stable_pressed: false,
            counter: 0,
        })
    }

    /// 루프 주기마다 호출. 디바운스된 눌림 상태를 돌려준다.
    pub fn update(&mut self) -> bool {
        let raw_pressed = self.pin.is_low();
        if raw_pressed == self.stable_pressed {
            self.counter = 0;
        } else {
            self.counter += 1;
            if self.counter >= DEBOUNCE_SAMPLES {
                self.stable_pressed = raw_pressed;
                self.counter = 0;
            }
        }
        self.stable_pressed
    }
}
