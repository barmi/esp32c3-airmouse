use esp_idf_svc::hal::{gpio::Gpio2, rmt::CHANNEL0};
use ws2812_esp32_rmt_driver::{Ws2812Esp32Rmt, RGB8};

/// 밝기 스케일. WS2812는 매우 밝아서 실내에서는 이 정도면 충분하고
/// 전류도 아낀다 (0.0~1.0).
const BRIGHTNESS: f32 = 0.08;

/// 에어마우스 동작 상태.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// 자이로 캘리브레이션 중 - 움직이면 안 되는 구간
    Calibrating,
    /// BLE 광고 중 (호스트 미연결)
    Advertising,
    /// 연결됨 - 정상 동작
    Connected,
}

impl Status {
    fn color(self) -> RGB8 {
        match self {
            Status::Calibrating => RGB8::new(255, 140, 0), // 주황
            Status::Advertising => RGB8::new(0, 80, 255),  // 파랑
            Status::Connected => RGB8::new(0, 255, 60),    // 초록
        }
    }
}

/// 온보드 WS2812(GPIO2) 상태 표시등.
pub struct StatusLed<'d> {
    drv: Ws2812Esp32Rmt<'d>,
    /// 마지막으로 실제 출력한 (상태, 켜짐) 조합 - 변화 없으면 재전송하지 않는다
    last: Option<(Status, bool)>,
}

impl<'d> StatusLed<'d> {
    pub fn new(channel: CHANNEL0<'d>, pin: Gpio2<'d>) -> anyhow::Result<Self> {
        let drv = Ws2812Esp32Rmt::new(channel, pin)
            .map_err(|e| anyhow::anyhow!("WS2812 초기화 실패: {e:?}"))?;
        Ok(Self { drv, last: None })
    }

    /// 상태를 표시한다. `blink_on`이 false면 소등(깜빡임 표현용).
    pub fn set(&mut self, status: Status, blink_on: bool) {
        if self.last == Some((status, blink_on)) {
            return;
        }
        self.last = Some((status, blink_on));

        let c = if blink_on {
            scale(status.color())
        } else {
            RGB8::new(0, 0, 0)
        };
        if let Err(e) = self.drv.write_nocopy([c]) {
            log::warn!("WS2812 쓰기 실패: {e:?}");
        }
    }
}

fn scale(c: RGB8) -> RGB8 {
    RGB8::new(
        (c.r as f32 * BRIGHTNESS) as u8,
        (c.g as f32 * BRIGHTNESS) as u8,
        (c.b as f32 * BRIGHTNESS) as u8,
    )
}
