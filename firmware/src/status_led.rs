// ws2812-esp32-rmt-driver 0.14가 esp-idf-hal의 `rmt-legacy` 피처를 강제하므로
// 구 RMT API(CHANNEL0)를 쓸 수밖에 없다. 드라이버가 신규 API로 옮겨가면 제거.
#![allow(deprecated)]

use esp_idf_svc::hal::{gpio::Gpio2, rmt::CHANNEL0};
use esp_idf_svc::sys;
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
    /// 연결됨 - 상대 이동 모드
    Connected,
    /// 연결됨 - 절대 좌표 모드
    ConnectedAbsolute,
    /// 화면 캘리브레이션 1단계 - 왼쪽 위 모서리를 가리키고 좌클릭
    ScreenCalTopLeft,
    /// 화면 캘리브레이션 2단계 - 오른쪽 아래 모서리를 가리키고 좌클릭
    ScreenCalBottomRight,
    /// 버튼 딥슬립 준비 - 손을 떼면 잠든다
    SleepArmed,
}

impl Status {
    fn color(self) -> RGB8 {
        match self {
            Status::Calibrating => RGB8::new(255, 140, 0), // 주황
            Status::Advertising => RGB8::new(0, 80, 255),  // 파랑
            Status::Connected => RGB8::new(0, 255, 60),    // 초록
            Status::ConnectedAbsolute => RGB8::new(0, 200, 255), // 하늘
            Status::ScreenCalTopLeft => RGB8::new(220, 0, 255), // 보라
            Status::ScreenCalBottomRight => RGB8::new(0, 200, 255), // 하늘 (깜빡임으로 구분)
            Status::SleepArmed => RGB8::new(255, 0, 0),    // 빨강
        }
    }
}

/// 온보드 WS2812(GPIO2) 상태 표시등.
pub struct StatusLed<'d> {
    drv: Ws2812Esp32Rmt<'d>,
    /// 마지막으로 실제 출력한 색 - 같으면 재전송하지 않는다
    last: Option<RGB8>,
    /// 전송하는 동안 APB 클럭을 최대(80MHz)로 묶어 두는 전원 관리 잠금.
    ///
    /// 구 RMT 드라이버는 이 잠금을 스스로 잡지 않는다. 그런데 전송이 끝나기를 태스크를
    /// 재운 채 기다리므로, 그 사이 칩이 라이트 슬립에 들어가 RMT 클럭이 멈추고 "전송 끝"
    /// 인터럽트로는 칩이 깨어나지 않아 펌웨어가 그대로 멈췄다 (충전기에서 파랑 고정, BLE
    /// 시작 전). 클럭 자동 조절로 APB가 40MHz로 내려가면 WS2812 타이밍도 어긋난다.
    pm_lock: sys::esp_pm_lock_handle_t,
}

impl<'d> StatusLed<'d> {
    pub fn new(channel: CHANNEL0<'d>, pin: Gpio2<'d>) -> anyhow::Result<Self> {
        let drv = Ws2812Esp32Rmt::new(channel, pin)
            .map_err(|e| anyhow::anyhow!("WS2812 초기화 실패: {e:?}"))?;
        let mut pm_lock: sys::esp_pm_lock_handle_t = core::ptr::null_mut();
        sys::esp!(unsafe {
            sys::esp_pm_lock_create(
                sys::esp_pm_lock_type_t_ESP_PM_APB_FREQ_MAX,
                0,
                c"ws2812".as_ptr(),
                &mut pm_lock,
            )
        })?;
        let mut led = Self {
            drv,
            last: None,
            pm_lock,
        };
        // 딥슬립에서 깨어나 핀 고정을 풀 때 데이터선이 잠깐 떠서 WS2812가 비트 하나를
        // 받아 둘 수 있다. 그러면 다음 색이 한 비트 밀려 나오므로 검정을 한 번 보내 비운다.
        led.off();
        led.last = None;
        Ok(led)
    }

    /// 상태를 표시한다. `blink_on`이 false면 소등(깜빡임 표현용).
    pub fn set(&mut self, status: Status, blink_on: bool) {
        let c = if blink_on {
            scale(status.color())
        } else {
            RGB8::new(0, 0, 0)
        };
        self.write(c);
    }

    /// 끈다 (대기 모드, 딥슬립 직전).
    pub fn off(&mut self) {
        self.write(RGB8::new(0, 0, 0));
    }

    fn write(&mut self, c: RGB8) {
        if self.last == Some(c) {
            return;
        }
        self.last = Some(c);
        unsafe { sys::esp_pm_lock_acquire(self.pm_lock) };
        let result = self.drv.write_nocopy([c]);
        unsafe { sys::esp_pm_lock_release(self.pm_lock) };
        if let Err(e) = result {
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
