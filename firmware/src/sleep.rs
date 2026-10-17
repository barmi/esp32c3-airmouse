//! ESP-IDF 전원 관리: 자동 라이트 슬립 설정, 딥슬립 진입과 복귀, USB 호스트 감지.
//!
//! 언제 잘지는 `power.rs`가 정하고, 여기서는 칩을 실제로 재우고 깨운다.

use crate::orientation::V3;
use esp_idf_svc::sys::{self, esp, EspError};
use std::sync::atomic::{AtomicU32, Ordering::Relaxed};

/// 딥슬립에서 깨우는 버튼: 좌클릭(GPIO3)·우클릭(GPIO4)·스크롤↓(GPIO5).
/// ESP32-C3는 GPIO0~5만 딥슬립 기상이 되어 스크롤↑(GPIO6)과 BOOT(GPIO9)로는 못 깨운다.
const WAKE_PINS: u64 = (1 << 3) | (1 << 4) | (1 << 5);

/// WS2812 데이터 핀. 딥슬립 동안 떠 있으면 잡음을 데이터로 받아 LED가 켜질 수 있어
/// LOW로 묶어 둔다.
const LED_PIN: sys::gpio_num_t = sys::gpio_num_t_GPIO_NUM_2;

/// 깨어 있을 때의 CPU 최고 클럭과, 할 일이 없을 때 내려가는 최저 클럭 (MHz).
/// 최저값은 크리스털 주파수(40MHz)로, ESP-IDF BLE 절전 예제와 같다.
const CPU_MAX_MHZ: i32 = 160;
const CPU_MIN_MHZ: i32 = 40;

/// RTC 메모리 값이 유효한지 표시하는 값. RTC 메모리는 딥슬립에서 깨어날 때만 유지되고
/// 전원을 넣거나 리셋하면 초기값(0)으로 돌아간다.
const SAVED_MAGIC_VALID: u32 = 0x5A17_B1A5;

#[link_section = ".rtc.data"]
static SAVED_MAGIC: AtomicU32 = AtomicU32::new(0);
#[link_section = ".rtc.data"]
static SAVED_BIAS: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];

/// 부팅 직후 한 번 호출한다. CPU 클럭 자동 조절과 자동 라이트 슬립을 켠다.
///
/// 라이트 슬립은 태스크가 모두 대기 중일 때 ESP-IDF가 알아서 들어가고, 타이머나
/// BLE 이벤트에 맞춰 깬다. USB로 컴퓨터에 연결된 동안은 sdkconfig의
/// `USJ_NO_AUTO_LS_ON_CONNECTION` 때문에 들어가지 않는다 (USB 시리얼 유지).
pub fn init() -> Result<(), EspError> {
    // 딥슬립 전에 걸어 둔 LED 핀 고정을 푼다. 풀지 않으면 RMT가 핀을 구동하지 못한다.
    // 고정한 적이 없으면 아무 일도 하지 않는다.
    unsafe { sys::gpio_hold_dis(LED_PIN) };

    let cfg = sys::esp_pm_config_t {
        max_freq_mhz: CPU_MAX_MHZ,
        min_freq_mhz: CPU_MIN_MHZ,
        light_sleep_enable: true,
    };
    esp!(unsafe { sys::esp_pm_configure(&cfg as *const _ as *const core::ffi::c_void) })
}

/// 라이트 슬립 중에도 LED 데이터선을 깨어 있을 때 상태(RMT 대기 = LOW)로 둔다.
/// LED를 초기화한 뒤 호출한다.
///
/// ESP-IDF는 정전기 리셋 우회책(`CONFIG_ESP_SLEEP_GPIO_RESET_WORKAROUND`, 기본값)으로
/// 라이트 슬립 중 모든 GPIO를 분리한다. 풀업·풀다운이 없는 WS2812 데이터선은 그러면
/// 잘 때마다(초당 최대 100번) 떠서, 잡음을 데이터로 받아 LED가 엉뚱하게 켜질 수 있다.
/// 버튼은 깨어나 풀업이 돌아온 뒤에 읽고, I2C는 보드의 10k 풀업(R17·R18)이 잡아 주므로
/// 분리된 채로 둔다.
pub fn keep_led_line_driven() {
    unsafe { sys::gpio_sleep_sel_dis(LED_PIN) };
}

/// 이번 부팅이 딥슬립에서 버튼으로 깨어난 것인지
pub fn woke_from_deep_sleep() -> bool {
    unsafe { sys::esp_sleep_get_wakeup_cause() == sys::esp_sleep_source_t_ESP_SLEEP_WAKEUP_GPIO }
}

/// 딥슬립에 들어가기 전에 저장한 자이로 바이어스. 딥슬립에서 깨어난 경우에만 있다.
pub fn saved_bias() -> Option<V3> {
    if !woke_from_deep_sleep() || SAVED_MAGIC.load(Relaxed) != SAVED_MAGIC_VALID {
        return None;
    }
    let axis = |i: usize| f32::from_bits(SAVED_BIAS[i].load(Relaxed));
    Some(V3::new(axis(0), axis(1), axis(2)))
}

/// USB로 컴퓨터(호스트)에 연결돼 있는지. 충전기나 배터리로만 동작 중이면 false.
/// 호스트가 1ms마다 보내는 SOF 패킷으로 판정한다.
pub fn usb_host_connected() -> bool {
    unsafe { sys::usb_serial_jtag_is_connected() }
}

/// 딥슬립에 들어간다. 돌아오지 않는다 — 깨어나면 처음부터 다시 부팅한다.
///
/// 호출하기 전에 BLE를 정리하고 LED를 끄고 IMU를 재워 둔다. `bias`는 RTC 메모리에
/// 남겨, 깨어난 뒤 보드를 손에 든 채로도 캘리브레이션 없이 바로 쓸 수 있게 한다.
pub fn deep_sleep(bias: V3) -> ! {
    for (slot, v) in SAVED_BIAS.iter().zip([bias.x, bias.y, bias.z]) {
        slot.store(v.to_bits(), Relaxed);
    }
    SAVED_MAGIC.store(SAVED_MAGIC_VALID, Relaxed);

    unsafe {
        sys::gpio_set_direction(LED_PIN, sys::gpio_mode_t_GPIO_MODE_OUTPUT);
        sys::gpio_set_level(LED_PIN, 0);
        sys::gpio_hold_en(LED_PIN);
        // 풀업은 ESP-IDF가 기상 조건(LOW)에 맞춰 잠들기 직전에 켠다
        // (CONFIG_ESP_SLEEP_GPIO_ENABLE_INTERNAL_RESISTORS, 기본값).
        let err = sys::esp_deep_sleep_enable_gpio_wakeup(
            WAKE_PINS,
            sys::esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_LOW,
        );
        if err != sys::ESP_OK {
            // 깨울 방법 없이 잠들면 리셋 버튼 말고는 되살릴 수 없다. 대신 재시작한다.
            log::error!("딥슬립 기상 핀 설정 실패 (esp_err {err}) — 잠들지 않고 재시작합니다");
            sys::esp_restart();
        }
        sys::esp_deep_sleep_start()
    }
}
