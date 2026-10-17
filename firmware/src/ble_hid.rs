use esp32_nimble::{
    enums::*, hid::*, utilities::mutex::Mutex, BLEAdvertisementData, BLECharacteristic, BLEDevice,
    BLEHIDDevice, BLEServer,
};
use std::sync::Arc;
use zerocopy::IntoBytes;
use zerocopy_derive::{Immutable, IntoBytes};

const MOUSE_ID: u8 = 0x01;
const ABS_ID: u8 = 0x02;

/// 리포트 디스크립터: 두 개의 포인터를 선언한다.
/// - Report ID 1: 표준 3버튼 + X/Y 상대 이동 + 휠 (상대 모드)
/// - Report ID 2: 3버튼 + X/Y 절대 좌표(0~32767) + 휠 (절대 모드)
///
/// 이 구조를 바꾸면 호스트가 캐시한 디스크립터와 어긋나므로 기기를 지우고 다시
/// 페어링해야 한다.
const HID_REPORT_DESCRIPTOR: &[u8] = hid!(
    (USAGE_PAGE, 0x01),      // Generic Desktop
    (USAGE, 0x02),           // Mouse
    (COLLECTION, 0x01),      // Application
    (USAGE, 0x01),           //   Pointer
    (COLLECTION, 0x00),      //   Physical
    (REPORT_ID, MOUSE_ID),   //
    (USAGE_PAGE, 0x09),      //     Buttons
    (USAGE_MINIMUM, 0x01),   //
    (USAGE_MAXIMUM, 0x03),   //
    (LOGICAL_MINIMUM, 0x00), //
    (LOGICAL_MAXIMUM, 0x01), //
    (REPORT_SIZE, 0x01),     //
    (REPORT_COUNT, 0x03),    //     버튼 3개 (좌/우/휠)
    (HIDINPUT, 0x02),        //     Data,Var,Abs
    (REPORT_SIZE, 0x05),     //     패딩 5비트
    (REPORT_COUNT, 0x01),    //
    (HIDINPUT, 0x03),        //     Const,Var,Abs
    (USAGE_PAGE, 0x01),      //     Generic Desktop
    (USAGE, 0x30),           //     X
    (USAGE, 0x31),           //     Y
    (USAGE, 0x38),           //     Wheel
    (LOGICAL_MINIMUM, 0x81), //     -127
    (LOGICAL_MAXIMUM, 0x7F), //     127
    (REPORT_SIZE, 0x08),     //
    (REPORT_COUNT, 0x03),    //
    (HIDINPUT, 0x06),        //     Data,Var,Rel
    (END_COLLECTION),        //   /Physical
    (END_COLLECTION),        // /Application
    // ------------------------------------------------- 절대 좌표 포인터
    (USAGE_PAGE, 0x01),            // Generic Desktop
    (USAGE, 0x02),                 // Mouse
    (COLLECTION, 0x01),            // Application
    (USAGE, 0x01),                 //   Pointer
    (COLLECTION, 0x00),            //   Physical
    (REPORT_ID, ABS_ID),           //
    (USAGE_PAGE, 0x09),            //     Buttons
    (USAGE_MINIMUM, 0x01),         //
    (USAGE_MAXIMUM, 0x03),         //
    (LOGICAL_MINIMUM, 0x00),       //
    (LOGICAL_MAXIMUM, 0x01),       //
    (REPORT_SIZE, 0x01),           //
    (REPORT_COUNT, 0x03),          //     버튼 3개
    (HIDINPUT, 0x02),              //     Data,Var,Abs
    (REPORT_SIZE, 0x05),           //     패딩 5비트
    (REPORT_COUNT, 0x01),          //
    (HIDINPUT, 0x03),              //     Const,Var,Abs
    (USAGE_PAGE, 0x01),            //     Generic Desktop
    (USAGE, 0x30),                 //     X
    (USAGE, 0x31),                 //     Y
    (LOGICAL_MINIMUM, 0x00),       //     0
    (LOGICAL_MAXIMUM, 0xFF, 0x7F), //     32767 (pointer::ABS_MAX와 같아야 함)
    (REPORT_SIZE, 0x10),           //     16비트
    (REPORT_COUNT, 0x02),          //
    (HIDINPUT, 0x02),              //     Data,Var,Abs ← 절대 좌표
    (USAGE, 0x38),                 //     Wheel
    (LOGICAL_MINIMUM, 0x81),       //     -127 (전역 값이라 다시 지정)
    (LOGICAL_MAXIMUM, 0x7F),       //     127
    (REPORT_SIZE, 0x08),           //
    (REPORT_COUNT, 0x01),          //
    (HIDINPUT, 0x06),              //     Data,Var,Rel
    (END_COLLECTION),              //   /Physical
    (END_COLLECTION),              // /Application
);

/// HID 마우스 입력 리포트. 디스크립터의 필드 순서와 일치해야 한다.
#[derive(IntoBytes, Immutable, Clone, Copy, Default)]
#[repr(C, packed)]
pub struct MouseReport {
    /// bit0=좌클릭, bit1=우클릭, bit2=휠클릭
    pub buttons: u8,
    pub dx: i8,
    pub dy: i8,
    pub wheel: i8,
}

/// HID 절대 좌표 입력 리포트. 디스크립터의 Report ID 2와 필드 순서가 같아야 한다.
/// 리틀 엔디언으로 보내야 하는데 ESP32-C3(RISC-V)가 리틀 엔디언이라 그대로 쓴다.
#[derive(IntoBytes, Immutable, Clone, Copy, Default)]
#[repr(C, packed)]
pub struct AbsReport {
    /// bit0=좌클릭, bit1=우클릭, bit2=휠클릭
    pub buttons: u8,
    /// 0..=32767, 화면 왼쪽 → 오른쪽
    pub x: u16,
    /// 0..=32767, 화면 위 → 아래
    pub y: u16,
    pub wheel: i8,
}

/// BLE HID(HOGP) 마우스.
///
/// ESP32-C3에는 USB OTG가 없어서 USB HID는 불가능 - BLE HID가 유일한 경로다.
/// 연결이 끊기면 esp32-nimble 기본값(advertise_on_disconnect=true)으로 광고가
/// 자동 재개되고, 본딩 정보는 NVS에 저장되어 재부팅 후에도 유지된다.
pub struct BleMouse {
    server: &'static mut BLEServer,
    input_mouse: Arc<Mutex<BLECharacteristic>>,
    input_abs: Arc<Mutex<BLECharacteristic>>,
}

impl BleMouse {
    pub fn new(device_name: &str) -> anyhow::Result<Self> {
        let device = BLEDevice::take();

        // HID 프로파일은 본딩 + 암호화가 필수 (macOS/Windows 공통)
        device
            .security()
            .set_auth(AuthReq::all())
            .set_io_cap(SecurityIOCap::NoInputNoOutput)
            .resolve_rpa();

        let server = device.get_server();
        server.on_connect(|_server, desc| {
            log::info!("BLE 연결됨: {:?}", desc.address());
        });
        server.on_disconnect(|desc, reason| {
            log::info!("BLE 연결 해제: {:?} (사유: {:?})", desc.address(), reason);
        });

        let mut hid = BLEHIDDevice::new(server);
        let input_mouse = hid.input_report(MOUSE_ID);
        let input_abs = hid.input_report(ABS_ID);

        hid.manufacturer("skshin");
        // sig=0x02(USB), VID=0x303A(Espressif), PID/버전은 임의
        hid.pnp(0x02, 0x303A, 0x4001, 0x0110);
        hid.hid_info(0x00, 0x01);
        hid.report_map(HID_REPORT_DESCRIPTOR);
        hid.set_battery_level(100);

        let advertising = device.get_advertising();
        advertising.lock().scan_response(false).set_data(
            BLEAdvertisementData::new()
                .name(device_name)
                .appearance(0x03C2) // HID Mouse
                .add_service_uuid(hid.hid_service().lock().uuid()),
        )?;
        advertising.lock().start()?;
        log::info!("BLE 광고 시작: {device_name}");

        Ok(Self {
            server,
            input_mouse,
            input_abs,
        })
    }

    pub fn connected(&self) -> bool {
        self.server.connected_count() > 0
    }

    pub fn send(&mut self, report: &MouseReport) {
        self.input_mouse
            .lock()
            .set_value(report.as_bytes())
            .notify();
    }

    pub fn send_abs(&mut self, report: &AbsReport) {
        self.input_abs.lock().set_value(report.as_bytes()).notify();
    }
}
