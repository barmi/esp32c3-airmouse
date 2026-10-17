use crate::pointer::ABS_MAX;
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
/// - Report ID 2: 펜 디지타이저 — 팁·배럴 스위치 + X/Y 절대 좌표(0~32767) (절대 모드)
///
/// 절대 쪽을 일반 마우스(Button 페이지 버튼)로 선언하면 macOS에서 클릭이 깨진다.
/// macOS의 IOHIDEventDriver는 장치에 상대 이동 요소가 있으면 Button 페이지 버튼을
/// 전부 상대 포인터에 붙이고, 버튼 상태를 계산할 때 방금 온 리포트인지 따지지 않고
/// 모든 버튼 요소의 마지막 값을 쓴다. 그 결과 절대 리포트 하나마다 상대 이벤트
/// (버튼 눌림)와 절대 이벤트(버튼 0)가 연달아 나가 눌림/뗌이 반복되고, 상대 모드
/// 에서는 절대 쪽 버튼의 옛 값(0)이 눌림을 덮어쓴다. 펜의 팁·배럴 스위치는
/// Digitizer 페이지라 디지타이저 경로에 남아 이런 섞임이 없다.
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
    // ------------------------------------------------- 절대 좌표: 펜 디지타이저
    // 일반 버튼(Button 페이지)을 두면 macOS가 상대 포인터 쪽에 붙여버려 클릭이
    // 깨지므로, 펜의 팁·배럴 스위치(Digitizer 페이지)로 버튼을 보낸다. 위 설명 참고.
    (USAGE_PAGE, 0x0D),            // Digitizer
    (USAGE, 0x02),                 // Pen
    (COLLECTION, 0x01),            // Application
    (REPORT_ID, ABS_ID),           //
    (USAGE, 0x20),                 //   Stylus
    (COLLECTION, 0x00),            //   Physical
    (USAGE, 0x42),                 //     Tip Switch    → 좌클릭
    (USAGE, 0x44),                 //     Barrel Switch → 우클릭
    (USAGE, 0x32),                 //     In Range
    (LOGICAL_MINIMUM, 0x00),       //
    (LOGICAL_MAXIMUM, 0x01),       //
    (REPORT_SIZE, 0x01),           //
    (REPORT_COUNT, 0x03),          //
    (HIDINPUT, 0x02),              //     Data,Var,Abs
    (REPORT_COUNT, 0x05),          //     패딩 5비트
    (HIDINPUT, 0x03),              //     Const,Var,Abs
    (USAGE_PAGE, 0x01),            //     Generic Desktop
    (USAGE, 0x30),                 //     X
    (USAGE, 0x31),                 //     Y
    (LOGICAL_MINIMUM, 0x00),       //     0
    (LOGICAL_MAXIMUM, 0xFF, 0x7F), //     32767 (pointer::ABS_MAX와 같아야 함)
    (REPORT_SIZE, 0x10),           //     16비트
    (REPORT_COUNT, 0x02),          //
    (HIDINPUT, 0x02),              //     Data,Var,Abs
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

/// 펜 스위치 비트 (Report ID 2 첫 바이트)
const PEN_TIP: u8 = 0x01;
const PEN_BARREL: u8 = 0x02;
const PEN_IN_RANGE: u8 = 0x04;

/// 펜 입력 리포트. 디스크립터의 Report ID 2와 필드 순서가 같아야 한다.
/// 리틀 엔디언으로 보내야 하는데 ESP32-C3(RISC-V)가 리틀 엔디언이라 그대로 쓴다.
#[derive(IntoBytes, Immutable, Clone, Copy, Default)]
#[repr(C, packed)]
struct PenReport {
    /// bit0=팁(좌클릭), bit1=배럴(우클릭), bit2=근접(In Range)
    switches: u8,
    x: u16,
    y: u16,
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

        // 연결된 뒤 호스트가 표시하는 이름은 광고 패킷의 이름이 아니라 GAP 기기 이름
        // 특성(0x2A00)이다. 지정하지 않으면 NimBLE 기본값 "nimble"이 나간다.
        BLEDevice::set_device_name(device_name)?;

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

    /// 호스트가 절대 좌표 리포트를 구독했는지. 디스크립터가 바뀐 펌웨어로 올린 뒤
    /// 재페어링하지 않으면 호스트는 예전 구조를 기억해 이 리포트를 구독하지 않고,
    /// 그 상태에서 보낸 절대 좌표는 아무 데도 가지 않는다.
    pub fn abs_subscribed(&self) -> bool {
        self.input_abs.lock().subscribed_count() > 0
    }

    /// 절대 좌표 모드 전송. `buttons`는 MouseReport와 같은 비트(좌=0x01, 우=0x02)이고
    /// 펜의 팁·배럴 스위치로 바뀐다. `x`, `y`는 0..=ABS_MAX (화면 왼쪽 위가 0).
    /// 이 Mac의 드라이버는 배럴 스위치를 우클릭으로 인식하지 않으므로, 호출하는 쪽은
    /// 우버튼을 상대 리포트로 보낸다.
    /// `in_range`를 false로 보내면 펜이 근접 범위를 벗어난 것으로 알린다(모드 종료 시).
    pub fn send_pen(&mut self, buttons: u8, x: u16, y: u16, in_range: bool) {
        let mut switches = 0;
        if buttons & 0x01 != 0 {
            switches |= PEN_TIP;
        }
        if buttons & 0x02 != 0 {
            switches |= PEN_BARREL;
        }
        if in_range {
            switches |= PEN_IN_RANGE;
        }
        // macOS(BLE는 DriverKit의 AppleUserHIDEventDriver가 처리)는 0..=ABS_MAX를
        // 화면에 그대로 펼친다. 실제 커서 위치를 읽어 확인했다 — 가장자리를 잘라내는
        // 것은 커널 드라이버(IOHIDEventDriver)의 동작이라 여기엔 해당하지 않는다.
        let report = PenReport {
            switches,
            x: x.min(ABS_MAX),
            y: y.min(ABS_MAX),
        };
        self.input_abs.lock().set_value(report.as_bytes()).notify();
    }
}
