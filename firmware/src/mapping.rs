//! 자이로 각속도 → HID 커서 이동 매핑 (에어마우스 코어)
//!
//! 축 매핑 (실보드 측정으로 확정 - 보드를 USB 커넥터가 앞(모니터 쪽)을
//! 향하게 쥐었을 때):
//! - yaw (좌우 회전)     = gyro **Z** → dx
//! - pitch (상하 기울임) = gyro **X** → dy
//! - roll (gyro Y)는 사용하지 않는다
//!
//! 튜닝 상수는 전부 이 파일 상단에 모아둔다.

/// 감도: 리포트당 커서 카운트 = 각속도(dps) × GAIN.
/// 100Hz 루프 기준 GAIN 0.18이면 손목 회전(~90dps)에 초당 약 1600카운트.
/// (0.10은 사용자 테스트에서 "너무 둔함" 피드백 → 0.18로 상향)
pub const GAIN: f32 = 0.18;

/// 데드존(dps): 캘리브레이션 잔여 바이어스와 미세 손떨림을 무시하는 문턱.
/// 정지 시 커서 드리프트가 보이면 이 값을 올린다.
pub const DEADZONE_DPS: f32 = 2.0;

/// 방향 부호. 실사용에서 방향이 반대로 느껴지면 여기만 뒤집는다.
pub const YAW_SIGN: f32 = -1.0; // 오른쪽으로 돌리면 커서도 오른쪽
pub const PITCH_SIGN: f32 = -1.0; // 앞을 들면 커서는 위로 (HID는 +y가 아래)

/// 소수점 잔여 누적 매퍼.
///
/// 리포트는 정수(i8)지만 느린 움직임은 리포트당 1카운트 미만이라,
/// 잔여를 누적해야 저속 정밀 조작이 가능하다.
#[derive(Default)]
pub struct CursorMapper {
    acc_x: f32,
    acc_y: f32,
}

impl CursorMapper {
    /// 바이어스 보정된 각속도(dps)를 받아 이번 리포트의 (dx, dy)를 돌려준다.
    pub fn update(&mut self, yaw_dps: f32, pitch_dps: f32) -> (i8, i8) {
        self.acc_x += YAW_SIGN * GAIN * deadzone(yaw_dps);
        self.acc_y += PITCH_SIGN * GAIN * deadzone(pitch_dps);

        let dx = take_step(&mut self.acc_x);
        let dy = take_step(&mut self.acc_y);
        (dx, dy)
    }
}

/// 데드존 적용. 문턱에서 값이 점프하지 않도록 문턱만큼 빼서 연속으로 만든다.
fn deadzone(v: f32) -> f32 {
    let mag = v.abs();
    if mag < DEADZONE_DPS {
        0.0
    } else {
        (mag - DEADZONE_DPS) * v.signum()
    }
}

/// 누적값에서 정수 부분을 꺼내고 잔여는 남긴다. i8 범위로 클램프.
fn take_step(acc: &mut f32) -> i8 {
    let step = acc.trunc().clamp(-127.0, 127.0);
    *acc -= step;
    step as i8
}
