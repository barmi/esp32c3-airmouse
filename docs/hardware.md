# 하드웨어

[ESP32-C3-DevKit-RUST-2](https://github.com/esp-rs/esp-rust-board) (esp-rust-board) 온보드 구성만 사용합니다. 추가 부품이나 납땜은 필요 없습니다.

## 구성 요소

| 구성 요소 | 사양 | 연결 |
|---|---|---|
| MCU | ESP32-C3 (RISC-V 160MHz, 4MB Flash, BLE 5.0) | — |
| IMU | ICM-42670-P (6축 자이로+가속도) | I2C 주소 `0x68` |
| 온습도 센서 | SHTC3 (이 프로젝트에서는 미사용) | I2C 주소 `0x70` |
| I2C 버스 | 400kHz | SDA=`GPIO7`, SCL=`GPIO8` |
| RGB LED | WS2812 (상태 표시용) | `GPIO2` |
| 단색 LED | (미사용) | `GPIO10` |
| 버튼 | BOOT 버튼 → 좌클릭 | `GPIO9` (active-low, 내부 풀업) |
| 전원 | USB-C, LiPo 충전(MCP73831) | 배터리 전압 측정 회로 없음 |

## 주의: RUST-2의 I2C 핀은 문서와 다릅니다

**공식 문서에 적힌 SDA=GPIO10은 RUST-1 값이며, RUST-2에서는 동작하지 않습니다.**

RUST-2 개정판에서 유저 LED가 `GPIO7` → `GPIO10`으로 옮겨가면서, 그 자리를 I2C SDA가 넘겨받았습니다. 그런데 [Espressif 공식 사용자 가이드](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c3/esp32-c3-devkit-rust-2/user_guide.html)와 [esp-rust-board README](https://github.com/esp-rs/esp-rust-board)의 핀 테이블에는 여전히 RUST-1 값(SDA=GPIO10, LED=GPIO7)이 실려 있습니다. 두 문서 모두 "유저 LED가 GPIO10으로 바뀌었다"는 변경 사항은 본문에 적어두고서 정작 핀 테이블은 갱신하지 않았습니다.

| | RUST-1 | **RUST-2 (이 프로젝트)** |
|---|---|---|
| I2C SDA | GPIO10 | **GPIO7** |
| 유저 LED | GPIO7 | GPIO10 |

이 값은 실제 보드에서 I2C 스캔으로 확인했습니다. 문서대로 GPIO10을 쓰면 IMU가 ACK하지 않고 다음 에러가 납니다.

```
IMU 오류: BusError(I2cError { kind: NoAcknowledge(Unknown), cause: ESP_FAIL })
```

### 핀맵을 직접 확인하는 방법

다른 보드 개정판을 쓰거나 배선이 의심스러우면, GPIO 조합을 순회하며 IMU(`0x68`)와 SHTC3(`0x70`)가 응답하는 쌍을 찾는 것이 가장 빠릅니다.

이때 **조합마다 `gpio_reset_pin()`으로 GPIO 매트릭스를 초기화해야 합니다.** ESP32의 GPIO 매트릭스는 이전 시도의 I2C 신호 라우팅을 핀에 남겨두기 때문에, 초기화하지 않으면 실제로는 연결되지 않은 조합까지 전부 ACK하는 것처럼 보입니다. 실제로 이 프로젝트 브링업 중 초기화 없이 스캔했을 때 10개 조합이 모두 "발견"으로 잡혔고, 초기화를 넣자 정답 하나(SDA=GPIO7, SCL=GPIO8)만 남았습니다.

## 왜 BLE인가 — USB HID는 불가능합니다

ESP32-C3의 USB 포트는 **Serial/JTAG 전용**입니다. USB OTG 컨트롤러가 없어서 USB HID 마우스로 동작할 수 없습니다. (USB OTG는 ESP32-S2/S3에만 있습니다.)

그래서 이 프로젝트는 **BLE HID(HOGP, HID over GATT Profile)** 로 구현합니다. USB 케이블은 전원 공급과 펌웨어 플래싱/디버깅 용도로만 쓰입니다.

## ICM-42670-P 특성

### 축 방향

USB 커넥터가 앞(모니터 쪽)을 향하도록 쥔 기준입니다. 실제 보드를 움직여 로그로 확인한 값입니다.

| 자이로 축 | 회전 | 용도 |
|---|---|---|
| **Z** | yaw — 손목 좌우 회전 | 커서 X (좌우) |
| **X** | pitch — 손목 상하 기울임 | 커서 Y (상하) |
| Y | roll — 옆으로 눕히기 | 미사용 |

### 전원 인가 직후 포화값 주의

자이로는 스핀업이 끝나기 전까지 **풀스케일 포화값(±2000dps)** 을 출력합니다. 부팅 직후 첫 샘플에서 이런 값이 나옵니다.

```
gyro[dps] x=-1998.05 y=-1998.05 z=-1998.05
```

캘리브레이션에서 이 값이 평균에 섞이면 치명적입니다. 200샘플 평균에 포화 샘플이 **2개만** 들어가도 바이어스가 20dps 틀어지고, 그러면 정지 상태에서도 커서가 계속 흐릅니다.

실제로 브링업 중 이 문제로 바이어스가 `x=-19.31, z=-13.40`(정상값의 100배)으로 측정된 적이 있습니다. 현재 코드는 앞 샘플을 버리는 것에 더해 **|ω| > 20dps인 샘플을 평균에서 제외**하여 이를 막습니다 ([firmware/src/imu.rs](../firmware/src/imu.rs)의 `calibrate_gyro`).

### 설정값

| 항목 | 값 | 이유 |
|---|---|---|
| 전원 모드 | 6축 low-noise | 자이로 + 가속도 모두 활성 |
| 자이로 풀스케일 | ±2000dps | 빠른 손동작에도 포화되지 않는 최대 범위 |
| 자이로 ODR | 200Hz | 메인 루프(100Hz)보다 높게 잡아 항상 최신 값 확보 |

## 전원

- USB-C (PD 미지원)
- LiPo 배터리 충전: MCP73831, 4.2V까지
- **보호회로 내장 배터리를 쓰세요.** MCP73831에는 과방전 보호가 없습니다.
- **배터리 잔량 측정 불가** — 전압 분배 회로가 없습니다. HID 배터리 서비스에는 100% 고정값을 보고합니다.

## GPIO9(BOOT 버튼) 주의

GPIO9는 부팅 스트래핑 핀입니다. 동작 중 입력으로 쓰는 데는 문제가 없지만, **버튼을 누른 채로 리셋하면 다운로드 모드로 진입**합니다. 펌웨어가 실행되지 않는다면 버튼에서 손을 떼고 다시 리셋하세요.

## 참고 자료

- [esp-rust-board 저장소](https://github.com/esp-rs/esp-rust-board) — 회로도, KiCad 파일
- [ESP32-C3-DevKit-RUST-2 공식 사용자 가이드](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c3/esp32-c3-devkit-rust-2/user_guide.html) — 핀 테이블은 위 경고 참고
- [ICM-42670-P 드라이버](https://github.com/jessebraham/icm42670)
