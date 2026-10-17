# 개발 가이드

빌드 환경 구축부터 플래싱, CI까지. 완성된 펌웨어를 쓰는 방법은 [사용법](usage.md)을 보세요.

## 개발 환경 (macOS Apple Silicon 기준)

ESP32-C3는 RISC-V라서 Xtensa용 포크 툴체인(`espup`)이 **필요 없습니다.** 표준 Rust nightly로 빌드합니다.

```bash
cargo install ldproxy espflash
```

Rust 툴체인은 [firmware/rust-toolchain.toml](../firmware/rust-toolchain.toml)이 자동으로 설치·선택하므로 따로 설치할 필요가 없습니다. 필요한 컴포넌트(`rust-src`, `rustfmt`, `clippy`)도 여기에 명시되어 있습니다.

Linux에서는 ESP-IDF 빌드에 네이티브 도구가 추가로 필요합니다 ([ci.yml](../.github/workflows/ci.yml)의 `Install ESP-IDF build prerequisites` 단계 참고).

## 빌드 / 플래시

**모든 cargo 명령은 `firmware/` 안에서 실행합니다.** 저장소 루트에는 `Cargo.toml`이 없습니다.

```bash
cd firmware
```

```bash
cargo build
```

```bash
cargo run
```

`cargo run`은 [.cargo/config.toml](../firmware/.cargo/config.toml)의 러너 설정에 따라 `espflash flash --monitor`를 실행합니다. 즉 빌드 → 플래시 → 시리얼 모니터까지 한 번에 됩니다.

보드는 내장 USB Serial/JTAG로 인식되므로 별도 드라이버 없이 `/dev/cu.usbmodem*`으로 잡힙니다.

### 첫 빌드는 오래 걸립니다

ESP-IDF v5.3.3과 RISC-V 크로스컴파일 툴체인을 `firmware/.embuild/`에 자동으로 내려받기 때문에 **10분 이상** 걸릴 수 있습니다. 이후 증분 빌드는 수 초입니다.

### 시리얼 모니터만 따로 열기

```bash
espflash monitor --port /dev/cu.usbmodem1101
```

`espflash monitor`는 붙을 때 보드를 **재부팅시킵니다** (다운로드 모드를 거치는 리셋이라 RTC 메모리의 부팅 기록도 지워집니다). 재부팅 직후 캘리브레이션이 있으니 보드를 내려놓고 여세요.

### 실행 중인 보드의 로그를 리셋 없이 보기

포트를 기본 설정으로 열면 보드가 재부팅되지 않고 돌던 로그를 그대로 읽습니다. DTR/RTS를 바꾸면 그 과정에서 리셋 조건을 지나가니 건드리지 마세요.

```bash
python3 -c "
import serial
port = serial.Serial('/dev/cu.usbmodem1101', 115200)
while True:
    print(port.readline().decode(errors='replace'), end='', flush=True)
"
```

상태 로그(5초마다)의 `부팅=` 필드에 최근 4번의 부팅 이유가 최근 것부터 나옵니다.

| 표기 | 뜻 |
|---|---|
| `전원` | 전원이 새로 들어옴 (배터리 없이 케이블을 뽑았다 꽂음, `espflash monitor`) |
| `USB` | 플래시할 때의 리셋 |
| `딥슬립/버튼GPIO3` | 딥슬립에서 좌클릭(GPIO3)으로 깨어남. GPIO4는 우클릭, GPIO5는 스크롤↓ |
| `딥슬립/타이머` | 딥슬립에서 타이머로 깨어남 — 생기면 안 되는 경우 |
| `패닉`, `워치독`, `저전압` | 비정상 재부팅 |

USB로 컴퓨터에 연결된 동안에는 펌웨어가 라이트 슬립과 딥슬립을 하지 않아 로그와 플래시가 끊기지 않습니다. 대신 절전 효과는 충전기나 배터리로 동작할 때만 나타납니다 ([전원 관리](architecture.md#전원-관리)).

## 툴체인이 특정 nightly에 고정된 이유

[rust-toolchain.toml](../firmware/rust-toolchain.toml)은 `nightly-2026-07-20`으로 고정되어 있습니다. 최신 nightly에서는 espidf 타겟의 std 빌드가 실패합니다.

```
error[E0425]: cannot find value `AT_FDCWD` in crate `libc`
  --> std/src/sys/fs/unix.rs (fchmodat 호출부)
```

[rust-lang/rust#158168](https://github.com/rust-lang/rust/pull/158168)(2026-07-26 머지)이 `set_permissions`를 `fchmodat`/`AT_FDCWD` 기반으로 바꿨는데, `libc` 크레이트의 espidf 모듈에는 이 심볼이 아직 없습니다. 해당 변경 직전 날짜로 고정한 이유입니다.

**libc가 espidf에 `AT_FDCWD`를 추가하면 고정을 풀 수 있습니다.** 그때는 `channel = "nightly"`로 되돌리고 빌드가 통과하는지 확인하면 됩니다.

## 코드 검사

CI가 `-D warnings`로 막고 있으므로, 푸시 전에 로컬에서 확인하는 것이 좋습니다.

```bash
cargo fmt && cargo clippy --release -- -D warnings
```

### 알려진 예외: RMT deprecated 경고

`ws2812-esp32-rmt-driver` 0.14가 `esp-idf-hal`의 `rmt-legacy` 피처를 강제하므로, 구 RMT API(`CHANNEL0`)를 쓸 수밖에 없습니다. 이 부분에만 사유를 적고 `#[allow(deprecated)]`를 좁게 걸어두었습니다 ([status_led.rs](../firmware/src/status_led.rs), [main.rs](../firmware/src/main.rs)). 드라이버가 신규 API로 옮겨가면 제거하세요.

## CI

[.github/workflows/ci.yml](../.github/workflows/ci.yml) — push와 PR마다 ubuntu 러너에서 실행합니다.

1. `cargo fmt --check`
2. `cargo build --release`
3. `cargo clippy --release -- -D warnings`

캐시는 세 가지입니다. ESP-IDF는 수 GB라 캐시가 없으면 매 실행 10분 이상 걸립니다.

| 캐시 대상 | 키 |
|---|---|
| cargo 레지스트리 + `firmware/target` | `Cargo.lock`, `rust-toolchain.toml` 해시 |
| `firmware/.embuild`, `~/.espressif` | `.cargo/config.toml`, `sdkconfig.defaults` 해시 |

`sdkconfig.defaults`나 ESP-IDF 버전을 바꾸면 캐시 키가 바뀌어 자동으로 재구성됩니다.

## ESP-IDF 설정

[sdkconfig.defaults](../firmware/sdkconfig.defaults)에서 관리합니다. 바꾸면 ESP-IDF가 재구성되므로 다음 빌드가 느려집니다.

| 설정 | 이유 |
|---|---|
| `CONFIG_ESP_MAIN_TASK_STACK_SIZE=20000` | Rust + BLE HID + IMU를 한 태스크에서 돌리므로 여유 확보 |
| `CONFIG_BT_NIMBLE_ENABLED=y` (+ Bluedroid 비활성) | esp32-nimble 크레이트 요구사항 |
| `CONFIG_BT_NIMBLE_NVS_PERSIST=y` | 본딩 정보 저장 — 재부팅 후 자동 재연결 |
| `CONFIG_BT_NIMBLE_LOG_LEVEL_WARNING=y` | NimBLE INFO 로그가 notify마다 찍혀 시리얼을 도배함 |
| `CONFIG_PM_ENABLE=y`, `CONFIG_FREERTOS_USE_TICKLESS_IDLE=y` | 클럭 자동 조절 + 자동 라이트 슬립 |
| `CONFIG_FREERTOS_HZ=1000` | 기본 100Hz면 라이트 슬립 최소 대기(3틱)가 30ms라 10ms 루프에서 잠들지 못함 |
| `CONFIG_PM_SLP_IRAM_OPT=y`, `CONFIG_PM_RTOS_IDLE_OPT=y` | 슬립 진입·복귀 코드를 IRAM에 두어 한 번에 더 오래 잠 |
| `CONFIG_BT_CTRL_MODEM_SLEEP=y` 외 3개 | BLE 연결을 유지한 채 슬립. 32kHz 크리스털 미실장이라 메인 크리스털을 저전력 클럭으로 씀 |
| `CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION=y` | USB로 컴퓨터에 연결된 동안은 라이트 슬립 금지 (USB 시리얼 유지) |

## 문제 해결

### `Device or resource busy`

시리얼 모니터가 이미 포트를 잡고 있습니다. 기존 `espflash monitor` 프로세스를 종료하고 다시 시도하세요.

### 절전 동작은 맥에 꽂은 채로는 확인할 수 없어요

USB로 컴퓨터에 연결된 동안은 라이트 슬립이 꺼지므로, 슬립과 관련된 문제는 충전기나 배터리로 동작할 때만 드러납니다. 그때는 시리얼 로그를 볼 수 없으니 맥 쪽에서 확인합니다.

- **BLE 연결·끊김**: `/usr/bin/log show --last 10m --info --predicate 'subsystem BEGINSWITH "com.apple.bluetooth"' | grep B0:A6:04` — 연결 완료, 끊김 사유(708 = 감시 타임아웃), 맥이 정한 연결 간격이 보입니다. zsh에서는 `log`가 내장 명령이라 경로를 붙여야 합니다.
- **스크롤·커서 속도**: 맥의 스크롤 이벤트 시각과 방향은 듣기 전용 이벤트 탭(권한 불필요)으로, 초당 마우스 이동 이벤트 수는 `CGEventSourceCounterForEventType`으로 잴 수 있습니다.

배터리를 뺀 상태에서는 USB를 뽑는 순간 보드가 꺼졌다가 충전기에서 새로 부팅합니다.

### 보드를 꽂았는데 포트가 안 보여요

배터리로 쓰다가 딥슬립에 들어간 상태면 USB를 꽂아도 깨어나지 않습니다(USB 연결은 기상 조건이 아닙니다). 좌클릭을 눌러 깨우거나 RST 버튼을 누르세요. 깨어난 뒤에는 USB 호스트가 연결돼 있으므로 다시 잠들지 않습니다.

### `No such file or directory` (플래시할 때)

`firmware/`가 아닌 곳에서 실행했을 가능성이 높습니다. 빌드 산출물 경로가 어긋난 것입니다.

### 빌드가 python/cmake 관련 오류로 실패

ESP-IDF 인스톨러가 시스템 python을 씁니다. conda 등 비표준 python이 PATH 앞에 있으면 실패할 수 있습니다. Homebrew python을 앞에 두고 `firmware/.embuild/`를 지운 뒤 다시 빌드하세요.

### 부팅은 되는데 IMU 초기화가 실패

I2C 핀 문제일 가능성이 큽니다. [하드웨어 문서의 핀맵 경고](hardware.md#주의-rust-2의-i2c-핀은-문서와-다릅니다)를 확인하세요.

## 참고 자료

- [The Rust on ESP Book](https://docs.esp-rs.org/book/)
- [esp-idf-template](https://github.com/esp-rs/esp-idf-template) — 프로젝트 구성의 기준
- [esp-idf-svc examples](https://github.com/esp-rs/esp-idf-svc/tree/master/examples)
- [esp32-nimble examples](https://github.com/taks/esp32-nimble/tree/main/examples)
