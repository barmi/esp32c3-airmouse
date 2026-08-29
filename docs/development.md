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

## 문제 해결

### `Device or resource busy`

시리얼 모니터가 이미 포트를 잡고 있습니다. 기존 `espflash monitor` 프로세스를 종료하고 다시 시도하세요.

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
