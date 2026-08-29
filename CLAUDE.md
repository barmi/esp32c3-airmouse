# esp32c3-airmouse

ESP32-C3-DevKit-RUST-2(esp-rust-board) 기반 BLE HID 에어마우스. Rust std(ESP-IDF) 환경.

## 워크플로

- GitHub 이슈(barmi/esp32c3-airmouse) 단위로 작업을 진행한다. 새 작업이 생기면 이슈부터 등록.
- 문서와 이슈는 한국어로 작성한다.

## 저장소 구조

- 루트에는 문서(README.md, CLAUDE.md)만 둔다. Rust 펌웨어는 `firmware/`가 cargo 프로젝트 루트.

## 빌드 / 실행

- 빌드/실행은 반드시 `firmware/` 안에서: `cargo build`, `cargo run`(espflash 플래시 + 시리얼 모니터).
- 첫 빌드는 ESP-IDF(v5.3.3)를 `firmware/.embuild/`에 내려받아 오래 걸린다. 보드 포트는 `/dev/cu.usbmodem*`.
- 타겟: `riscv32imc-esp-espidf` (build-std, firmware/rust-toolchain.toml이 자동 적용).
- nightly는 `nightly-2026-07-20`으로 고정 — 이후 nightly는 std의 fchmodat/AT_FDCWD 사용(rust-lang/rust#158168) 때문에 espidf 타겟 빌드가 깨진다. libc가 espidf에 해당 심볼을 추가하면 고정 해제 가능.

## 하드웨어 핵심 사실

- ESP32-C3에는 USB OTG가 없다 → USB HID 마우스 불가. BLE HID(HOGP)로 구현한다.
- I2C: **SDA=GPIO7, SCL=GPIO8** / ICM-42670-P 주소 0x68 (SHTC3 0x70이 같은 버스)
  - RUST-2 개정판 실측값. RUST-1은 SDA=GPIO10이었고 공식 문서 핀 테이블에는 그 값이 잘못 남아 있다.
- WS2812=GPIO2, 단색 LED=GPIO10, BOOT 버튼=GPIO9 (active-low, 부팅 스트래핑 핀)
- 배터리 전압 측정 회로 없음 (MCP73831은 충전만 지원)
- sdkconfig.defaults에 NimBLE가 활성화되어 있다 (esp32-nimble 크레이트용)
- 자이로 축: **yaw(좌우)=Z, pitch(상하)=X, roll=Y(미사용)** — USB 커넥터가 앞을 향하게 쥔 기준
- 전원 인가 직후 자이로는 풀스케일 포화값(±2000dps)을 뱉는다. 캘리브레이션은 앞 샘플을 버리고 |ω|>20dps 샘플도 제외해야 한다 (안 하면 바이어스가 수십 dps 틀어져 커서가 흐른다)

## 주의

- `ws2812-esp32-rmt-driver` 0.14는 esp-idf-hal의 `rmt-legacy`를 강제하므로 구 RMT API(CHANNEL0) 사용이 불가피하다. 관련 `#[allow(deprecated)]`는 이 이유이며, 드라이버가 신규 API로 옮겨가면 제거한다.
- CI가 `cargo clippy -- -D warnings`로 막고 있으니 경고를 남기지 말 것.
