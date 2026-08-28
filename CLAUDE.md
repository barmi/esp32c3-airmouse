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
- 타겟: `riscv32imc-esp-espidf` (nightly + build-std, firmware/rust-toolchain.toml이 자동 적용).

## 하드웨어 핵심 사실

- ESP32-C3에는 USB OTG가 없다 → USB HID 마우스 불가. BLE HID(HOGP)로 구현한다.
- I2C: SDA=GPIO10, SCL=GPIO8 / ICM-42670-P 주소 0x68 (SHTC3 0x70이 같은 버스)
- WS2812=GPIO2, 단색 LED=GPIO7, BOOT 버튼=GPIO9 (active-low, 부팅 스트래핑 핀)
- 배터리 전압 측정 회로 없음 (MCP73831은 충전만 지원)
- sdkconfig.defaults에 NimBLE가 활성화되어 있다 (esp32-nimble 크레이트용)
