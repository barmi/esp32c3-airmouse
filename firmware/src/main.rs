fn main() {
    // ESP-IDF 링커 패치 적용 (esp-idf 프로젝트 필수 보일러플레이트)
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("esp32c3-airmouse: boot OK");

    let mut uptime = 0u32;
    loop {
        std::thread::sleep(std::time::Duration::from_secs(5));
        uptime += 5;
        log::info!("alive: {uptime}s");
    }
}
