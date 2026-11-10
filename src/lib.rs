mod app;
mod car;
mod history;
mod obd;
mod sniff;
mod vin;

#[cfg(not(target_os = "android"))]
use std::path::PathBuf;

/// Desktop entry (called from src/main.rs). Parses CLI args and runs the native window.
#[cfg(not(target_os = "android"))]
pub fn desktop_main() -> eframe::Result<()> {
    let mut screenshot = None;
    let mut open = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--screenshot" => screenshot = args.next().map(PathBuf::from),
            "--open" => open = args.next(),
            _ => {}
        }
    }

    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_title("VoltaChill")
            .with_inner_size([1024.0, 720.0])
            .with_min_inner_size([900.0, 620.0]),
        ..Default::default()
    };
    eframe::run_native(
        "ScanLab",
        options,
        Box::new(move |cc| Ok(Box::new(app::ScanApp::new(cc, screenshot, open)))),
    )
}

/// Android entry. The NativeActivity (android-game-activity, via winit) calls `android_main`.
/// Bluetooth goes through the Kotlin BLE bridge (obd::android_ble).
#[cfg(target_os = "android")]
#[no_mangle]
fn android_main(app: winit::platform::android::activity::AndroidApp) {
    android_logger::init_once(android_logger::Config::default().with_max_level(log::LevelFilter::Info));
    let options = eframe::NativeOptions {
        android_app: Some(app),
        ..Default::default()
    };
    let _ = eframe::run_native(
        "ScanLab",
        options,
        Box::new(move |cc| Ok(Box::new(app::ScanApp::new(cc, None, None)))),
    );
}
