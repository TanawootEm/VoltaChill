// Desktop binary. All logic lives in the library (src/lib.rs) so the same modules
// also build into the Android cdylib (android_main). On Android there is no binary
// entry — the NativeActivity loads the cdylib and calls android_main.
#[cfg(not(target_os = "android"))]
fn main() -> eframe::Result<()> {
    voltachill::desktop_main()
}

#[cfg(target_os = "android")]
fn main() {}
