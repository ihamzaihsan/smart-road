use std::{env, error::Error, path::PathBuf};

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=SDL2_LIB_DIR");

    if env::var_os("CARGO_FEATURE_GRAPHICS").is_none() {
        return Ok(());
    }

    // Use an installed SDL2 library without compiling SDL's C source.
    if let Some(directory) = env::var_os("SDL2_LIB_DIR") {
        let directory = PathBuf::from(directory);
        println!(
            "cargo:rustc-link-search=native={}",
            directory.canonicalize()?.display()
        );
    }
    Ok(())
}
