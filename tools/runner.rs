//! Simple Teensy 4 program loader, for use as a Cargo runner.
//!
//! # Dependencies
//!
//! - `rust-objcopy`
//! - `teensy_loader_cli`
//!
//! See the teensy4-rs documentation for more information.

use std::{env, error, path::PathBuf, process::{Command, Stdio}, thread::sleep, time::Duration};

/// Loader configurations.
///
/// You may override these values using environment variables.
struct Configuration {
    /// `objcopy` program name.
    objcopy: String,
    /// `teensy_loader_cli` program name.
    loader: String,
    /// sudo program name.
    sudo: String,
    /// 'minicom' program name.
    minicom: String,
}

impl Configuration {
    fn new() -> Self {
        let objcopy = env::var("TEENSY4RS_OBJCOPY").unwrap_or_else(|_| "rust-objcopy".into());
        let loader = env::var("TEENSY4RS_LOADER").unwrap_or_else(|_| "./teensy_loader_cli".into());
        let sudo = "sudo".into();
        let minicom = "minicom".into();
        Self { objcopy, loader, sudo, minicom }
    }
}

fn main() -> Result<(), Box<dyn error::Error>> {
    let elf_path = env::args()
        .nth(1)
        .map(PathBuf::from)
        .ok_or("Supply the path to a Teensy 4 ELF program")?;

    let mut hex_path = elf_path.clone();
    hex_path.set_extension("hex");

    let cfg = Configuration::new();

    Command::new(cfg.objcopy)
        .args(["-O", "ihex"])
        .arg(&elf_path)
        .arg(&hex_path)
        .output()?;

    Command::new(cfg.loader)
        .args(["-w", "-v", "--mcu=TEENSY41"])
        .arg(&hex_path)
        .spawn()?
        .wait()?;

    // sleep(Duration::from_millis(500));

    // Command::new(cfg.sudo)
    //     .arg(cfg.minicom)
    //     .spawn()?;
    
    Ok(())
}