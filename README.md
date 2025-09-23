# Embedded Fun

I bought an SparkFun Thing Plus - ESP32 WROOM (USB-C) and am learning how to write software for it in Rust using the no_std esp-hal library.

## Prerequisites

### All Platforms
- Rust toolchain (install via [rustup.rs](https://rustup.rs/))
- `espflash` for flashing the ESP32 (`cargo install espflash`)

### Installing the ESP32 Toolchain

#### All Platforms (Recommended)
```bash
# Install espup
cargo install espup --locked

# Install ESP toolchains and tools
espup install

# Source the environment variables (required for each new terminal)
# macOS/Linux:
source ~/export-esp.sh

# Windows PowerShell:
# Run the export-esp.ps1 script that espup creates
```

#### Alternative: Manual Installation
If you prefer to manually download espup:

**macOS (Apple Silicon)**:
```bash
curl -LO https://github.com/esp-rs/espup/releases/latest/download/espup-aarch64-apple-darwin
chmod +x espup-aarch64-apple-darwin
./espup-aarch64-apple-darwin install
```

**macOS (Intel)**:
```bash
curl -LO https://github.com/esp-rs/espup/releases/latest/download/espup-x86_64-apple-darwin
chmod +x espup-x86_64-apple-darwin
./espup-x86_64-apple-darwin install
```

**Linux**:
```bash
curl -LO https://github.com/esp-rs/espup/releases/latest/download/espup-x86_64-unknown-linux-gnu
chmod +x espup-x86_64-unknown-linux-gnu
./espup-x86_64-unknown-linux-gnu install
```

**Windows**: Download and run `espup-x86_64-pc-windows-msvc.exe` from [espup releases](https://github.com/esp-rs/espup/releases/latest)

## Building the Project

1. Source the ESP environment variables:
   ```bash
   source ~/export-esp.sh  # macOS/Linux
   # or run the export-esp.ps1 script on Windows
   ```

2. Build the project:
   ```bash
   cargo build
   ```

3. Flash to ESP32:
   ```bash
   espflash flash target/xtensa-esp32-none-elf/debug/esp32-wifi --monitor
   ```

   Or use cargo run with the runner configured:
   ```bash
   cargo run
   ```

## Project Structure

- `src/` - Source code directory
  - `bin/` - Binary targets
    - `main.rs` - Main application entry point
- `.cargo/config.toml` - Cargo configuration with target and environment settings
- `Cargo.toml` - Project dependencies and configuration
- `rust-toolchain.toml` - Rust toolchain specification
- `build.rs` - Build script

## Features

- [x] ESP32 bare metal support using esp-hal
- [x] WiFi station mode capability (esp-wifi)
- [ ] DHCP client via smoltcp
- [ ] Async runtime support (Embassy) (maybe)
- [ ] I2C communication (coming soon)

## Troubleshooting

### Build Issues

#### Missing xtensa-esp32-elf-gcc
If the build fails with "linker xtensa-esp32-elf-gcc not found":
- Ensure you've installed the toolchain: `espup install`
- Source the environment variables: `source ~/export-esp.sh`
- The environment must be sourced in each new terminal session

#### General Build Issues
- Clear the build cache: `cargo clean`
- Verify the ESP toolchain is installed: `rustup toolchain list | grep esp`
- Check that environment variables are set: `echo $PATH | grep esp`

### Flash Issues
- Check the USB port name (varies by OS and device)
- Try lowering the baud rate if flashing fails
- Ensure the ESP32 is in download mode (hold BOOT button while connecting)

### WiFi Connection Issues
- Verify SSID and password are correct
- Check that the ESP32 is within range of the WiFi router
- Monitor serial output for detailed error messages
