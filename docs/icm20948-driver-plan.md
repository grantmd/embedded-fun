# ICM-20948 Custom Driver Implementation Plan

## Overview

Plan for implementing a custom Rust driver for the ICM-20948 9-DoF IMU to replace the old `icm20948` crate.

## Why Write Our Own?

1. **Modern embedded-hal 1.0** - No adapter layer needed
2. **Magnetometer support** - Full 9-DoF access (accel + gyro + mag)
3. **Tailored to our needs** - Only implement what we use
4. **Learning experience** - Deep understanding of the hardware
5. **Async-friendly** - Can integrate well with Embassy

## What We Need to Implement

### 1. Basic Register Access (~30 lines)

The ICM-20948 has a **bank-switching system** with 4 register banks (0-3).

```rust
pub struct Icm20948<I2C> {
    i2c: I2C,
    address: u8,
    current_bank: u8,
    // Scale factors based on configured ranges
    accel_scale: f32,
    gyro_scale: f32,
}

impl<I2C: embedded_hal::i2c::I2c> Icm20948<I2C> {
    fn select_bank(&mut self, bank: u8) -> Result<(), I2C::Error> {
        // Write to REG_BANK_SEL (0x7F) to switch banks
    }

    fn write_register(&mut self, bank: u8, reg: u8, value: u8) -> Result<(), I2C::Error> {
        // Switch bank if needed, then write register
    }

    fn read_register(&mut self, bank: u8, reg: u8) -> Result<u8, I2C::Error> {
        // Switch bank if needed, then read register
    }

    fn read_registers(&mut self, bank: u8, reg: u8, buffer: &mut [u8]) -> Result<(), I2C::Error> {
        // For burst reads (e.g., all 6 accel bytes at once)
    }
}
```

**Key Registers:**
- `REG_BANK_SEL` (0x7F) - Bank selection register (available in all banks)

### 2. Initialization Sequence (~50-100 lines)

```rust
impl<I2C: embedded_hal::i2c::I2c> Icm20948<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            current_bank: 0xFF, // Invalid to force first select
            accel_scale: 1.0,
            gyro_scale: 1.0,
        }
    }

    pub fn init(&mut self, delay: &mut impl DelayMs<u8>) -> Result<(), I2C::Error> {
        // 1. Check WHO_AM_I (should be 0xEA)
        // 2. Reset device (PWR_MGMT_1, bit 7)
        // 3. Wait for reset to complete
        // 4. Wake up device (clear SLEEP bit)
        // 5. Set clock source to auto-select
        // 6. Configure accelerometer range (±2g, ±4g, ±8g, ±16g)
        // 7. Configure gyroscope range (±250, ±500, ±1000, ±2000 dps)
        // 8. Enable accelerometer and gyroscope
        // 9. Set sample rates
        // 10. Configure low-pass filters (optional)
    }
}
```

**Key Registers (Bank 0):**
- `WHO_AM_I` (0x00) - Should read 0xEA
- `PWR_MGMT_1` (0x06) - Power management (reset, sleep, clock source)
- `PWR_MGMT_2` (0x07) - Enable/disable sensors

**Key Registers (Bank 2):**
- `GYRO_SMPLRT_DIV` (0x00) - Gyro sample rate divider
- `GYRO_CONFIG_1` (0x01) - Gyro range and DLPF
- `ACCEL_SMPLRT_DIV_1/2` (0x10-0x11) - Accel sample rate divider
- `ACCEL_CONFIG` (0x14) - Accel range and DLPF

### 3. Reading Accelerometer & Gyroscope (~40 lines)

```rust
impl<I2C: embedded_hal::i2c::I2c> Icm20948<I2C> {
    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        let mut buffer = [0u8; 6];
        self.read_registers(0, ACCEL_XOUT_H, &mut buffer)?;

        let x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let z = i16::from_be_bytes([buffer[4], buffer[5]]);

        Ok((
            x as f32 * self.accel_scale,
            y as f32 * self.accel_scale,
            z as f32 * self.accel_scale,
        ))
    }

    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        let mut buffer = [0u8; 6];
        self.read_registers(0, GYRO_XOUT_H, &mut buffer)?;

        let x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let z = i16::from_be_bytes([buffer[4], buffer[5]]);

        Ok((
            x as f32 * self.gyro_scale,
            y as f32 * self.gyro_scale,
            z as f32 * self.gyro_scale,
        ))
    }

    pub fn read_temp(&mut self) -> Result<f32, I2C::Error> {
        let mut buffer = [0u8; 2];
        self.read_registers(0, TEMP_OUT_H, &mut buffer)?;

        let raw = i16::from_be_bytes([buffer[0], buffer[1]]);

        // Temperature in °C = (raw / 333.87) + 21.0
        Ok((raw as f32 / 333.87) + 21.0)
    }
}
```

**Key Registers (Bank 0):**
- `ACCEL_XOUT_H` (0x2D) - Start of 6-byte accel data
- `GYRO_XOUT_H` (0x33) - Start of 6-byte gyro data
- `TEMP_OUT_H` (0x39) - Start of 2-byte temp data

**Scale Factors:**
- Accel ±2g: 16384 LSB/g → scale = 9.81 / 16384 = 0.0005985 m/s²/LSB
- Accel ±4g: 8192 LSB/g → scale = 9.81 / 8192 = 0.001197 m/s²/LSB
- Accel ±8g: 4096 LSB/g → scale = 9.81 / 4096 = 0.002394 m/s²/LSB
- Accel ±16g: 2048 LSB/g → scale = 9.81 / 2048 = 0.004788 m/s²/LSB

- Gyro ±250 dps: 131 LSB/(deg/s) → scale = (π/180) / 131 = 0.0001332 rad/s/LSB
- Gyro ±500 dps: 65.5 LSB/(deg/s) → scale = (π/180) / 65.5 = 0.0002663 rad/s/LSB
- Gyro ±1000 dps: 32.8 LSB/(deg/s) → scale = (π/180) / 32.8 = 0.0005326 rad/s/LSB
- Gyro ±2000 dps: 16.4 LSB/(deg/s) → scale = (π/180) / 16.4 = 0.001065 rad/s/LSB

### 4. Magnetometer Access (~100-150 lines)

**The Challenge:** The AK09916 magnetometer is connected to the ICM-20948's internal I2C bus. We need to use the ICM-20948's I2C master controller to talk to it.

There are two approaches:

#### Approach A: Automatic Mode (Easier)
Configure the ICM-20948 to automatically read the mag and place the data in EXT_SENS_DATA registers.

```rust
impl<I2C: embedded_hal::i2c::I2c> Icm20948<I2C> {
    fn init_magnetometer(&mut self, delay: &mut impl DelayMs<u8>) -> Result<(), I2C::Error> {
        // Bank 0: Enable I2C master mode
        self.write_register(0, USER_CTRL, 0x20)?; // I2C_MST_EN

        // Bank 3: Configure I2C master clock speed
        self.write_register(3, I2C_MST_CTRL, 0x07)?; // 400 kHz

        // Bank 3: Configure slave 0 to read from magnetometer WHO_AM_I
        self.write_register(3, I2C_SLV0_ADDR, AK09916_ADDR | 0x80)?; // Read mode
        self.write_register(3, I2C_SLV0_REG, AK09916_WIA2)?;
        self.write_register(3, I2C_SLV0_CTRL, 0x81)?; // Enable, read 1 byte

        delay.delay_ms(10);

        // Check WHO_AM_I (should be 0x09)
        let whoami = self.read_register(0, EXT_SENS_DATA_00)?;
        if whoami != 0x09 {
            return Err(...);
        }

        // Set magnetometer to continuous measurement mode
        // (Need to write to mag's control register via I2C master)
        self.write_register(3, I2C_SLV0_ADDR, AK09916_ADDR)?; // Write mode
        self.write_register(3, I2C_SLV0_REG, AK09916_CNTL2)?;
        self.write_register(3, I2C_SLV0_DO, 0x08)?; // Continuous mode 4 (100Hz)
        self.write_register(3, I2C_SLV0_CTRL, 0x81)?; // Enable, write 1 byte

        delay.delay_ms(10);

        // Configure slave 0 to automatically read mag data (8 bytes)
        self.write_register(3, I2C_SLV0_ADDR, AK09916_ADDR | 0x80)?; // Read mode
        self.write_register(3, I2C_SLV0_REG, AK09916_ST1)?; // Start at ST1
        self.write_register(3, I2C_SLV0_CTRL, 0x88)?; // Enable, read 8 bytes

        Ok(())
    }

    pub fn read_mag(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        // Read from EXT_SENS_DATA registers where ICM puts mag data
        let mut buffer = [0u8; 8];
        self.read_registers(0, EXT_SENS_DATA_00, &mut buffer)?;

        // ST1 is buffer[0], check DRDY bit
        if buffer[0] & 0x01 == 0 {
            return Err(...); // Data not ready
        }

        // Mag data is little-endian (unlike accel/gyro!)
        let x = i16::from_le_bytes([buffer[1], buffer[2]]);
        let y = i16::from_le_bytes([buffer[3], buffer[4]]);
        let z = i16::from_le_bytes([buffer[5], buffer[6]]);

        // ST2 is buffer[7], check overflow bit
        if buffer[7] & 0x08 != 0 {
            return Err(...); // Magnetic overflow
        }

        // AK09916 sensitivity: 0.15 µT/LSB
        const MAG_SCALE: f32 = 0.15;

        Ok((
            x as f32 * MAG_SCALE,
            y as f32 * MAG_SCALE,
            z as f32 * MAG_SCALE,
        ))
    }
}
```

**Key Registers (Bank 0):**
- `USER_CTRL` (0x03) - Enable I2C master
- `EXT_SENS_DATA_00-23` (0x3B-0x52) - External sensor data from I2C master

**Key Registers (Bank 3):**
- `I2C_MST_CTRL` (0x01) - I2C master clock config
- `I2C_SLV0_ADDR` (0x03) - Slave 0 I2C address
- `I2C_SLV0_REG` (0x04) - Slave 0 register address
- `I2C_SLV0_CTRL` (0x05) - Slave 0 control (enable, length)
- `I2C_SLV0_DO` (0x06) - Slave 0 data out (for writes)

**AK09916 Magnetometer:**
- I2C Address: 0x0C
- `WIA2` (0x01) - WHO_AM_I, should read 0x09
- `ST1` (0x10) - Status 1 (data ready)
- `HXL-HZH` (0x11-0x16) - Mag data (X, Y, Z - little endian!)
- `ST2` (0x18) - Status 2 (overflow)
- `CNTL2` (0x31) - Control (power, mode)
- `CNTL3` (0x32) - Control 3 (soft reset)

#### Approach B: Manual Mode (More Control)
Manually send I2C commands to the magnetometer when we want to read.

This gives more flexibility but is more complex and slower.

### 5. Configuration Helpers (Optional)

```rust
pub enum AccelRange {
    G2,   // ±2g
    G4,   // ±4g
    G8,   // ±8g
    G16,  // ±16g
}

pub enum GyroRange {
    Dps250,   // ±250 degrees/second
    Dps500,   // ±500 degrees/second
    Dps1000,  // ±1000 degrees/second
    Dps2000,  // ±2000 degrees/second
}

impl<I2C: embedded_hal::i2c::I2c> Icm20948<I2C> {
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), I2C::Error> {
        // Update ACCEL_CONFIG register and self.accel_scale
    }

    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), I2C::Error> {
        // Update GYRO_CONFIG_1 register and self.gyro_scale
    }
}
```

## Implementation Effort Estimates

### Minimum Viable (Accel + Gyro only)
**Time:** 2-3 hours
- Basic register access with bank switching
- Initialization sequence
- Read accel/gyro, apply scaling
- Tested and working

### Full Featured (Accel + Gyro + Mag + Temp)
**Time:** 6-10 hours
- Everything above
- Magnetometer via I2C master in auto mode
- Temperature reading
- Configuration helpers for ranges
- Good error handling

### Production Quality
**Time:** 20+ hours
- All the above
- FIFO buffer support
- Interrupt configuration
- DMP (Digital Motion Processor) support
- Comprehensive testing
- Full documentation
- Examples

## Resources

- **Datasheet:** [ICM-20948 Datasheet v1.3](https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf)
- **AK09916 Datasheet:** [AK09916 Datasheet](https://www.akm.com/content/dam/documents/products/electronic-compass/ak09916c/ak09916c-en-datasheet.pdf)
- **Register Map:** See Section 6 and 7 of ICM-20948 datasheet
- **Application Note:** TDK has app notes on using the I2C master for magnetometer access

## Advantages Summary

1. ✅ **Modern embedded-hal 1.0** - No compatibility shims
2. ✅ **Full 9-DoF support** - Accelerometer, gyroscope, AND magnetometer
3. ✅ **Cleaner code** - Tailored to our exact needs
4. ✅ **Better understanding** - Know exactly how the hardware works
5. ✅ **Maintainable** - We control the code and can fix issues
6. ✅ **Embassy-friendly** - Can make async if needed later
7. ✅ **Temperature sensor** - Bonus, easy to add

## Next Steps When Ready

1. Create `src/icm20948.rs` module
2. Implement basic register access
3. Implement initialization
4. Implement accel/gyro reading
5. Test thoroughly
6. Add magnetometer support
7. Add configuration helpers as needed

## Notes

- The ICM-20948 can also act as an SPI device, but we're using I2C
- Default I2C address is 0x69 (can be 0x68 if AD0 pin is low)
- The device has extensive FIFO and interrupt capabilities we can add later if needed
- Sample rates are configurable up to 1.125 kHz for accel/gyro
- Magnetometer max rate is 100 Hz (limited by AK09916)
