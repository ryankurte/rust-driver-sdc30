//! Scd30 driver library
//!
//! Copyright 2019 Ryan Kurte
//!
//! ``` no_run
//! use std::time::Duration;
//! use linux_embedded_hal::{I2cdev, Delay};
//! use sensor_scd30::Scd30;
//!
//! // Open I2C port
//! let i2c = I2cdev::new("/dev/i2c-1").unwrap();
//!
//! // Connect to sensor
//! let mut scd = Scd30::new(i2c, Delay{}).unwrap();
//!
//! // Start continuous sampling mode
//! scd.start_continuous(10).unwrap();
//!
//! // Poll for data
//! loop {
//!     // Keep looping until ready
//!     if scd.data_ready().unwrap() {
//!         continue;
//!     }
//!
//!     // Fetch data when available
//!     let m = scd.read_data().unwrap();
//!     println!("Measurement: {:?}", m);
//! }
//!
//!
//! ```

#![no_std]

use core::fmt::Debug;
use core::marker::PhantomData;

use embedded_hal::delay::DelayNs;

pub mod device;
use device::*;

pub mod base;
use base::*;

/// Scd30 sensor object
/// This is generic over an I2C connector and associated error type
pub struct Scd30<Conn, Delay, Err> {
    conn: Conn,
    delay: Delay,
    _err: PhantomData<Err>,
}

/// Scd30 error object
#[derive(Debug)]
pub enum Error<ConnErr> {
    Conn(ConnErr),
    Crc(u8, u8),
    NoDevice,
}

impl<ConnErr> From<ConnErr> for Error<ConnErr> {
    fn from(conn_err: ConnErr) -> Self {
        Error::Conn(conn_err)
    }
}

/// Scd30 measurement object
#[derive(PartialEq, Clone, Debug)]
pub struct Measurement {
    /// CO2 concentration in parts-per-million (PPM)
    /// Range: 0 - 10,000
    pub co2: f32,
    /// Temperature in degrees celsius
    /// Range: -40 - 125 C
    pub temp: f32,
    /// Relative Humidity (%)
    /// Range: 0 - 100
    pub rh: f32,
}

impl<Conn, Delay, Err> Scd30<Conn, Delay, Err>
where
    Conn: Base<Err>,
    Delay: DelayNs,
    Err: Debug,
{
    /// Create a new Scd30 sensor instance
    pub fn new(conn: Conn, delay: Delay) -> Result<Self, Error<Err>> {
        // Create sensor object
        let mut s = Scd30 {
            conn,
            delay,
            _err: PhantomData,
        };

        // Check communication
        let v = s.firmware_version()?;
        if v == 0x00 || v == 0xFF {
            return Err(Error::NoDevice);
        }

        // Return sensor
        Ok(s)
    }

    /// Start continuous sensing mode with optional pressure compensation
    /// pressure_compensation should either be the current pressure in millibar or 0 to disable compensation
    pub fn start_continuous(&mut self, pressure_compensation: u16) -> Result<(), Error<Err>> {
        self.conn
            .write_command(Command::StartContinuousMode, Some(pressure_compensation))
    }

    /// Stop continuous sensing mode
    pub fn stop_continuous(&mut self) -> Result<(), Error<Err>> {
        self.conn.write_command(Command::StopContinuousMode, None)
    }

    /// Configure measurement interval in seconds
    pub fn set_measurement_interval(&mut self, interval: u16) -> Result<(), Error<Err>> {
        self.conn
            .write_command(Command::SetMeasurementInterval, Some(interval))
    }

    /// Enable or disable Automatic Self-Calibration
    pub fn set_afc(&mut self, enabled: bool) -> Result<(), Error<Err>> {
        let v = match enabled {
            true => 1,
            false => 0,
        };

        self.conn.write_command(Command::SetAfc, Some(v))
    }

    /// Set Forced Recalibration Value
    /// This allows the sensor to be recalibrates using a reference CO2 source
    pub fn set_frc(&mut self, cal_ppm: u16) -> Result<(), Error<Err>> {
        self.conn.write_command(Command::SetFrc, Some(cal_ppm))
    }

    /// Set Temperature Compensation
    /// Allows compensation for temperature variation during operation
    pub fn set_temp_offset(&mut self, temperature: f32) -> Result<(), Error<Err>> {
        let temperature = (temperature as u16) * 100;
        self.conn
            .write_command(Command::SetTempOffset, Some(temperature))
    }

    /// Set Altitude Compensation
    /// Allows compensation for CO2 measurement using altitude over sea level
    pub fn set_alt_offset(&mut self, altitude: u16) -> Result<(), Error<Err>> {
        self.conn.write_command(Command::SetAltComp, Some(altitude))
    }

    /// Soft reset the underlying device
    pub fn soft_reset(&mut self) -> Result<(), Error<Err>> {
        self.conn.write_command(Command::SoftReset, None)
    }

    /// Fetch the device firmware version
    pub fn firmware_version(&mut self) -> Result<u16, Error<Err>> {
        let mut buff = [0u8; 3];

        self.conn
            .read_command(Command::GetFirmwareVersion, &mut buff)?;

        let crc = crc8(&buff[..2]);
        if crc != buff[2] {
            return Err(Error::Crc(crc, buff[2]));
        }

        let v: u16 = (buff[0] as u16) << 8 | (buff[1] as u16);

        Ok(v)
    }

    /// Check whether measurement data is available in the buffer
    pub fn data_ready(&mut self) -> Result<bool, Error<Err>> {
        let mut buff = [0u8; 3];

        self.conn.read_command(Command::GetDataReady, &mut buff)?;

        let crc = crc8(&buff[..2]);
        if crc != buff[2] {
            return Err(Error::Crc(crc, buff[2]));
        }

        Ok(buff[1] != 0)
    }

    /// Read measurement data from the buffer
    pub fn read_data(&mut self) -> Result<Measurement, Error<Err>> {
        let mut buff = [0u8; 18];

        self.conn
            .read_command(Command::ReadMeasurement, &mut buff)?;

        let co2 = convert(&buff[0..6])?;
        let temp = convert(&buff[6..12])?;
        let rh = convert(&buff[12..18])?;

        Ok(Measurement { co2, temp, rh })
    }
}

/// Convert from a 6-byte response line into an F32 value
fn convert<E>(line: &[u8]) -> Result<f32, Error<E>> {
    // Lines MUST be 6 bytes long (MMSB, MLSB, CRC, LMSB, LLSB, CRC)
    assert_eq!(line.len(), 6);

    // Check CRC
    let crc1 = crc8(&line[0..2]);
    if crc1 != line[2] {
        return Err(Error::Crc(crc1, line[2]));
    }

    let crc2 = crc8(&line[3..5]);
    if crc2 != line[5] {
        return Err(Error::Crc(crc2, line[5]));
    }

    // Build temporary u32
    // Note the returned data is _big endian_
    let u: u32 = ((line[0] as u32) << 24)
        | ((line[1] as u32) << 16)
        | ((line[3] as u32) << 8)
        | ((line[4] as u32) << 0);

    // Transmute into float
    let v = f32::from_bits(u);

    Ok(v)
}

#[cfg(test)]
mod test {
    extern crate std;
    use std::vec;

    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    use embedded_hal_mock::eh1::MockError;

    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[test]
    fn test_start_continuous() {
        // Set up expectations
        let expectations = [I2cTransaction::write(
            DEFAULT_ADDRESS,
            vec![0x00, 0x10, 0x00, 0x00, 0x81],
        )];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Scd30 {
            conn: i2c.clone(),
            delay: NoopDelay {},
            _err: PhantomData,
        };

        // Start continuous mode
        sensor.start_continuous(0).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_stop_continuous() {
        // Set up expectations
        let expectations = [I2cTransaction::write(DEFAULT_ADDRESS, vec![0x01, 0x04])];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Scd30 {
            conn: i2c.clone(),
            delay: NoopDelay {},
            _err: PhantomData,
        };

        // Stop continuous mode
        sensor.stop_continuous().unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_set_measurement_interval() {
        // Set up expectations
        let expectations = [I2cTransaction::write(
            DEFAULT_ADDRESS,
            vec![0x46, 0x00, 0x00, 0x02, 0xE3],
        )];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Scd30 {
            conn: i2c.clone(),
            delay: NoopDelay {},
            _err: PhantomData,
        };

        // Set measurement interval to 2s
        sensor.set_measurement_interval(2).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_set_frc() {
        // Set up expectations
        let expectations = [I2cTransaction::write(
            DEFAULT_ADDRESS,
            vec![0x52, 0x04, 0x01, 0xc2, 0x50],
        )];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Scd30 {
            conn: i2c.clone(),
            delay: NoopDelay {},
            _err: PhantomData,
        };

        // Set forced recalibration to 450ppm
        sensor.set_frc(450).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn set_temp_offset() {
        // Set up expectations
        let expectations = [I2cTransaction::write(
            DEFAULT_ADDRESS,
            vec![0x54, 0x03, 0x01, 0xF4, 0x33],
        )];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Scd30 {
            conn: i2c.clone(),
            delay: NoopDelay {},
            _err: PhantomData,
        };

        // Set temperature to 5 degrees
        sensor.set_temp_offset(5.0).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn set_alt_offset() {
        // Set up expectations
        let expectations = [I2cTransaction::write(
            DEFAULT_ADDRESS,
            vec![0x51, 0x02, 0x03, 0xE8, 0xD4],
        )];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Scd30 {
            conn: i2c.clone(),
            delay: NoopDelay {},
            _err: PhantomData,
        };

        // Set altitude to 1000m
        sensor.set_alt_offset(1000).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_soft_reset() {
        // Set up expectations
        let expectations = [I2cTransaction::write(DEFAULT_ADDRESS, vec![0xD3, 0x04])];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Scd30 {
            conn: i2c.clone(),
            delay: NoopDelay {},
            _err: PhantomData,
        };

        // Signal for soft reset
        sensor.soft_reset().unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_read_data_ready() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x02, 0x02]),
            I2cTransaction::read(DEFAULT_ADDRESS | I2C_READ_FLAG, vec![0x00, 0x01, 0xB0]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Scd30 {
            conn: i2c.clone(),
            delay: NoopDelay {},
            _err: PhantomData,
        };

        // Read data ready
        let ready = sensor.data_ready().unwrap();
        assert!(ready);

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_read_measurement() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x03, 0x00]),
            I2cTransaction::read(
                DEFAULT_ADDRESS | I2C_READ_FLAG,
                vec![
                    0x43, 0xDB, 0xCB, 0x8C, 0x2E, 0x8F, // CO2: 439 ppm
                    0x41, 0xD9, 0x70, 0xE7, 0xFF, 0xF5, // Temperature: 27.2 C
                    0x42, 0x43, 0xBF, 0x3A, 0x1B, 0x74, // Relative humidity, 48.8 %
                ],
            ),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Scd30 {
            conn: i2c.clone(),
            delay: NoopDelay {},
            _err: PhantomData,
        };

        // Read measurement
        let m = sensor.read_data().unwrap();

        assert_approx_eq!(m.co2, 439.0, 0.1);
        assert_approx_eq!(m.temp, 27.2, 0.1);
        assert_approx_eq!(m.rh, 48.8, 0.1);

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_convert() {
        // Test vectors from datasheet
        let tests = &[
            ([0x43, 0xDB, 0xCB, 0x8C, 0x2E, 0x8F], 439.0),
            ([0x41, 0xD9, 0x70, 0xE7, 0xFF, 0xF5], 27.2),
            ([0x42, 0x43, 0xBF, 0x3A, 0x1B, 0x74], 48.8),
        ];

        for t in tests {
            let v = convert::<()>(&t.0).unwrap();
            assert_approx_eq!(v, t.1, 0.1);
        }
    }
}
