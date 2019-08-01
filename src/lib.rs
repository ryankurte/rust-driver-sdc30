//! Sdc30 driver library
//! 
//! Copyright 2019 Ryan Kurte

//#![no_std]

use core::fmt::Debug;
use core::marker::PhantomData;

extern crate embedded_hal;
use embedded_hal::blocking::i2c;

#[macro_use] extern crate log;

pub mod device;
use device::*;

pub mod base;
use base::*;

/// Sdc30 sensor object
/// This is generic over an I2C connector and associated error type
pub struct Sdc30<Conn, Err> {
    conn: Conn,
    _err: PhantomData<Err>,
}

/// Sdc30 error object
#[derive(Debug)]
pub enum Error<ConnErr> {
    Conn(ConnErr),
    Crc(u8, u8),
    NoDevice,
}

impl <ConnErr> From<ConnErr> for Error<ConnErr> {
    fn from(conn_err: ConnErr) -> Self {
        Error::Conn(conn_err)
    }
}

/// Sdc30 measurement object
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

impl <Conn, Err> Sdc30 <Conn, Err> where
    Conn: i2c::Read<Error=Err> + i2c::Write<Error=Err> + i2c::WriteRead<Error=Err>,
    Err: Debug,
{
    /// Create a new Sdc30 sensor instance
    pub fn new(conn: Conn) -> Result<Self, Error<Err>> {
        // Create sensor object
        let mut s = Sdc30{ conn, _err: PhantomData };

        // Check communication
        let v = s.firmware_version()?;
        if v == 0x00 || v == 0xFF {
            return Err(Error::NoDevice)
        }

        // Return sensor
        Ok(s)
    }

    /// Start continuous sensing mode with optional pressure compensation
    /// pressure_compensation should either be the current pressure in millibar or 0 to disable compensation
    pub fn start_continuous(&mut self, pressure_compensation: u16) -> Result<(), Error<Err>> {
        self.write_command(Command::StartContinuousMode, Some(pressure_compensation))
    }

    /// Stop continuous sensing mode
    pub fn stop_continuous(&mut self) -> Result<(), Error<Err>> {
        self.write_command(Command::StopContinuousMode, None) 
    }

    /// Configure measurement interval in seconds
    pub fn set_measurement_interval(&mut self, interval: u16) -> Result<(), Error<Err>> {
        self.write_command(Command::SetMeasurementInterval, Some(interval))
    }


    /// Enable or disable Automatic Self-Calibration
    pub fn set_afc(&mut self, enabled: bool) -> Result<(), Error<Err>> {
        let v = match enabled {
            true => 1,
            false => 0,
        };

        self.write_command(Command::SetAfc, Some(v))
    }


    /// Set Forced Recalibration Value
    /// This allows the sensor to be recalibrates using a reference CO2 source
    pub fn set_frc(&mut self, cal_ppm: u16) -> Result<(), Error<Err>> {
        self.write_command(Command::SetFrc, Some(cal_ppm))
    }

    /// Set Temperature Compensation
    /// Allows compensation for temperature variation during operation
    pub fn set_temp_offset(&mut self, temperature: f32) -> Result<(), Error<Err>> {
        let temperature = (temperature as u16) * 100;
        self.write_command(Command::SetTempOffset, Some(temperature))
    }

    /// Set Altitude Compensation
    /// Allows compensation for CO2 measurement using altitude over sea level
    pub fn set_alt_offset(&mut self, altitude: u16) -> Result<(), Error<Err>> {
        self.write_command(Command::SetAltComp, Some(altitude))
    }

    /// Soft reset the underlying device
    pub fn soft_reset(&mut self) -> Result<(), Error<Err>> {
        self.write_command(Command::SoftReset, None)
    }

    /// Fetch the device firmware version
    pub fn firmware_version(&mut self) -> Result<u16, Error<Err>> {
        let mut buff = [0u8; 3];

        self.read_command(Command::GetFirmwareVersion, &mut buff)?;

        let crc = Self::crc(&buff[..2]);
        if crc != buff[2] {
            return Err(Error::Crc(crc, buff[2]));
        }

        let v: u16 = (buff[0] as u16) << 8 | (buff[1] as u16);

        Ok(v)
    }

    /// Check whether measurement data is available in the buffer
    pub fn data_ready(&mut self) -> Result<bool, Error<Err>> {
        let mut buff = [0u8; 3];

        self.read_command(Command::GetDataReady, &mut buff)?;

        let crc = Self::crc(&buff[..2]);
        if crc != buff[2] {
            return Err(Error::Crc(crc, buff[2]));
        }

        Ok(buff[1] != 0)
    }

    /// Read measurement data from the buffer
    pub fn read_data(&mut self) -> Result<Measurement, Error<Err>> {
        let mut buff = [0u8; 18];

        self.read_command(Command::ReadMeasurement, &mut buff)?;

        let co2 = Self::convert(&buff[0..6])?;
        let temp = Self::convert(&buff[6..12])?;
        let rh = Self::convert(&buff[12..18])?;

        Ok(Measurement{co2, temp, rh})
    }


    /// Convert from a 6-byte response line into an F32 value
    fn convert(line: &[u8]) -> Result<f32, Error<Err>> {
        // Lines MUST be 6 bytes long (MMSB, MLSB, CRC, LMSB, LLSB, CRC)
        assert_eq!(line.len(), 6);

        // Check CRC
        let crc1 = Self::crc(&line[0..2]);
        if crc1 != line[2] {
            return Err(Error::Crc(crc1, line[2]));
        }

        let crc2 = Self::crc(&line[3..5]);
        if crc2 != line[5] {
            return Err(Error::Crc(crc2, line[5]));
        }
        
        // Build temporary u32
        // Note the returned data is _big endian_
        let u: u32 = ((line[0] as u32) << 24 )| ((line[1] as u32) << 16 )
            | ((line[3] as u32) << 8) | ((line[4] as u32) << 0);

        // Transmute into float
        let v = f32::from_bits(u);

        Ok(v)
    }

    /// Helper for device CRC-8 calculation
    fn crc(data: &[u8]) -> u8 {
        let mut crc = CRC_INIT;

        // For each byte
        for v in data {
            // XOR with current byte
            crc ^= v;

            // For each bit (in -ve order, but, doesn't actually matter here)
            for _bit in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ CRC_POLY;
                } else {
                    crc = crc << 1;
                }
            }
        }

        // Apply final xor    
        crc ^ CRC_XOR
    }
}



#[cfg(test)]
mod test {
    extern crate std;
    use std::vec;

    extern crate embedded_hal_mock;
    use embedded_hal_mock::MockError;
    use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    extern crate assert_approx_eq;
    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[test]
    fn test_start_continuous() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x00, 0x10, 0x00, 0x00, 0x81]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30{ conn: i2c.clone(), _err: PhantomData };

        // Start continuous mode
        sensor.start_continuous(0).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_stop_continuous() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x01, 0x04]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30{ conn: i2c.clone(), _err: PhantomData };

        // Stop continuous mode
        sensor.stop_continuous().unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_set_measurement_interval() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x46, 0x00, 0x00, 0x02, 0xE3]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30{ conn: i2c.clone(), _err: PhantomData };

        // Set measurement interval to 2s
        sensor.set_measurement_interval(2).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_set_frc() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x52, 0x04, 0x01, 0xc2, 0x50]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30{ conn: i2c.clone(), _err: PhantomData };

        // Set forced recalibration to 450ppm
        sensor.set_frc(450).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn set_temp_offset() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x54, 0x03, 0x01, 0xF4, 0x33]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30{ conn: i2c.clone(), _err: PhantomData };

        // Set temperature to 5 degrees
        sensor.set_temp_offset(5.0).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn set_alt_offset() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x51, 0x02, 0x03, 0xE8, 0xD4]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30{ conn: i2c.clone(), _err: PhantomData };

        // Set altitude to 1000m
        sensor.set_alt_offset(1000).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_soft_reset() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0xD3, 0x04]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30{ conn: i2c.clone(), _err: PhantomData };

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
        let mut sensor = Sdc30{ conn: i2c.clone(), _err: PhantomData };

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
            I2cTransaction::read(DEFAULT_ADDRESS | I2C_READ_FLAG, vec![
                0x43, 0xDB, 0xCB, 0x8C, 0x2E, 0x8F, // CO2: 439 ppm
                0x41, 0xD9, 0x70, 0xE7, 0xFF, 0xF5, // Temperature: 27.2 C
                0x42, 0x43, 0xBF, 0x3A, 0x1B, 0x74, // Relative humidity, 48.8 %
            ]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30{ conn: i2c.clone(), _err: PhantomData };

        // Read measurement
        let m = sensor.read_data().unwrap();

        assert_approx_eq!(m.co2, 439.0, 0.1);
        assert_approx_eq!(m.temp, 27.2, 0.1);
        assert_approx_eq!(m.rh, 48.8, 0.1);

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_crc() {
        // Test vectors from datasheet
        let tests = &[
            ([0xbe, 0xef], 0x92),
            ([0x00, 0x00], 0x81),
            ([0x43, 0xDB], 0xCB),
        ];

        for t in tests {
            let v = Sdc30::<I2cMock, MockError>::crc(&t.0);
            assert_eq!(v, t.1);
        }
        
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
            let v = Sdc30::<I2cMock, MockError>::convert(&t.0).unwrap();
            assert_approx_eq!(v, t.1, 0.1);
        }
    }
}