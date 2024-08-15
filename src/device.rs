//! Scd30 device definitions
//!
//! Copyright 2019 Ryan Kurte

/// Scd30 default I2C address
/// (note this is shifted left 1 bit on the wire)
pub const DEFAULT_ADDRESS: u8 = 0x61;

pub const I2C_WRITE_FLAG: u8 = 0x00;
pub const I2C_READ_FLAG: u8 = 0x01;

pub const CRC_POLY: u8 = 0x31;
pub const CRC_INIT: u8 = 0xff;
pub const CRC_XOR: u8 = 0x00;

/// Scd30 I2C Command
/// Command and data are big endian 16-bit unsigned integers, all Command with data are followed by a CRC-8 checksum
#[derive(PartialEq, Clone, Debug)]
pub enum Command {
    /// Start continuous mode
    /// Data is a u16 representing pressure in mBar for compensation
    /// or zero for no pressure compensation
    StartContinuousMode = 0x0010,

    /// Stop continuous mode
    /// No associated data or CRC
    StopContinuousMode = 0x0104,

    /// Set interval for continuous measurement mode
    /// Data is a u16 in seconds between 2 and 1800
    SetMeasurementInterval = 0x4600,

    /// Fetch data ready status
    /// This returns 1 if data is available in the buffer, 0 otherwise
    GetDataReady = 0x0202,

    /// Read a measurement from the buffer
    ReadMeasurement = 0x0300,

    /// Enable or Disable Automatic Self Calibration (ASC)
    /// Data is a u16, 1 enables ASC and 0 disables ASC
    SetAfc = 0x5306,

    /// Set Forced Recalibration Value (FRC)
    /// This is used to compensate for sensor drift when a CO2 reference value is available
    /// Data is a u16 CO2 concentration in ppm
    SetFrc = 0x5204,

    /// Set temperature offset
    /// Data is a uint16 in degrees celsius * 100, ie. 43 degrees -> 430u16
    SetTempOffset = 0x5403,

    /// Set altitude compensation
    /// This allows NDIR CO2 sensing to be calibrated by altitude
    /// Data is uint16 in meters above sea level
    SetAltComp = 0x5102,

    /// Soft Reset the device
    /// No associated data or CRC
    SoftReset = 0xd304,

    GetFirmwareVersion = 0xD100,
}
