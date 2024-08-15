//! Base communication implementation for interacting with Scd30 device
//!
//! Copyright 2019 Ryan Kurte

use core::fmt::Debug;

use embedded_hal::i2c;
use log::trace;

use crate::device::*;
use crate::Error;

/// Base API for reading and writing to the device
/// This should not be required by consumers, but is exposed to support alternate use (or in future provide ModBus support)
pub trait Base<Err> {
    /// Write a command to the device with optional data
    fn write_command(&mut self, command: Command, data: Option<u16>) -> Result<(), Error<Err>>;
    /// Read information from the device
    fn read_command(&mut self, command: Command, data: &mut [u8]) -> Result<(), Error<Err>>;
}

/// Helper for device CRC-8 calculation
pub fn crc8(data: &[u8]) -> u8 {
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

/// Base implementation for I2C devices
impl<Conn, Err> Base<Err> for Conn
where
    Conn: i2c::I2c<Error = Err>,
    Err: Debug,
{
    fn write_command(&mut self, command: Command, data: Option<u16>) -> Result<(), Error<Err>> {
        let c = command as u16;

        let mut buff: [u8; 5] = [(c >> 8) as u8, (c & 0xFF) as u8, 0, 0, 0];

        let len = match data {
            Some(d) => {
                buff[2] = (d >> 8) as u8;
                buff[3] = (d & 0xFF) as u8;
                buff[4] = crc8(&buff[2..4]);
                5
            }
            None => 2,
        };

        trace!("Writing command: {:?} data: {:?}", c, data);

        self.write(DEFAULT_ADDRESS | I2C_WRITE_FLAG, &buff[..len])
            .map_err(|e| Error::Conn(e))
    }

    fn read_command(&mut self, command: Command, data: &mut [u8]) -> Result<(), Error<Err>> {
        // Write command to initialise read
        let c = command as u16;
        let cmd = [(c >> 8) as u8, (c & 0xFF) as u8];

        trace!("Writing command: {:x?}", cmd);

        // First write the read command
        self.write(DEFAULT_ADDRESS | I2C_WRITE_FLAG, &cmd)
            .map_err(|e| Error::Conn(e))?;

        // Then, read the data back
        self.read(DEFAULT_ADDRESS | I2C_READ_FLAG, data)
            .map_err(|e| Error::Conn(e))?;

        // Note: this two-phase approach is specified in the datasheet

        trace!("Read data: {:x?}", data);

        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_crc() {
        // Test vectors from datasheet
        let tests = &[
            ([0xbe, 0xef], 0x92),
            ([0x00, 0x00], 0x81),
            ([0x43, 0xDB], 0xCB),
        ];

        for t in tests {
            let v = crc8(&t.0);
            assert_eq!(v, t.1);
        }
    }
}
