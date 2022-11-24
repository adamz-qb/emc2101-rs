#![cfg_attr(not(test), no_std)]
//! EMC2101 driver.

use embedded_hal::blocking::i2c;

/// EMC2101 sensor's I2C address.
pub const SENSOR_ADDRESS: u8 = 0b0100_1100; // This is I2C address 0x4C;

/// EMC2101 sensor's Product ID.
pub enum ProductID {
    EMC2101 = 0x16,
    EMC2101R = 0x28,
}

/// Commands that can be sent to the EMC2101 sensor.
pub enum Command {
    ProductID = 0b1111_1101, // 0xFD
}

/// Driver errors.
#[derive(Debug, PartialEq)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),
    /// The device Product ID is not supported
    InvalidID,
    /// Errors such as overflowing the stack.
    Internal,
}

/// An EMC2101 sensor on the I2C bus `I`.
///
/// The address of the sensor will be `SENSOR_ADDRESS` from this package, unless there is some kind
/// of special address translating hardware in use.
pub struct EMC2101<I>
where
    I: i2c::Read + i2c::Write,
{
    i2c: I,
    address: u8,
}

impl<E, I> EMC2101<I>
where
    I: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// Initializes the SCD30 driver.
    ///
    /// This consumes the I2C bus `I`. Before you can get temperature and fan measurements,
    /// you must call the `init` method which calibrates the sensor. The address will almost always
    /// be `SENSOR_ADDRESS` from this crate.
    pub fn new(i2c: I, address: u8) -> Self {
        EMC2101 { i2c, address }
    }

    /// Run the EMC2101 init routine.
    pub fn init(
        &mut self,
    ) -> Result<(), Error<E>> {
    // ) -> Result<EMC2101Initialized<I>, Error<E>> {

        self.check_id()?;

        Ok(())
        // Ok(EMC2101Initialized { EMC2101: self })
    }

    /// check_id asks the EMC2101 sensor to report its Product ID.
    fn check_id(&mut self) -> Result<ProductID, Error<E>> {
        let command: [u8; 1] = [Command::ProductID as u8];
        let mut read_buffer = [0u8; 1];

        self.i2c.write(self.address, &command).map_err(Error::I2c)?;
        self.i2c
            .read(self.address, &mut read_buffer)
            .map_err(Error::I2c)?;

        let product_id_byte = read_buffer[0];
        if product_id_byte == ProductID::EMC2101 as u8 {
            return Ok(ProductID::EMC2101);
        }
        if product_id_byte == ProductID::EMC2101R as u8 {
            return Ok(ProductID::EMC2101R);
        }
        defmt::error!("Wrong chip ID.");
        Err(Error::InvalidID)
    }

    /// send_initialize sends the Initialize command to the sensor which make it calibrate.
    ///
    /// After sending initialize, there is a required 40ms wait period and verification
    /// that the sensor reports itself calibrated. See the `init` method.
    // fn send_initialize(&mut self) -> Result<(), Error<E>> {
    //     // Send CheckStatus, read one byte back.
    //     let command: [u8; 3] = [
    //         // Initialize = 0b1011_1110. Equivalent to 0xBE, Section 5.3, page 8, Table 9
    //         Command::Initialize as u8,
    //         // Two parameters as described in the datasheet. There is no indication what these
    //         // parameters mean, just that they should be provided. There is also no returned
    //         // value.
    //         0b0000_1000, // 0x08
    //         0b0000_0000, // 0x00
    //     ];

    //     self.i2c.write(self.address, &command).map_err(Error::I2c)?;

    //     Ok(())
    // }

    /// Destroys this driver and releases the I2C bus `I`
    pub fn destroy(self) -> Self {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::{EMC2101, SENSOR_ADDRESS};
    use embedded_hal_mock::i2c::Mock as I2cMock;
    use embedded_hal_mock::i2c::Transaction;

    /// Test creating new EMC2101 sensors.
    ///
    /// Test that we can create multiple EMC2101 devices. We test this because it's one of the
    /// measures of success for this driver.
    #[test]
    fn emc2101_new() {
        // In the real app we'd used shared-bus to share the i2c bus between the two drivers, but
        // I think this is fine for a test.
        let mock_i2c_1 = I2cMock::new(&[]);
        let mock_i2c_2 = I2cMock::new(&[]);

        let _emc2101_1 = EMC2101::new(mock_i2c_1, SENSOR_ADDRESS);
        let _emc2101_2 = EMC2101::new(mock_i2c_2, SENSOR_ADDRESS);
    }

    /// Test sending the CheckStatus i2c command, and read a status byte back.
    #[test]
    fn check_status() {
        let expectations = vec![
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::ProductID as u8]),
            Transaction::read(SENSOR_ADDRESS, vec![super::ProductID::EMC2101 as u8]),
        ];
        let mock_i2c = I2cMock::new(&expectations);

        let mut emc2101 = EMC2101::new(mock_i2c, SENSOR_ADDRESS);
        let dev_id = emc2101.check_id().unwrap();
        assert_eq!(dev_id as u8, super::ProductID::EMC2101 as u8);

        let mut mock = emc2101.destroy().i2c;
        mock.done(); // verify expectations
    }
}
