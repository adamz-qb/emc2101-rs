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

/// Registers of the EMC2101 sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Register {
    Configuration = 0x03,
    ProductID = 0xFD,
}

impl From<Register> for u8 {
    fn from(r: Register) -> u8 {
        r as u8
    }
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
    I: i2c::Read + i2c::Write + i2c::WriteRead,
{
    i2c: I,
    address: u8,
}

impl<E, I> EMC2101<I>
where
    I: i2c::Read<Error = E> + i2c::Write<Error = E> + i2c::WriteRead<Error = E>,
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
    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.check_id()?;
        self.enable_tach_input()
    }

    /// check_id asks the EMC2101 sensor to report its Product ID.
    fn check_id(&mut self) -> Result<ProductID, Error<E>> {
        let product_id_byte = self.read_reg(Register::ProductID)?;
        if product_id_byte == ProductID::EMC2101 as u8 {
            return Ok(ProductID::EMC2101);
        }
        if product_id_byte == ProductID::EMC2101R as u8 {
            return Ok(ProductID::EMC2101R);
        }
        defmt::error!("Wrong chip ID.");
        Err(Error::InvalidID)
    }

    /// enable_tach_input enable using the TACH/ALERT pin as an input to read the fan speed
    /// signal from a 4-pin fan.
    fn enable_tach_input(&mut self) -> Result<(), Error<E>> {
        self.update_reg(Register::Configuration, 0b0000_0010, 0)
    }
    
    // fn write_reg<R: Into<u8>>(&mut self, reg: R, value: u8) -> Result<(), Error<E>> {
    //     self.i2c.write(self.address, &[reg.into(), value]).map_err(Error::I2c)?;
    //     Ok(())
    // }

    fn update_reg<R: Into<u8>>(&mut self, reg: R, mask_set: u8, mask_clear: u8) -> Result<(), Error<E>> {
        let reg = reg.into();
        let mut buf = [0x00];
        self.i2c.write_read(self.address, &[reg], &mut buf).map_err(Error::I2c)?;
        buf[0] |= mask_set;
        buf[0] &= !mask_clear;
        self.i2c.write(self.address, &[reg, buf[0]]).map_err(Error::I2c)?;
        Ok(())
    }

    fn read_reg<R: Into<u8>>(&mut self, reg: R) -> Result<u8, Error<E>> {
        let mut buf = [0x00];
        self.i2c.write_read(self.address, &[reg.into()], &mut buf).map_err(Error::I2c)?;
        Ok(buf[0])
    }

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

    /// Test reading the Product ID Register.
    #[test]
    fn check_id() {
        let expectations = vec![
            Transaction::write_read(SENSOR_ADDRESS, vec![super::Register::ProductID as u8], vec![super::ProductID::EMC2101 as u8]),
        ];
        let mock_i2c = I2cMock::new(&expectations);

        let mut emc2101 = EMC2101::new(mock_i2c, SENSOR_ADDRESS);
        let dev_id = emc2101.check_id().unwrap();
        assert_eq!(dev_id as u8, super::ProductID::EMC2101 as u8);

        let mut mock = emc2101.destroy().i2c;
        mock.done(); // verify expectations
    }
}
