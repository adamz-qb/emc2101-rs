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
    FanConfig = 0x4A,
    FanSetting = 0x4C,
    PWMFrequency = 0x4D,
    PWMFrequencyDivide = 0x4E,
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
    /// I2C bus error.
    I2c(E),
    /// The device Product ID is not supported.
    InvalidID,
    /// The given Value is not valid.
    InvalidValue,
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
    /// Initializes the EMC2101 driver.
    ///
    /// This consumes the I2C bus `I`. Before you can get temperature and fan measurements,
    /// you must call the `init` method which calibrates the sensor. The address will almost always
    /// be `SENSOR_ADDRESS` from this crate.
    pub fn new(i2c: I, address: u8) -> Result<Self, Error<E>> {
        let mut emc2101 = EMC2101 { i2c, address };
        emc2101.check_id()?;
        Ok(emc2101)
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

    /// enable_tach_input configure ALERT#/TACH pin as high impedance TACH input.
    /// This may require an external pull-up resistor to set the proper signaling levels.
    pub fn enable_tach_input(&mut self) -> Result<&mut Self, Error<E>> {
        // Set Configuration[2] ALT_TCH : The ALERT#/TACH pin will function as high impedance TACH input.
        self.update_reg(Register::Configuration, 0b0000_0100, 0)?;
        Ok(self)
    }

    /// enable_alert_input configure ALERT#/TACH pin as open drain active low ALERT# interrupt.
    pub fn enable_alert_input(&mut self) -> Result<&mut Self, Error<E>> {
        // Clear Configuration[2] ALT_TCH : The ALERT#/TACH pin will function as open drain,
        // active low interrupt.
        self.update_reg(Register::Configuration, 0, 0b0000_0100)?;
        Ok(self)
    }

    /// set_fan_pwm set FAN in PWM mode and configure it's base frequency.
    pub fn set_fan_pwm(&mut self, freq_hz: u32) -> Result<&mut Self, Error<E>> {
        match freq_hz {
            1_400 => {
                // Set FanConfig[3] CLK_SEL : The base clock that is used to determine the PWM
                // frequency is 1.4kHz.
                // Clear FanConfig[2] CLK_OVR : The base clock frequency is determined by the
                // CLK_SEL bit.
                self.update_reg(Register::FanConfig, 0b0000_1000, 0b0000_0100)?;
            }
            360_000 => {
                // Clear FanConfig[3] CLK_SEL : The base clock that is used to determine the PWM
                // frequency is 360kHz.
                // Clear FanConfig[2] CLK_OVR : The base clock frequency is determined by the
                // CLK_SEL bit.
                self.update_reg(Register::FanConfig, 0, 0b0000_1100)?;
            }
            23..=160_000 => {
                // Set FanConfig[2] CLK_OVR : The base clock that is used to determine the PWM frequency
                // is set by the Frequency Divide Register.
                self.update_reg(Register::FanConfig, 0b0000_0010, 0b0000_1000)?;
                // The PWM frequency when the PWMFrequencyDivide Register is used is shown in equation :
                // PWM_D = (360k / (2 * PWM_F * FREQ))
                let div: u16 = (160_000u32 / freq_hz) as u16;
                let pwm_f: u8 = (div >> 8) as u8 & 0x1F;
                let pwm_d: u8 = (div & 0xFF) as u8;
                // The PWMFrequency Register determines the final PWM frequency and "effective resolution"
                // of the PWM driver.
                self.write_reg(Register::PWMFrequency, pwm_f)?;
                // When the CLK_OVR bit is set to a logic '1', the PWMFrequencyDivide Register is used in
                // conjunction with the PWMFrequency Register to determine the final PWM frequency that the
                // load will see.
                self.write_reg(Register::PWMFrequencyDivide, pwm_d)?;
            }
            _ => {
                defmt::error!("Invalid PWM Frequency.");
                return Err(Error::InvalidValue);
            }
        }
        // Clear Configuration[4] DAC : PWM output enabled at FAN pin.
        self.update_reg(Register::Configuration, 0, 0b0001_0000)?;
        Ok(self)
    }

    /// set_fan_dac set FAN in DAC mode.
    pub fn set_fan_dac(&mut self) -> Result<&mut Self, Error<E>> {
        // Set Configuration[4] DAC : DAC output enabled at FAN pin.
        self.update_reg(Register::Configuration, 0b0001_0000, 0)?;
        Ok(self)
    }

    /// set_fan_power set the FAN power in percent (for both modes PWM/DAC).
    pub fn set_fan_power(&mut self, percent: u8) -> Result<&mut Self, Error<E>> {
        if percent > 100 {
            defmt::error!("Invalid Fan Power.");
            return Err(Error::InvalidValue);
        }
        let val: u8 = (percent * 64 / 100) as u8;
        // The FanSetting Register drives the fan driver when the Fan Control Look-Up Table is not used.
        // Any data written to the FanSetting register is applied immediately to the fan driver (PWM or DAC).
        self.write_reg(Register::FanSetting, val)?;
        Ok(self)
    }

    /// write_reg blindly write a single register with a fixed value.
    fn write_reg<R: Into<u8>>(&mut self, reg: R, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[reg.into(), value])
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// update_reg first read the register value, apply a set mask, then a clear mask, and write the new value
    /// only if different from the initial value.
    fn update_reg<R: Into<u8>>(
        &mut self,
        reg: R,
        mask_set: u8,
        mask_clear: u8,
    ) -> Result<(), Error<E>> {
        let reg = reg.into();
        let mut buf = [0x00];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .map_err(Error::I2c)?;
        let current = buf[0];
        buf[0] |= mask_set;
        buf[0] &= !mask_clear;
        if current != buf[0] {
            self.i2c
                .write(self.address, &[reg, buf[0]])
                .map_err(Error::I2c)?;
        }
        Ok(())
    }

    /// read_reg read a register value.
    fn read_reg<R: Into<u8>>(&mut self, reg: R) -> Result<u8, Error<E>> {
        let mut buf = [0x00];
        self.i2c
            .write_read(self.address, &[reg.into()], &mut buf)
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Destroys this driver and releases the I2C bus `I`.
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
        let expectations = vec![Transaction::write_read(
            SENSOR_ADDRESS,
            vec![super::Register::ProductID as u8],
            vec![super::ProductID::EMC2101 as u8],
        )];
        // In the real app we'd used shared-bus to share the i2c bus between the two drivers, but
        // I think this is fine for a test.
        let mock_i2c_1 = I2cMock::new(&expectations);
        let mock_i2c_2 = I2cMock::new(&expectations);

        let _emc2101_1 = EMC2101::new(mock_i2c_1, SENSOR_ADDRESS).unwrap();
        let _emc2101_2 = EMC2101::new(mock_i2c_2, SENSOR_ADDRESS).unwrap();
    }
}
