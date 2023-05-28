#![cfg_attr(not(test), no_std)]
//! EMC2101 driver.

#[cfg(feature = "defmt")]
use defmt::{debug, error, trace, Format};
use embedded_hal::blocking::i2c;
use fugit::HertzU32;

/// EMC2101 sensor's I2C address.
pub const DEFAULT_ADDRESS: u8 = 0b0100_1100; // This is I2C address 0x4C;

/// EMC2101 sensor's Product ID.
pub enum ProductID {
    EMC2101 = 0x16,
    EMC2101R = 0x28,
}

/// ADC Conversion Rates.
pub enum ConversionRate {
    Rate1_16Hz = 0,
    Rate1_8Hz = 1,
    Rate1_4Hz = 2,
    Rate1_2Hz = 3,
    Rate1Hz = 4,
    Rate2Hz = 5,
    Rate4Hz = 6,
    Rate8Hz = 7,
    Rate16Hz = 8,
    Rate32Hz = 9,
}

/// ADC Filter Levels.
pub enum FilterLevel {
    Disabled = 0,
    Level1 = 1,
    Level2 = 3,
}

/// Registers of the EMC2101 sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Register {
    InternalTemperature = 0x00,
    ExternalTemperatureMSB = 0x01,
    Status = 0x02,
    Configuration = 0x03,
    ConversionRate = 0x04,
    InternalTempLimit = 0x05,
    ExternalTempLimitHigh = 0x07,
    ExternalTempLimitLow = 0x08,
    ExternalTemperatureForce = 0x0C,
    ExternalTemperatureLSB = 0x10,
    AlertMask = 0x16,
    ExternalTempCriticalLimit = 0x19,
    ExternalTempCriticalHysteresis = 0x21,
    TachLSB = 0x46,
    TachMSB = 0x47,
    FanConfig = 0x4A,
    FanSetting = 0x4C,
    PWMFrequency = 0x4D,
    PWMFrequencyDivide = 0x4E,
    FanControlLUTHysteresis = 0x4F,
    FanControlLUTT1 = 0x50,
    FanControlLUTS1 = 0x51,
    AveragingFilter = 0xBF,
    ProductID = 0xFD,
}

impl From<Register> for u8 {
    fn from(r: Register) -> u8 {
        r as u8
    }
}

/// Driver errors.
#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    /// I2C bus error.
    I2c(E),
    /// The device Product ID is not supported.
    InvalidID,
    /// The given Value is not valid.
    InvalidValue,
    /// The given array as an invalid size.
    InvalidSize,
    /// The given array is not sorted.
    InvalidSorting,
    /// Errors such as overflowing the stack.
    Internal,
}

/// Device Satuts.
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct Status {
    pub eeprom_error: bool,
    pub ext_diode_fault: bool,
    pub adc_busy: bool,
    pub temp_int_high: bool,
    pub temp_ext_high: bool,
    pub temp_ext_low: bool,
    pub temp_ext_critical: bool,
    pub tack_limit: bool,
}

/// Look-up Table
pub struct Level {
    pub temp: u8,
    pub percent: u8,
}

/// An EMC2101 sensor on the I2C bus `I`.
///
/// The address of the sensor will be `DEFAULT_ADDRESS` from this package, unless there is some kind
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
    /// This consumes the I2C bus `I`. The address will almost always
    /// be `DEFAULT_ADDRESS` from this crate.
    pub fn new_with_address(i2c: I, address: u8) -> Result<Self, Error<E>> {
        let mut emc2101 = EMC2101 { i2c, address };
        #[cfg(feature = "defmt")]
        trace!("new");
        emc2101.check_id()?;
        // Disable all alerts interrupt, will be enable one by one calling monitor_xxx() functions.
        emc2101.write_reg(Register::AlertMask, 0xFF)?;
        Ok(emc2101)
    }
    pub fn new(i2c: I) -> Result<Self, Error<E>> {
        EMC2101::new_with_address(i2c, DEFAULT_ADDRESS)
    }

    /// check_id asks the EMC2101 sensor to report its Product ID.
    fn check_id(&mut self) -> Result<ProductID, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("check_id");
        let product_id_byte = self.read_reg(Register::ProductID)?;
        if product_id_byte == ProductID::EMC2101 as u8 {
            return Ok(ProductID::EMC2101);
        }
        if product_id_byte == ProductID::EMC2101R as u8 {
            return Ok(ProductID::EMC2101R);
        }
        #[cfg(feature = "defmt")]
        error!("Wrong chip ID.");
        Err(Error::InvalidID)
    }

    /// get_status gives the device current status.
    pub fn get_status(&mut self) -> Result<Status, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("get_status");
        let s = self.read_reg(Register::Status)?;
        Ok(Status {
            eeprom_error: s & 0x20 == 0x20,
            ext_diode_fault: s & 0x04 == 0x04,
            adc_busy: s & 0x80 == 0x80,
            temp_int_high: s & 0x40 == 0x40,
            temp_ext_high: s & 0x10 == 0x10,
            temp_ext_low: s & 0x08 == 0x08,
            temp_ext_critical: s & 0x02 == 0x02,
            tack_limit: s & 0x01 == 0x01,
        })
    }

    /// configure_adc set the conversion rate in Hertz and the filter level.
    pub fn configure_adc(
        &mut self,
        rate: ConversionRate,
        filter: FilterLevel,
    ) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("configure_adc");
        // ConversionRate[3:0] : ADC conversion rate.
        self.write_reg(Register::ConversionRate, rate as u8)?;
        // AveragingFilter[2:1] FILTER[1:0] : control the level of digital filtering
        // that is applied to the External Diode temperature measurements.
        let f_set: u8 = (filter as u8) << 1;
        let f_clr: u8 = !f_set & 0x06;
        self.update_reg(Register::AveragingFilter, f_set, f_clr)?;
        Ok(self)
    }

    /// force_temp_external force the external temperature value to a virtual value.
    /// When determining the position of the Fan Control Look-up Table, the contents
    /// of the ExternalTemperatureForce Register will be used instead of the measured
    /// External Diode temperature as normal.
    pub fn force_temp_external(&mut self, value: i8) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("force_temp_external");
        self.write_reg(Register::ExternalTemperatureForce, value as u8)?;
        // Set FanConfig[6] FORCE : the ExternalTemperatureForce Register is used.
        self.update_reg(Register::FanConfig, 0b0100_0000, 0)?;
        Ok(self)
    }

    /// real_temp_external let the measured External Diode temperature be used to
    /// determine the position in the Fan Control Look-up Table.
    pub fn real_temp_external(&mut self) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("real_temp_external");
        // Clear FanConfig[6] FORCE : the ExternalTemperatureForce Register is not used.
        self.update_reg(Register::FanConfig, 0, 0b0100_0000)?;
        Ok(self)
    }

    /// get_temp_conversion_rate get the current conversion rate in Hertz.
    pub fn get_temp_conversion_rate(&mut self) -> Result<ConversionRate, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("get_temp_conversion_rate");
        let rate: ConversionRate = match self.read_reg(Register::ConversionRate)? {
            0 => ConversionRate::Rate1_16Hz,
            1 => ConversionRate::Rate1_8Hz,
            2 => ConversionRate::Rate1_4Hz,
            3 => ConversionRate::Rate1_2Hz,
            4 => ConversionRate::Rate1Hz,
            5 => ConversionRate::Rate2Hz,
            6 => ConversionRate::Rate4Hz,
            7 => ConversionRate::Rate8Hz,
            8 => ConversionRate::Rate16Hz,
            9..=15 => ConversionRate::Rate32Hz,
            _ => return Err(Error::InvalidValue),
        };
        Ok(rate)
    }

    /// get_temp_internal read the internal temperature value in degree Celsius.
    pub fn get_temp_internal(&mut self) -> Result<i8, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("get_temp_internal");
        let val = self.read_reg(Register::InternalTemperature)?;
        Ok(val as i8)
    }

    /// get_temp_external read the external temperature value in degree Celsius.
    pub fn get_temp_external(&mut self) -> Result<i8, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("get_temp_external");
        let val = self.read_reg(Register::ExternalTemperatureMSB)?;
        Ok(val as i8)
    }

    /// get_temp_external read the external temperature value in degree Celsius.
    pub fn get_temp_external_precise(&mut self) -> Result<f32, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("get_temp_external_precise");
        let msb = self.read_reg(Register::ExternalTemperatureMSB)?;
        let lsb = self.read_reg(Register::ExternalTemperatureLSB)?;
        let raw: i16 = (((msb as u16) << 8) + lsb as u16) as i16;
        let ret: f32 = (raw >> 5) as f32 * 0.125;
        Ok(ret)
    }

    /// monitor_temp_internal_high start monitoring the internal temperature and will create
    /// an alert when the temperature exceeds the limit.
    /// The temp_int_high will be true in Status until the internal temperature drops below the high limit.
    pub fn monitor_temp_internal_high(&mut self, limit: i8) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("monitor_temp_internal_high");
        // If the measured temperature for the internal diode exceeds the Internal Temperature limit,
        // then the INT_HIGH bit is set in the Status Register. It remains set until the internal
        // temperature drops below the high limit.
        self.write_reg(Register::InternalTempLimit, limit as u8)?;
        // Clear AlertMask[6] INT_MSK : The Internal Diode will generate an interrupt if measured temperature
        // exceeds the Internal Diode high limit.
        self.update_reg(Register::AlertMask, 0, 0b0100_0000)?;
        Ok(self)
    }

    /// monitor_temp_external_critical start monitoring the external temperature and will create
    /// an alert when the temperature exceeds the critical limit.
    /// The temp_ext_critical will be true in Status until the external temperature drops below the critical
    /// limit minus critical hysteresis.
    pub fn monitor_temp_external_critical(
        &mut self,
        limit: i8,
        hysteresis: u8,
    ) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("monitor_temp_external_critical");
        // If the external diode exceeds the TCRIT Temp limit (even if it does not exceeds the External Diode
        // Temperature Limit), the TCRIT bit is set in the Status Register. It remains set until the external
        // temperature drops below the Critical Limit minus the Critical Hysteresis.
        self.write_reg(Register::ExternalTempCriticalLimit, limit as u8)?;
        self.write_reg(Register::ExternalTempCriticalHysteresis, hysteresis)?;
        // Clear AlertMask[4] HIGH_MSK : The External Diode will generate an interrupt if measured temperature
        // exceeds the External Diode high limit.
        self.update_reg(Register::AlertMask, 0, 0b0001_0000)?;
        Ok(self)
    }

    /// monitor_temp_external_high start monitoring the external temperature and will create
    /// an alert when the temperature exceeds the high limit.
    /// The temp_ext_high will be true in Status until the external temperature drops below the high limit.
    pub fn monitor_temp_external_high(&mut self, limit: i8) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("monitor_temp_external_high");
        // If the measured temperature for the external diode exceeds the External Temperature High limit,
        // then the EXT_HIGH bit is set in the Status Register. It remains set until the external
        // temperature drops below the high limit.
        self.write_reg(Register::ExternalTempLimitHigh, limit as u8)?;
        // Clear AlertMask[4] HIGH_MSK : The External Diode will generate an interrupt if measured temperature
        // exceeds the External Diode high limit.
        self.update_reg(Register::AlertMask, 0, 0b0001_0000)?;
        Ok(self)
    }

    /// monitor_temp_external_low start monitoring the external temperature and will create
    /// an alert when the temperature drops below the low limit.
    /// The temp_ext_low will be true in Status until the external temperature exceeds the low limit.
    pub fn monitor_temp_external_low(&mut self, limit: i8) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("monitor_temp_external_low");
        // If the measured temperature for the external diode drops below the External Temperature Low limit,
        // then the EXT_LOW bit is set in the Status Register. It remains set until the external
        // temperature exceeds the low limit.
        self.write_reg(Register::ExternalTempLimitLow, limit as u8)?;
        // Clear AlertMask[3] LOW_MSK : The External Diode will generate an interrupt if measured temperature
        // drops below the External Diode low limit.
        self.update_reg(Register::AlertMask, 0, 0b0000_1000)?;
        Ok(self)
    }

    /// enable_alert_output configure ALERT#/TACH pin as open drain active low ALERT# interrupt.
    /// This may require an external pull-up resistor to set the proper signaling levels.
    pub fn enable_alert_output(&mut self) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("enable_alert_output");
        // Clear Configuration[2] ALT_TCH : The ALERT#/TACH pin will function as open drain,
        // active low interrupt.
        // Clear Configuration[7] MASK : The ALERT#/TACH pin will be asserted if any bit is set in the
        // Status Register. Once the pin is asserted, it remains asserted.
        self.update_reg(Register::Configuration, 0, 0b1000_0100)?;
        Ok(self)
    }

    /// enable_tach_input configure ALERT#/TACH pin as high impedance TACH input.
    pub fn enable_tach_input(&mut self) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("enable_tach_input");
        // Set Configuration[2] ALT_TCH : The ALERT#/TACH pin will function as high impedance TACH input.
        self.update_reg(Register::Configuration, 0b0000_0100, 0)?;
        Ok(self)
    }

    /// set_fan_pwm set FAN in PWM mode and configure it's base frequency.
    pub fn set_fan_pwm(
        &mut self,
        frequency: HertzU32,
        inverted: bool,
    ) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("set_fan_pwm");
        match frequency.raw() {
            1_400 => {
                // Set FanConfig[3] CLK_SEL : The base clock that is used to determine the PWM
                // frequency is 1.4kHz.
                // Clear FanConfig[2] CLK_OVR : The base clock frequency is determined by the
                // CLK_SEL bit.
                if inverted {
                    // Set FanConfig[4] POLARITY : The polarity of the Fan output driver is inverted.
                    // A 0x00 setting will correspond to a 100% duty cycle or maximum DAC output voltage.
                    self.update_reg(Register::FanConfig, 0b0001_1000, 0b0000_0100)?;
                } else {
                    // Clear FanConfig[4] POLARITY : The polarity of the Fan output driver is non-inverted.
                    // A 0x00 setting will correspond to a 0% duty cycle or minimum DAC output voltage.
                    self.update_reg(Register::FanConfig, 0b0000_1000, 0b0001_0100)?;
                }
            }
            360_000 => {
                // Clear FanConfig[3] CLK_SEL : The base clock that is used to determine the PWM
                // frequency is 360kHz.
                // Clear FanConfig[2] CLK_OVR : The base clock frequency is determined by the
                // CLK_SEL bit.
                if inverted {
                    // Set FanConfig[4] POLARITY : The polarity of the Fan output driver is inverted.
                    // A 0x00 setting will correspond to a 100% duty cycle or maximum DAC output voltage.
                    self.update_reg(Register::FanConfig, 0b0001_0000, 0b0000_1100)?;
                } else {
                    // Clear FanConfig[4] POLARITY : The polarity of the Fan output driver is non-inverted.
                    // A 0x00 setting will correspond to a 0% duty cycle or minimum DAC output voltage.
                    self.update_reg(Register::FanConfig, 0, 0b0001_1100)?;
                }
            }
            23..=160_000 => {
                // Set FanConfig[2] CLK_OVR : The base clock that is used to determine the PWM frequency
                // is set by the Frequency Divide Register.
                if inverted {
                    // Set FanConfig[4] POLARITY : The polarity of the Fan output driver is inverted.
                    // A 0x00 setting will correspond to a 100% duty cycle or maximum DAC output voltage.
                    self.update_reg(Register::FanConfig, 0b0001_0010, 0b0000_1000)?;
                } else {
                    // Clear FanConfig[4] POLARITY : The polarity of the Fan output driver is non-inverted.
                    // A 0x00 setting will correspond to a 0% duty cycle or minimum DAC output voltage.
                    self.update_reg(Register::FanConfig, 0b0000_0010, 0b0001_1000)?;
                }
                // The PWM frequency when the PWMFrequencyDivide Register is used is shown in equation :
                // PWM_D = (360k / (2 * PWM_F * FREQ))
                let div: u16 = (160_000u32 / frequency.raw()) as u16;
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
                #[cfg(feature = "defmt")]
                error!("Invalid PWM Frequency.");
                return Err(Error::InvalidValue);
            }
        }
        // Clear Configuration[4] DAC : PWM output enabled at FAN pin.
        self.update_reg(Register::Configuration, 0, 0b0001_0000)?;
        Ok(self)
    }

    /// set_fan_dac set FAN in DAC mode.
    pub fn set_fan_dac(&mut self, inverted: bool) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("set_fan_dac");
        // Set Configuration[4] DAC : DAC output enabled at FAN pin.
        self.update_reg(Register::Configuration, 0b0001_0000, 0)?;
        if inverted {
            // Set FanConfig[4] POLARITY : The polarity of the Fan output driver is inverted.
            // A 0x00 setting will correspond to a 100% duty cycle or maximum DAC output voltage.
            self.update_reg(Register::FanConfig, 0b0001_0000, 0)?;
        } else {
            // Clear FanConfig[4] POLARITY : The polarity of the Fan output driver is non-inverted.
            // A 0x00 setting will correspond to a 0% duty cycle or minimum DAC output voltage.
            self.update_reg(Register::FanConfig, 0, 0b0001_0000)?;
        }
        Ok(self)
    }

    /// set_fan_power set the FAN power in percent (for both modes PWM/DAC).
    /// The 'power' must be between 0 and 100%.
    /// If Look-up Table was enabled, it will be disabled and the fixed power value will be used.
    pub fn set_fan_power(&mut self, percent: u8) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("set_fan_power");
        if percent > 100 {
            #[cfg(feature = "defmt")]
            error!("Invalid Fan Power.");
            return Err(Error::InvalidValue);
        }
        let fan_config: u8 = self.read_reg(Register::FanConfig)?;
        // FanConfig[5] PROG == 0 :
        // the FanSetting Register and Fan Control Look-Up Table Registers are read-only.
        if fan_config & 0x20 == 0x00 {
            // Set FanConfig[5] PROG : the FanSetting Register and Fan Control Look-Up
            // Table Registers can be written and the Fan Control Look-Up Table Registers
            // will not be used.
            self.write_reg(Register::FanConfig, fan_config | 0x20)?;
        }
        let val: u8 = percent * 64 / 100;
        // The FanSetting Register drives the fan driver when the Fan Control Look-Up
        // Table is not used.
        // Any data written to the FanSetting register is applied immediately to the
        // fan driver (PWM or DAC).
        self.write_reg(Register::FanSetting, val)?;
        Ok(self)
    }

    /// set_fan_lut set the FAN according to a Look-up Table with hysteresis.
    /// The 'lut' must be 8 or less levels and be ordered with lower temperature value first.
    /// Each level temperature must be between 0 and 127 degrees Celsius.
    /// Each level power must be between 0 and 100%.
    pub fn set_fan_lut(&mut self, lut: &[Level], hysteresis: u8) -> Result<&mut Self, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("set_fan_lut");
        if lut.len() > 8 || lut.is_empty() {
            return Err(Error::InvalidSize);
        }
        if hysteresis > 31 {
            return Err(Error::InvalidValue);
        }
        let fan_config: u8 = self.read_reg(Register::FanConfig)?;
        // FanConfig[5] PROG == 0 :
        // the FanSetting Register and Fan Control Look-Up Table Registers are read-only
        // and the Fan Control Look-Up Table Registers will be used.
        if fan_config & 0x20 == 0x00 {
            // Set FanConfig[5] PROG : the FanSetting Register and Fan Control Look-Up
            // Table Registers can be written and the Fan Control Look-Up Table Registers
            // will not be used.
            self.write_reg(Register::FanConfig, fan_config | 0x20)?;
        }
        let mut last_temp: u8 = 0;
        for (index, level) in (0_u8..).zip(lut.iter()) {
            if level.temp > 127 {
                return Err(Error::InvalidValue);
            }
            if level.percent > 100 {
                return Err(Error::InvalidValue);
            }
            if level.temp <= last_temp {
                return Err(Error::InvalidSorting);
            }
            last_temp = level.temp;
            self.write_reg((Register::FanControlLUTT1 as u8) + 2 * index, level.temp)?;
            let power: u8 = level.percent * 64 / 100;
            self.write_reg((Register::FanControlLUTS1 as u8) + 2 * index, power)?;
        }
        // FanControlLUTHysteresis determines the amount of hysteresis applied to the temperature
        // inputs of the fan control Fan Control Look-Up Table.
        self.write_reg(Register::FanControlLUTHysteresis, hysteresis)?;
        // Clear FanConfig[5] PROG : the FanSetting Register and Fan Control Look-Up Table
        // Registers are read-only and the Fan Control Look-Up Table Registers will be used.
        self.write_reg(Register::FanConfig, fan_config | 0x20)?;
        Ok(self)
    }

    /// get_fan_rpm gives the Fan speed in RPM.
    pub fn get_fan_rpm(&mut self) -> Result<u16, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("get_fan_rpm");
        let msb = self.read_reg(Register::TachMSB)?;
        let lsb = self.read_reg(Register::TachLSB)?;
        let raw: u16 = ((msb as u16) << 8) + (lsb as u16);
        if raw == 0xFFFF {
            return Ok(0);
        }
        Ok((5_400_000u32 / (raw as u32)) as u16)
    }

    /// read_reg read a register value.
    fn read_reg<R: Into<u8>>(&mut self, reg: R) -> Result<u8, Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("read_reg");
        let mut buf = [0x00];
        let reg = reg.into();
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .map_err(Error::I2c)?;
        #[cfg(feature = "defmt")]
        debug!("R @0x{:x}={:x}", reg, buf[0]);
        Ok(buf[0])
    }

    /// write_reg blindly write a single register with a fixed value.
    fn write_reg<R: Into<u8>>(&mut self, reg: R, value: u8) -> Result<(), Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("write_reg");
        let reg = reg.into();
        #[cfg(feature = "defmt")]
        debug!("W @0x{:x}={:x}", reg, value);
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// update_reg first read the register value, apply a set mask, then a clear mask, and write the new value
    /// only if different from the initial value.
    fn update_reg<R: Into<u8> + Clone>(
        &mut self,
        reg: R,
        mask_set: u8,
        mask_clear: u8,
    ) -> Result<(), Error<E>> {
        #[cfg(feature = "defmt")]
        trace!("update_reg");
        let current = self.read_reg(reg.clone())?;
        let updated = current | mask_set & !mask_clear;
        if current != updated {
            self.write_reg(reg, updated)?;
        }
        Ok(())
    }

    /// Destroys this driver and releases the I2C bus `I`.
    pub fn destroy(self) -> Self {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::{DEFAULT_ADDRESS, EMC2101};
    use embedded_hal_mock::i2c::Mock as I2cMock;
    use embedded_hal_mock::i2c::Transaction;

    /// Test creating new EMC2101 sensors.
    ///
    /// Test that we can create multiple EMC2101 devices. We test this because it's one of the
    /// measures of success for this driver.
    #[test]
    fn emc2101_new() {
        let expectations = vec![
            Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![super::Register::ProductID as u8],
                vec![super::ProductID::EMC2101 as u8],
            ),
            Transaction::write(
                DEFAULT_ADDRESS,
                vec![super::Register::AlertMask as u8, 0xFF],
            ),
        ];
        // In the real app we'd used shared-bus to share the i2c bus between the two drivers, but
        // I think this is fine for a test.
        let mock_i2c_1 = I2cMock::new(&expectations);
        let mock_i2c_2 = I2cMock::new(&expectations);

        let _emc2101_1 = EMC2101::new(mock_i2c_1).unwrap();
        let _emc2101_2 = EMC2101::new(mock_i2c_2).unwrap();
    }
}
