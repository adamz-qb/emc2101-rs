#![no_std]
#![no_main]

use nucleo_l476rg as _; // global logger + panicking-behavior + memory layout

use cortex_m::delay::Delay;

use stm32_hal2::{
    self,
    clocks::Clocks,
    //low_power,
    gpio::{OutputType, Pin, PinMode, Port},
    i2c::{I2c, I2cConfig, I2cSpeed, NoiseFilter},
    pac,
};

use emc2101_driver::{EMC2101, SENSOR_ADDRESS};

#[cortex_m_rt::entry]
fn main() -> ! {
    // Set up ARM Cortex-M peripherals. These are common to many MCUs, including all STM32 ones.
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up peripherals specific to the microcontroller you're using.
    let dp = pac::Peripherals::take().unwrap();

    // This line is required to prevent the debugger from disconnecting on entering WFI.
    // This appears to be a limitation of many STM32 families. Not required in production code,
    // and significantly increases power consumption in low-power modes.
    stm32_hal2::debug_workaround();

    // Create an initial clock configuration that uses the MCU's internal oscillator (HSI),
    // sets the MCU to its maximum system clock speed.
    let clock_cfg = Clocks::default();

    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock_cfg.setup().unwrap();

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());
    delay.delay_ms(100);

    // Configure pins for I2c.
    defmt::info!("config PB8 as I2C1_SCL");
    let mut scl = Pin::new(Port::B, 8, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);

    defmt::info!("config PB9 as I2C1_SDA");
    let mut sda = Pin::new(Port::B, 9, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    let i2c_cfg = I2cConfig {
        speed: I2cSpeed::Fast400K, // Set to Fast mode, at 400Khz.
        // Set a digital noise filter instead of the default analog one.
        noise_filter: NoiseFilter::Digital(5),
        ..Default::default()
    };

    defmt::info!("config I2C1 at 400kHz");
    // customize the config, including setting different preset speeds:
    let i2c1 = I2c::new(dp.I2C1, i2c_cfg, &clock_cfg);

    defmt::info!("config EMC2101 on I2C1");
    let mut emc2101 = EMC2101::new(i2c1, SENSOR_ADDRESS).unwrap();

    emc2101.enable_tach_input().unwrap();

    let int_temp = emc2101.get_temp_internal().unwrap();
    defmt::info!("EMC2101 internal temperature = {=i8}", int_temp);

    let status = emc2101.get_status().unwrap();
    defmt::info!("EMC2101 status = {:?}", status);

    if !status.ext_diode_fault {
        let ext_temp = emc2101.get_temp_external().unwrap();
        defmt::info!("EMC2101 external temperature = {=i8}", ext_temp);
    }

    nucleo_l476rg::exit()
}
