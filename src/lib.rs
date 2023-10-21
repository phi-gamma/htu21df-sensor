#![no_std]

/*!
 * HTU21D(F) temperature / humidity sensor by TE MEAS as found in
 * components like the GY-21.
 *
 * Datasheet: https://cdn-shop.adafruit.com/datasheets/1899_HTU21D.pdf
 */

mod constants {
    /*
     * Below values are from the datasheet.
     */
    /** slave address of the HTU21D sensor */
    const I2C_ADDR        : u8 = 0x40;

    /** HTU21D no-hold temperature register */
    const I2C_TEMP        : u8 = 0xf3;

    /** HTU21D no-hold humidity register */
    const I2C_HUMI        : u8 = 0xf5;

    /** power cycle the sensor */
    const SOFT_RESET      : u8 = 0xfe;

    /** wait before reading temperature */
    const TEMP_DELAY_MS   : u8 = 50;

    /** wait before reading humidity */
    const HUMI_DELAY_MS   : u8 = 16;

    /** wait after resetting the sensor */
    const RESET_DELAY_MS  : u8 = 15;

    /** wait for sensor to become ready */
    const POWERUP_DELAY_MS: u8 = 15;
}

#[derive(Debug, Default)]
pub struct Sensor<I2C> {
    i2c: I2C,
}

impl<I2C> Sensor<I2C> {
    pub fn create(i2c: I2C) -> Self {
        Self { i2c }
    }
}
