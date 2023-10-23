#![allow(unused)]
#![deny(unsafe_code)]
#![cfg_attr(not(test), no_std)]

/*!
 * HTU21D(F) temperature / humidity sensor by TE MEAS as found in
 * components like the GY-21.
 *
 * Datasheet: https://cdn-shop.adafruit.com/datasheets/1899_HTU21D.pdf
 */

use embedded_hal::blocking::{delay::DelayUs,
                             i2c::{Read, Write}};

mod constants
{
    /*
     * Below values are from the datasheet.
     */
    /** slave address of the HTU21D sensor */
    pub const I2C_ADDR: u8 = 0x40;

    /** HTU21D no-hold temperature register */
    pub const I2C_TEMP: u8 = 0xf3;

    /** HTU21D no-hold humidity register */
    pub const I2C_HUMI: u8 = 0xf5;

    /** power cycle the sensor */
    pub static SOFT_RESET: [u8; 1] = [0xfe];

    /** wait before reading temperature */
    pub const TEMP_DELAY_MS: u8 = 50;

    /** wait before reading humidity */
    pub const HUMI_DELAY_MS: u8 = 16;

    /**
     * Wait after resetting the sensor (power cycle and reinitialization).
     * It is guaranteed that the sensor reboots in less than 15 ms; cf. p.
     * 12 of HTU21 datasheet.
     */
    pub const RESET_DELAY_MS: u16 = 15;

    /** wait for sensor to become ready */
    pub const POWERUP_DELAY_MS: u8 = 15;
}

#[derive(Debug)]
pub struct Sensor<I2C>
{
    i2c: I2C,
}

#[derive(Debug)]
enum Command
{
    Reset,
}

impl Command
{
    fn as_bytes(&self) -> &'static [u8]
    {
        match self {
            Command::Reset => &constants::SOFT_RESET,
        }
    }
}

impl<I2C, E> Sensor<I2C>
where I2C: Read<Error = E> + Write<Error = E>
{
    pub fn create(i2c: I2C) -> Self { Self { i2c } }

    fn send_command(&mut self, cmd: Command) -> Result<(), E>
    {
        self.i2c.write(constants::I2C_ADDR, cmd.as_bytes())
    }

    pub fn reset(&mut self, delay: &mut impl DelayUs<u16>) -> Result<(), E>
    {
        self.send_command(Command::Reset)?;
        delay.delay_us(constants::RESET_DELAY_MS * 1000);
        Ok(())
    }
}

/** 16 bits of temperature; cf. p. 15 of the datasheet. */
#[inline(always)]
fn calc_temperature(lo: u8, hi: u8) -> f32
{
    let sigout = (hi as u16) << 8u16 | (lo as u16 & 0xfc_u16);

    /* Temp = -46.85 + 175.72 (S_Temp/2^16) */
    0.002681274_f32 * (sigout as f32) - 46.85_f32
}

/** 12 bits of humidity; cf. p. 15 of the datasheet. */
#[inline(always)]
fn calc_relative_humidity(lo: u8, hi: u8) -> f32
{
    let sigout = (hi as u16) << 8u16 | (lo as u16 & 0xf0_u16);

    /* RH = -6 + 125 (S_RH/2^16) */
    0.001907349_f32 * (sigout as f32) - 6.0_f32
}

