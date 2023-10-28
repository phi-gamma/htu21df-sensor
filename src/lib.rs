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
    pub const I2C_COMMAND_TEMPERATURE: u8 = 0xf3;

    /** HTU21D no-hold humidity register */
    pub const I2C_COMMAND_HUMIDITY: u8 = 0xf5;

    /** power cycle the sensor */
    pub const SOFT_RESET: u8 = 0xfe;

    /** wait before reading temperature */
    pub const TEMP_DELAY_MS: u16 = 50;

    /** wait before reading humidity */
    pub const HUMI_DELAY_MS: u16 = 16;

    /**
     * Wait after resetting the sensor (power cycle and reinitialization).
     * It is guaranteed that the sensor reboots in less than 15 ms; cf. p.
     * 12 of HTU21 datasheet.
     */
    pub const RESET_DELAY_MS: u16 = 15;

    /** wait for sensor to become ready */
    pub const POWERUP_DELAY_MS: u8 = 15;

    /** The checksum uses a 9-bit polynomial of 2^8 + 2^5 + 2^4 + 1. */
    pub const CRC8_POLY: u32 = 0b100110001;
}

/**
 * Struct representing an interface to the HTU21D(F) on the I²C bus.
 */
#[derive(Debug)]
pub struct Sensor<I2C>
{
    i2c:  I2C,
    addr: u8,
}

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
enum Command
{
    Reset = constants::SOFT_RESET,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RawMeasurement
{
    lo: u8,
    hi: u8,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Measurement<Output, const D: u16, const C: u8>(Output);

pub type Humidity = Measurement<
    f32,
    { constants::HUMI_DELAY_MS },
    { constants::I2C_COMMAND_HUMIDITY },
>;
pub type Temperature = Measurement<
    f32,
    { constants::TEMP_DELAY_MS },
    { constants::I2C_COMMAND_TEMPERATURE },
>;

impl<Output: Copy, const D: u16, const C: u8> Measurement<Output, D, C>
{
    const fn i2n_command(&self) -> u8 { C }

    const fn delay_ms(&self) -> u16 { D }

    pub const fn value(&self) -> Output { self.0 }
}

impl From<RawMeasurement> for u16
{
    fn from(RawMeasurement { lo, hi }: RawMeasurement) -> Self
    {
        (hi as u16) << 8 | (lo as u16 & 0xfc_u16)
    }
}

impl From<RawMeasurement> for Temperature
{
    /** 16 bits of temperature; cf. p. 15 of the datasheet. */
    fn from(raw: RawMeasurement) -> Self
    {
        let sigout: u16 = raw.into();

        /* Temp = -46.85 + 175.72 (S_Temp/2^16) */
        Self(0.002681274_f32 * (sigout as f32) - 46.85_f32)
    }
}

impl From<RawMeasurement> for Humidity
{
    /** 12 bits of humidity; cf. p. 15 of the datasheet. */
    fn from(raw: RawMeasurement) -> Self
    {
        let sigout: u16 = raw.into();

        /* RH = -6 + 125 (S_RH/2^16) */
        Self(0.001907349_f32 * (sigout as f32) - 6.0_f32)
    }
}

impl From<Command> for u8
{
    fn from(cmd: Command) -> Self
    {
        match cmd {
            Command::Reset => constants::SOFT_RESET,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Error<I2cError>
{
    I2c(I2cError),
    Crc,
}

impl<I2C, E> Sensor<I2C>
where I2C: Read<Error = E> + Write<Error = E>
{
    /**
     * Create a new struct ``Sensor`` for the given I²C interface with the
     * HTU21D listening on the default bus address.
     *
     * Cf. p. 10 of the datasheet.
     */
    pub fn create(i2c: I2C) -> Result<Self, Error<E>>
    {
        Ok(Self { i2c, addr: constants::I2C_ADDR })
    }

    /**
     * Create a new struct ``Sensor`` for the given I²C interface with
     * a custom bus address.
     *
     * Use this e. g. if you’ve resolved a bus address conflict with a
     * multiplexer and the HTU21D is listening on a non-default address.
     */
    pub fn with_address(i2c: I2C, addr: u8) -> Result<Self, Error<E>>
    {
        Ok(Self { i2c, addr })
    }

    /**
     * Issue commands to the HTU21D over I²C. ``cmd`` must be a valid command
     * as describe in the datasheet.
     */
    fn send_command<C: Into<u8>>(&mut self, cmd: C) -> Result<(), Error<E>>
    {
        let cmd: u8 = cmd.into();
        self.i2c.write(self.addr, [cmd].as_slice()).map_err(Error::I2c)
    }

    fn start_measurement<const C: u8>(&mut self) -> Result<(), Error<E>>
    {
        self.send_command(C)
    }

    fn measurement_result<M>(&mut self) -> Result<M, Error<E>>
    where M: From<RawMeasurement>
    {
        let raw = self.raw_measurement_result()?;
        Ok(raw.into())
    }

    fn raw_measurement_result(&mut self) -> Result<RawMeasurement, Error<E>>
    {
        self.read_value_checked().map(|(hi, lo)| RawMeasurement { hi, lo })
    }

    /**
     * Read three bytes from I²C where the third byte is considered the CRC.
     * Returns an error if this byte does not match the CRC computed from the
     * first two bytes.
     *
     * When the CRC check succeeds, a pair holding the high and low byte is
     * returned.
     */
    fn read_value_checked(&mut self) -> Result<(u8, u8), Error<E>>
    {
        let mut buf = [0; 3];
        self.i2c.read(self.addr, &mut buf).map_err(Error::I2c)?;

        if calc_crc(buf[0], buf[1]) != buf[2] {
            Err(Error::Crc)
        } else {
            Ok((buf[0], buf[1]))
        }
    }

    pub fn measure_temperature(
        &mut self,
        delay: &mut impl DelayUs<u16>,
    ) -> Result<Temperature, Error<E>>
    {
        self.measure::<f32, { constants::TEMP_DELAY_MS }, {constants::I2C_COMMAND_TEMPERATURE}>(delay)
    }

    pub fn measure_humidity(
        &mut self,
        delay: &mut impl DelayUs<u16>,
    ) -> Result<Humidity, Error<E>>
    {
        self.measure::<f32, { constants::HUMI_DELAY_MS }, {constants::I2C_COMMAND_HUMIDITY}>(delay)
    }

    /**
     * Send a soft reset command to the sensor and wait for the appropriate time
     * for the device to initialize itself.
     *
     * Cf. p. 12 of the datasheet.
     */
    pub fn reset(
        &mut self,
        delay: &mut impl DelayUs<u16>,
    ) -> Result<(), Error<E>>
    {
        self.send_command(Command::Reset)?;
        delay.delay_us(constants::RESET_DELAY_MS * 1000);
        Ok(())
    }

    /**
     * Read a sequence of bytes from a HTU21D sensor registers with the given
     * delay between write and read operations.
     */
    #[inline]
    fn measure<Output, const D: u16, const C: u8>(
        &mut self,
        delay: &mut impl DelayUs<u16>,
    ) -> Result<Measurement<Output, D, C>, Error<E>>
    where
        Measurement<Output, D, C>: From<RawMeasurement>,
    {
        self.start_measurement::<C>()?;
        delay.delay_us(D * 1000);
        self.measurement_result::<Measurement<Output, D, C>>()
    }
}

/**
 * Compute 8-bit CRC using a 9 bit polynomial.
 *
 * Datasheet p. 14. (The description reads suspiciously similar to
 * that on the Wikipedia page:
 * https://en.wikipedia.org/wiki/Cyclic_redundancy_check#Computation )
 */
#[inline]
const fn calc_crc(hi: u8, lo: u8) -> u8
{
    let mut crc: u32 = (hi as u32) << 16 | (lo as u32) << 8;
    let mut div: u32 = constants::CRC8_POLY << 15;

    let mut b = 24usize;
    while b != 8 {
        b -= 1;
        if ((crc & (1 << b)) != 0) {
            crc ^= div;
        }
        div >>= 1;
    }

    /* The eight bits left on the right are the CRC. */
    (crc & 0xff) as u8
}

#[cfg(test)]
mod tests
{
    use super::*;

    /** Page 15 in datasheet. */
    #[test]
    fn crc()
    {
        assert_eq!(calc_crc(0b00000000u8, 0b11011100u8), 0b01111001u8);
        assert_eq!(calc_crc(0b01101000u8, 0b00111010u8), 0b01111100u8);
        assert_eq!(calc_crc(0b01001110u8, 0b10000101u8), 0b01101011u8);
    }

    /** Page 15 in datasheet. */
    #[test]
    fn humidity()
    {
        use RawMeasurement as RM;

        /* 0x7c80 ⇒ 54.8 %RH */
        assert_eq!(
            Humidity::from(RM { lo: 0x80, hi: 0x7c }),
            Measurement(54.791027)
        );
        /* 0x4e85 ⇒ 32.3 %RH */
        assert_eq!(
            Humidity::from(RM { lo: 0x85, hi: 0x4e }),
            Measurement(32.337715)
        );
    }

    /** Page 15 in datasheet. */
    fn temp()
    {
        use RawMeasurement as RM;
        /* 0x683a ⇒ 24.7 °C */
        assert_eq!(
            Temperature::from(RM { lo: 0x3a, hi: 0x68 }),
            Measurement(24.7)
        );
    }
}
