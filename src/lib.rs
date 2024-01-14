/* SPDX-License-Identifier: MIT */

#![allow(unused)]
#![deny(unsafe_code)]
#![deny(missing_docs)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]

/*!
 * # About
 *
 * This is a Rust driver for the *HTU21D(F)* temperature / humidity sensor
 * by TE MEAS found in widely available sensor components. It was tested on
 * GY-21 sensor but should work with different HTU21 based sensors too.
 *
 * The implementation is based on [the official datasheet by TE
 * MEAS](https://cdn-shop.adafruit.com/datasheets/1899_HTU21D.pdf).
 *
 * # Example performing measurements with the HTU21D on the ESP32
 *
 * In this example only the HTU21 (GY-21) is accessed over I²C but since
 * in a real-world application that is unlikely to be the only slave
 * device on the bus, the I²C struct is wrapped with the
 * [``shared-bus``](https://docs.rs/shared-bus) crate.
 *
 * ```ignore
 *  use esp_println::{print, println};
 *  use hal::{clock::ClockControl, gpio::IO, i2c::I2C, peripherals::Peripherals,
 *            prelude::*, Delay};
 *  use htu21df_sensor::Sensor;
 *
 *  // set up clock and delay handle to ensure the proper duration between
 *  // measurement initiation and reading the result
 *  let clocks = ClockControl::max(system.clock_control).freeze();
 *  let mut delay = Delay::new(&clocks);
 *
 *  // initialize I²C with the default GPIO pins on ESP32
 *  let i2c = I2C::new(
 *      peripherals.I2C0,
 *      io.pins.gpio21,
 *      io.pins.gpio22,
 *      100u32.kHz(),
 *      /* TODO: this will go away soon */
 *      &mut system.peripheral_clock_control,
 *      &clocks,
 *  );
 *
 *  // wait for the sensor to become live; the 15 are suggested in the datasheet
 *  delay.delay_ms(15);
 *
 *  // no need to own the I²C bus
 *  let bus = shared_bus::BusManagerSimple::new(i2c);
 *
 *  // finally, initialize the sensor
 *  println!("initializing HTU21D ...");
 *  let mut htu = Sensor::new(bus.acquire_i2c(), Some(&mut delay)).expect("sensor init");
 *
 *  // the sensor is now ready to use
 *  print!("measuring ...");
 *  let humidity: f32 = htu.measure_humidity(&mut delay).expect("humidity").value();
 *  let temperature: f32 = htu.measure_temperature(&mut delay).expect("temperature").value();
 *
 *  println!(" done. temperature: {} °C, rel. humidity: {} %", temperature, humidity);
 * ```
 *
 * # Implementation notes
 *
 * As of version 0.1 only blocking APIs are provided.
 */

use embedded_hal::blocking::{delay::DelayMs,
                             i2c::{Read, Write}};

#[cfg(feature = "std")] mod std;

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
    pub const TEMPERATURE_DELAY_MS: u16 = 50;

    /** wait before reading humidity */
    pub const HUMIDITY_DELAY_MS: u16 = 16;

    /**
     * Wait after resetting the sensor (power cycle and reinitialization).
     * It is guaranteed that the sensor reboots in less than 15 ms; cf. p.
     * 12 of HTU21 datasheet.
     */
    pub const RESET_DELAY_MS: u16 = 15;

    /** wait for sensor to become ready */
    pub const POWERUP_DELAY_MS: u16 = 15;

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

/** Two bytes read from I²C bus representing a measurement. Values are
 * provided in big endian order. */
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RawMeasurement
{
    lo: u8,
    hi: u8,
}

/** Two constants are required to perform a measurement: the command or register
 * address ``C`` and the delay ``D`` in ms from initiating the measurement to
 * reading the result. Both values are different for temperature and relative
 * humidity on the HTU21. */
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Measurement<Output, const D: u16, const C: u8>(Output);

/** A type defining the measurement for relative humidity. */
pub type Humidity = Measurement<
    f32,
    { constants::HUMIDITY_DELAY_MS },
    { constants::I2C_COMMAND_HUMIDITY },
>;

/** A type defining the measurement for temperature. */
pub type Temperature = Measurement<
    f32,
    { constants::TEMPERATURE_DELAY_MS },
    { constants::I2C_COMMAND_TEMPERATURE },
>;

impl<Output: Copy, const D: u16, const C: u8> Measurement<Output, D, C>
{
    const fn i2n_command(&self) -> u8 { C }

    const fn delay_ms(&self) -> u16 { D }

    /** Access the output value of a measurement. */
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

/** Error conditions returned from struct ``Sensor`` member functions. */
#[derive(Copy, Clone, Debug)]
pub enum Error<I2cError>
{
    /** ``I2c`` wraps I²C errors. */
    I2c(I2cError),

    /** ``Crc`` indicates a corrupt reading. */
    Crc,
}

impl<I2C, E> Sensor<I2C>
where I2C: Read<Error = E> + Write<Error = E>
{
    /**
     * Create a new struct ``Sensor`` for the given I²C interface with the
     * HTU21D listening on the default bus address. If ``delay`` is passed,
     * an initial reset is performed as recommended in the datasheet.
     *
     * Cf. p. 10 of the datasheet.
     */
    pub fn new(
        i2c: I2C,
        delay: Option<&mut impl DelayMs<u16>>,
    ) -> Result<Self, Error<E>>
    {
        let mut htu = Self { i2c, addr: constants::I2C_ADDR };

        if let Some(delay) = delay {
            delay.delay_ms(constants::POWERUP_DELAY_MS);
            htu.reset(delay)?;
        }

        Ok(htu)
    }

    /**
     * Release the owned I²C device by consuming ``self``.
     *
     * This can be useful for sharing the same bus between multiple
     * drivers manually without resorting to a manager like the
     * [``shared-bus``](https://docs.rs/shared-bus) crate.
     */
    pub fn destroy(self) -> I2C { self.i2c }

    /**
     * Create a new struct ``Sensor`` for the given I²C interface with
     * a custom bus address. If ``delay`` is passed, an initial reset is
     * performed as recommended in the datasheet.
     *
     * Use this e. g. if you’ve resolved a bus address conflict with a
     * multiplexer and the HTU21D is listening on a non-default address.
     */
    pub fn with_address(
        i2c: I2C,
        delay: Option<&mut impl DelayMs<u16>>,
        addr: u8,
    ) -> Result<Self, Error<E>>
    {
        let mut htu = Self { i2c, addr };

        if let Some(delay) = delay {
            delay.delay_ms(constants::POWERUP_DELAY_MS);
            htu.reset(delay)?;
        }

        Ok(htu)
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

    /**
     * Initiate a measurement of a particular type.
     *
     * This will return immediately after submitting the measurement command
     * over I²C without waiting for the sensor to become ready for reading
     * the result. It can be used paired with ``measurement_result()`` to
     * implement non-blocking APIs. */
    pub fn start_measurement<const C: u8>(&mut self) -> Result<(), Error<E>>
    {
        self.send_command(C)
    }

    /**
     * Read a value for which a measurement has been initiated.
     *
     * This will return immediately after reading the value from the
     * I²C without waiting for the sensor to become ready first.
     * It can be used together with ``start_measurement()`` to implement
     * non-blocking APIs.
     *
     * Note that this API provides no safeguards against conflating
     * measurement types or insufficient delay since initiating the
     * measurement. Ensuring read readiness of the sensor is up to
     * caller.
     */
    pub fn measurement_result<M>(&mut self) -> Result<M, Error<E>>
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

    /** Perform one blocking temperature measurement for the given measurement
     * type, waiting the appropriate amount of time before reading the value. */
    pub fn measure_temperature(
        &mut self,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<Temperature, Error<E>>
    {
        self.measure::<f32, { constants::TEMPERATURE_DELAY_MS }, {constants::I2C_COMMAND_TEMPERATURE}>(delay)
    }

    /** Perform one blocking humidity measurement for the given measurement
     * type, waiting the appropriate amount of time before reading the value. */
    pub fn measure_humidity(
        &mut self,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<Humidity, Error<E>>
    {
        self.measure::<f32, { constants::HUMIDITY_DELAY_MS }, {constants::I2C_COMMAND_HUMIDITY}>(delay)
    }

    /**
     * Send a soft reset command to the sensor and wait for the appropriate time
     * for the device to initialize itself.
     *
     * Cf. p. 12 of the datasheet.
     */
    pub fn reset(
        &mut self,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<(), Error<E>>
    {
        self.send_command(constants::SOFT_RESET)?;
        delay.delay_ms(constants::RESET_DELAY_MS);
        Ok(())
    }

    /**
     * Read a sequence of bytes from a HTU21D sensor registers with the given
     * delay between write and read operations.
     */
    #[inline]
    fn measure<Output, const D: u16, const C: u8>(
        &mut self,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<Measurement<Output, D, C>, Error<E>>
    where
        Measurement<Output, D, C>: From<RawMeasurement>,
    {
        self.start_measurement::<C>()?;
        delay.delay_ms(D);
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
    #[test]
    fn temp()
    {
        use RawMeasurement as RM;
        /* 0x683a ⇒ 24.7 °C */
        assert_eq!(
            Temperature::from(RM { lo: 0x3a, hi: 0x68 }),
            Measurement(24.686394)
        );
    }
}
