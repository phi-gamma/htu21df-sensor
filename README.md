# HTU21D(F) sensor support

Crate for reading temperature and humidity values from a HTU21D(F)
sensor over I²C.

The implementation is based on the datasheet provided by MEAS [^0]
and was tested with a GY-21 sensor board. The API is heavily inspired
by the [``shtcx`` crate](https://docs.rs/shtcx/latest/shtcx/).

[^0] https://cdn-shop.adafruit.com/datasheets/1899_HTU21D.pdf

