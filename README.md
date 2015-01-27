# GY-85

Arduino implementation for GY-85 (ADXL345 accelerometer, ITG3200 gyroscope and HMC5883L magnetometer)

## Theory

The GY-85 contains three microcontroller, measuring acceleration, orientation and Earth's magnetic field. Values can be gathered using the I2C protocol.

### Accelerometer (ADXL345)

> An accelerometer is a device that measures proper acceleration ("g-force"). Proper acceleration is not the same as coordinate acceleration (rate of change of velocity). For example, an accelerometer at rest on the surface of the Earth will measure an acceleration g= 9.81 m/s2 straight upwards. By contrast, accelerometers in free fall orbiting and accelerating due to the gravity of Earth will measure zero.
*[Accelerometer](http://en.wikipedia.org/wiki/Accelerometer)* on Wikipedia, the free encyclopedia.

The accelerometer used on the GY-85 is the ADXL345 from Analog Devices. It measures acceleration for all three axis (x, y, z) and has a resolution up to 13 bit (detects changes less than 1.0°). The ADXL345 also has some nice extra features like tap and double tab detection, that can be used to trigger an interrupt on the Arduino.

The chip usually returns digitalized sensor values with a resolution of 10 bit. To be able to further work with this data, the values have to be converted to a common unit like G.

    Value in G = Measurement Value * (G-range/(2^Resolution))

The G-range and Resolution used in this formula depends of the configuration of the chip. The ADXL345 supports the ranges ±2g/±4g/±8g/±16g. As Resolution, 10 or 13 bit can be set. Measurement Value is simple the raw value, read from the chip for one axis.
If the ADXL345 is used with the default settings, a resolution of 10 bit and a range of ±2g(=a range of 4g), the following formula can be used:

    Value in G = Measurement Value * (8/(2^10)) = Measurement Value * (8/1024) = Measurement Value * 0.0039

This calculation must be done for each axis separately. The resulting values are expected to be within ±1g.

    xg = valX * 0.0039;
    yg = valY * 0.0039;
    zg = valZ * 0.0039;

#### Links

- [Produkt page: ADXL345](http://www.analog.com/en/mems-sensors/mems-inertial-sensors/adxl345/products/product.html)
- [Datasheet: ADXL345 accelerometer, Ref D](http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf)

## Wiring

- **[GY-85]** -> **[Arduino]**
- VCC_IN -> 3.3V
- GND -> GND
- SCL -> A5
- SDA -> A4

## Links

### Datasheets

- [ITG3200 gyroscope, Rev. 1.4](https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf)
- [HMC5883L magnetometer](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf)

### Guides

- [A Guide To using IMU (Accelerometer and Gyroscope Devices) in Embedded Applications.](http://www.starlino.com/imu_guide.html)
- [Sparkfun: ADXL345 Quickstart Guide](https://www.sparkfun.com/tutorials/240)
