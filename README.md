# SEN-10724

This library provides a set of functions to easily interact with the [SparkFun 9 Degrees of Freedom Sensor Stick](https://www.sparkfun.com/products/retired/10724) sold at SparkFun website. This electronic board contains an accelerometer, a gyroscope and a magnetometer. In this library, different high level functions are provided, that allow to read data from all three sensors in an easy way. This library also provides functions to change the configuration parameters of each sensor (see datasheet) and functions to calibrate each sensor individually.

<p align="center">
  <img width="200" height="200" src="https://cdn.sparkfun.com//assets/parts/5/6/0/5/10724-01a.jpg">
</p>


# Introduction
This repository contains a C++ library that can be used with Arduino to interact with the SparkFun 9DOF freedom board, which contains an accelerometer, a magnetometer and a gyroscope. See datasheets for further information:

* [Accelerometer (ADXL345)](https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf)
* [Magnetometer (HMC5883L)](http://cdn.sparkfun.com/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf)
* [Gyroscope (ITG-3200)](https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf)

The board can be connected to the Arduino via I2C using only four connectors. The setup is pretty straight forward, just connect the following pins to Arduino:

* SCL
* SDA
* GND
* VCC

# Usage
A class `SEN10724.cpp` is provided in the `src` folder. Just instantiate an object with:
```cpp
SEN10724 sen10724;
```
Its important to bear in mind that we are dealing with three different hardware components that are in the same board, using I2C protocol to communicate with them, so each one has its own address and its own registers. If you are interested in this, you can check both the datasheets and the `SEN10724.h`file, which has all the constants taken from the documentation of each sensor.

Next step is to initialize each component with different parameters. Depending on the component, we will need to set different ones. The accelerometer needs the sampling frequency and the scale. The gyroscope the sampling frequency, low pass configuration, clock and rate divider. Finally, the magnetometer needs the range, the measurement mode and the datarate. This configuration is done as follows:

For the accelerometer
```cpp
// Accelerometer configuration
sen10724.setAccRange(ADXL345_RANGE_16G);
sen10724.setAccDataRate(ADXL345_DATARATE_400HZ);
```

For the gyroscope
```cpp
// Gyroscope configuration
sen10724.setGyrFs(ITG3200_FS_2000);
sen10724.setGyrLowPass(ITG3200_LOWPASS_42);
sen10724.setGyrClk(ITG3200_CLK_INTERNAL);
sen10724.setGyrRateDivider(9);
```

For the magnetometer
```cpp
// Magnetometer configuration
sen10724.setMagRange(HMC5883L_RANGE_1_3GA);
sen10724.setMagMeasurementMode(HMC5883L_CONTINOUS);
sen10724.setMagDataRate(HMC5883L_DATARATE_15HZ);
sen10724.setMagSamples(HMC5883L_SAMPLES_1);
```

Note that the parameters such as `ITG3200_FS_2000` or `HMC5883L_SAMPLES_1` are defined in `SEN10724.h` file in the enums `itg3200_fs_t` and `hmc5883l_samples_t`. You can find all enums in the header file, so feel free to configure each sensor as you wish. Note that some combinations might not work, so read the datesheet carefully for further information about that.

Once we have set up our sensors, we are ready to start reading data from them. Three different functions are provided:
* `readRaw`: Simply reads the data that is stored in each register (x, y, z) of each sensor. Note that this values are not very useful because the have not been converted to real unit (such as m/s/s or degrees/s).
* `readScaled`: This functions read the data from the sensor but scaled, so the output is the measurements in real units (see datasheets).
* `readCalib`: This functions first read data from the sensor, and then correct them according to a given set of calibration parameters. Note that if no calibration parameters, the output of `readScaled`and `readCalib` will be the same.

For example, if you want to read the scaled values for the accelerometer, gyroscope and magnetomer, you can do something like this. Note that each pointer contains three values, one per axis in the order: x-axis, y-axis and z-axis. So `accscaled[0]` is the measurement from the x axis of the accelerometer in G.
```cpp
float* accscaled = sen10724.readAccScaled();
float* gyrScaled = sen10724.readGyrScaled();
float* magscaled = sen10724.readMagScaled();
```

If you use `...readScaled` or `...readCalib`, the units are in `g` for the accelerometer, `Gauss` for the magnetometer and `deg/s` for the gyroscope.

Note that other three functions are provided `readAccRaw()`, `readGyrRaw()` and `readMagRaw` for reading the raw data from the sensor. This data is not scaled according to the LSB per unit, so this data can really be used, but can be useful for debugging.

```cpp
float* accraw = sen10724.readAccRaw();
float* gyrraw = sen10724.readGyrRaw();
float* magraw = sen10724.readMagRaw();
```

# Calibration
This code also provides a set of functions to perform calibration on the sensors. This means that after measuring the data that each sensor reads in some given conditions, that data is used to perform some corrections. In [4] you can find very good information about how the calibration can be done:
* Accelerometer: We need to know the minimum and maximum vales that the accelerometer reads. In a perfect world, it will measure +1 as maximum and -1 as minimum, but since the sensor is not perfect, this will vary.
* Gyroscope: Since the gyroscope measures angular speed, if it is static, all the measurements should be zero. However, this is not true, we will always have a bit of error.
* Magnetometer: The method for calibration that is used is the ellipsoid fit.

So if you want to calibrate the sensors, you need to first define all this values like follows. Of course, this will vary depending on your sensor, temperature, and so on.
```cpp
float accMaxValues[3] = {1.064, 1.016, 1.044};
float accMinValues[3] = {-1.068, -1.08, -1.236};
float gyrOffsets[3]   = {-0.904, 5.426, 0.626};
float magEllipsoidCenter[3]       = {-178.266, 92.8594, -59.4538};
float magEllipsoidTransform[3][3] = {{0.789315, -0.0151758, -0.0645366},
                                     {-0.0151758, 0.982619, 0.00424189},
                                     {-0.0645366, 0.00424189, 0.975379}};
```

Next step is to set the values:
```cpp
sen10724.setAccMaxMinValues(accMaxValues, accMinValues);
sen10724.setGyrOffsets(gyrOffsets);
sen10724.setMagEllipsoidCalib(magEllipsoidCenter, magEllipsoidTransform);
```

So now, when calling to `readAccCalib()`, `readGyrCalib()` and `readMagCalib()`, the program will take that parameter into account and the output values to that function would be the calibrated measurements.


# References
* [1] https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
* [2] https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
* [3] http://cdn.sparkfun.com/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf
* [4] https://github.com/Razor-AHRS/razor-9dof-ahrs
* [5] https://github.com/jarzebski/Arduino-ADXL345
* [6] https://github.com/jarzebski/Arduino-HMC5883L
