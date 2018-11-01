/*
Author  : alrevuelta
Version : 1.0.0
File    : SEN10724.h
Brief   : This file contains an Arduino library to be used with the SEN-10724
9 degrees of freedom sensor stick sold at Sparkfun. This hardware contains three
sensors (ADXL345 accelerometer, ITG-3200 gyroscope and HMC5883L magnetometer).
Different functions are provided that allow to read data from the sensors, change
its configuration parameters and calibrate them.

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SEN10724_h
#define SEN10724_h
#include "Arduino.h"

// Accelerometer register address
#define ADXL345_ADDRESS               (0x53)
#define ADXL345_REG_DEVID             (0x00)
#define ADXL345_REG_THRESH_TAP        (0x1D)
#define ADXL345_REG_OFSX              (0x1E)
#define ADXL345_REG_OFSY              (0x1F)
#define ADXL345_REG_OFSZ              (0x20)
#define ADXL345_REG_DUR               (0x21)
#define ADXL345_REG_LATENT            (0x22)
#define ADXL345_REG_WINDOW            (0x23)
#define ADXL345_REG_THRESH_ACT        (0x24)
#define ADXL345_REG_THRESH_INACT      (0x25)
#define ADXL345_REG_TIME_INACT        (0x26)
#define ADXL345_REG_ACT_INACT_CTL     (0x27)
#define ADXL345_REG_THRESH_FF         (0x28)
#define ADXL345_REG_TIME_FF           (0x29)
#define ADXL345_REG_TAP_AXES          (0x2A)
#define ADXL345_REG_ACT_TAP_STATUS    (0x2B)
#define ADXL345_REG_BW_RATE           (0x2C)
#define ADXL345_REG_POWER_CTL         (0x2D)
#define ADXL345_REG_INT_ENABLE        (0x2E)
#define ADXL345_REG_INT_MAP           (0x2F)
#define ADXL345_REG_INT_SOURCE        (0x30)
#define ADXL345_REG_DATA_FORMAT       (0x31)
#define ADXL345_REG_DATAX0            (0x32)
#define ADXL345_REG_DATAX1            (0x33)
#define ADXL345_REG_DATAY0            (0x34)
#define ADXL345_REG_DATAY1            (0x35)
#define ADXL345_REG_DATAZ0            (0x36)
#define ADXL345_REG_DATAZ1            (0x37)
#define ADXL345_REG_FIFO_CTL          (0x38)
#define ADXL345_REG_FIFO_STATUS       (0x39)

// Gyroscope register address
#define ITG3200_ADDRESS               (0x68)
#define ITG3200_REG_WHO_AM_I          (0x00)
#define ITG3200_REG_SMPLRT_DIV        (0x15)
#define ITG3200_REG_DLPF_FS           (0x16)
#define ITG3200_REG_INT_CFG           (0x17)
#define ITG3200_REG_INT_STATUS        (0x1A) 
#define ITG3200_REG_TEMP_OUT_H        (0x1B)
#define ITG3200_REG_TEMP_OUT_L        (0x1C)
#define ITG3200_REG_GYRO_XOUT_H       (0x1D)
#define ITG3200_REG_GYRO_XOUT_L       (0x1E)
#define ITG3200_REG_GYRO_YOUT_H       (0x1F)
#define ITG3200_REG_GYRO_YOUT_L       (0x20)
#define ITG3200_REG_GYRO_ZOUT_H       (0x21)
#define ITG3200_REG_GYRO_ZOUT_L       (0x22)
#define ITG3200_REG_PWR_MGM           (0x3E)

// Magnetometer register address
#define HMC5883L_ADDRESS              (0x1E)
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

// Accelerometer
typedef enum
{
  ADXL345_DATARATE_3200HZ    = 0b1111,
  ADXL345_DATARATE_1600HZ    = 0b1110,
  ADXL345_DATARATE_800HZ     = 0b1101,
  ADXL345_DATARATE_400HZ     = 0b1100,
  ADXL345_DATARATE_200HZ     = 0b1011,
  ADXL345_DATARATE_100HZ     = 0b1010,
  ADXL345_DATARATE_50HZ      = 0b1001,
  ADXL345_DATARATE_25HZ      = 0b1000,
  ADXL345_DATARATE_12_5HZ    = 0b0111,
  ADXL345_DATARATE_6_25HZ    = 0b0110,
  ADXL345_DATARATE_3_13HZ    = 0b0101,
  ADXL345_DATARATE_1_56HZ    = 0b0100,
  ADXL345_DATARATE_0_78HZ    = 0b0011,
  ADXL345_DATARATE_0_39HZ    = 0b0010,
  ADXL345_DATARATE_0_20HZ    = 0b0001,
  ADXL345_DATARATE_0_10HZ    = 0b0000
} adxl345_dataRate_t;

typedef enum
{
  ADXL345_RANGE_16G          = 0b11,
  ADXL345_RANGE_8G           = 0b10,
  ADXL345_RANGE_4G           = 0b01,
  ADXL345_RANGE_2G           = 0b00
} adxl345_range_t;

// Gyroscope
typedef enum
{
  ITG3200_FS_1       = 0b00,
  ITG3200_FS_2       = 0b01,
  ITG3200_FS_3       = 0b10,
  ITG3200_FS_2000    = 0b11
} itg3200_fs_t;

typedef enum
{
  ITG3200_LOWPASS_256          = 0b000, // 8 kHz F internal
  ITG3200_LOWPASS_188          = 0b001, // 1 kHz F internal
  ITG3200_LOWPASS_98           = 0b010, // 1 kHz F internal
  ITG3200_LOWPASS_42           = 0b011, // 1 kHz F internal
  ITG3200_LOWPASS_20           = 0b100, // 1 kHz F internal
  ITG3200_LOWPASS_10           = 0b101, // 1 kHz F internal
  ITG3200_LOWPASS_5            = 0b110, // 1 kHz F internal
  ITG3200_LOWPASS_RES          = 0b111  // Reserved
} itg3200_lowPass_t;

typedef enum
{
  ITG3200_CLK_INTERNAL      = 0b000,
  ITG3200_CLK_PLL_X         = 0b001,
  ITG3200_CLK_PLL_Y         = 0b010,
  ITG3200_CLK_PLL_Z         = 0b011,
  ITG3200_CLK_PLL_32        = 0b100,
  ITG3200_CLK_PLL_19        = 0b101,
  ITG3200_CLK_RES1          = 0b110, // Reserved
  ITG3200_CLK_RES2          = 0b111  // Reserved
} itg3200_clk_t;

// Magnetometer
typedef enum
{
  HMC5883L_SAMPLES_8     = 0b11,
  HMC5883L_SAMPLES_4     = 0b10,
  HMC5883L_SAMPLES_2     = 0b01,
  HMC5883L_SAMPLES_1     = 0b00
} hmc5883l_samples_t;

typedef enum
{
  HMC5883L_DATARATE_75HZ       = 0b110,
  HMC5883L_DATARATE_30HZ       = 0b101,
  HMC5883L_DATARATE_15HZ       = 0b100,
  HMC5883L_DATARATE_7_5HZ      = 0b011,
  HMC5883L_DATARATE_3HZ        = 0b010,
  HMC5883L_DATARATE_1_5HZ      = 0b001,
  HMC5883L_DATARATE_0_75_HZ    = 0b000
} hmc5883l_dataRate_t;

typedef enum
{
  HMC5883L_RANGE_8_1GA     = 0b111,
  HMC5883L_RANGE_5_6GA     = 0b110,
  HMC5883L_RANGE_4_7GA     = 0b101,
  HMC5883L_RANGE_4GA       = 0b100,
  HMC5883L_RANGE_2_5GA     = 0b011,
  HMC5883L_RANGE_1_9GA     = 0b010,
  HMC5883L_RANGE_1_3GA     = 0b001,
  HMC5883L_RANGE_0_88GA    = 0b000
} hmc5883l_range_t;

typedef enum
{
  HMC5883L_IDLE          = 0b10,
  HMC5883L_SINGLE        = 0b01,
  HMC5883L_CONTINOUS     = 0b00
} hmc5883l_mode_t;

class SEN10724
{
public:
  
  // Accelerometer
  bool begin(void);
  void clearAccSettings(void);
  float* readAccRaw(void);
  float* readAccScaled(void);
  float* readAccCalib(void);
  void  setAccRange(adxl345_range_t range);
  adxl345_range_t getAccRange(void);
  void  setAccDataRate(adxl345_dataRate_t dataRate);
  adxl345_dataRate_t getAccDataRate(void);
  void setAccMaxMinValues(float accMaxValues[3], float accMinValues[3]);
  
  // Gyroscope
  float* readGyrRaw(void);
  float* readGyrScaled(void);
  float* readGyrCalib(void);
  float  readGyrTemperature(void);
  void  setGyrFs(itg3200_fs_t fs);
  itg3200_fs_t getGyrFs(void);
  void  setGyrLowPass(itg3200_lowPass_t lpass);
  itg3200_lowPass_t getGyrLowPass(void);
  void  setGyrClk(itg3200_clk_t clk);
  itg3200_clk_t getGyrClk(void);
  void setGyrRateDivider(uint8_t rate);
  uint8_t getGyrRateDivider(void);
  void setGyrOffsets(float gyrOffsets[3]);
  
  // Magnetometer
  float* readMagRaw(void);
  float* readMagScaled(void);
  float* readMagCalib(void);
  void  setMagRange(hmc5883l_range_t range);
  hmc5883l_range_t getMagRange(void);
  void  setMagMeasurementMode(hmc5883l_mode_t mode);
  hmc5883l_mode_t getMagMeasurementMode(void);
  void  setMagDataRate(hmc5883l_dataRate_t dataRate);
  hmc5883l_dataRate_t getMagDataRate(void);
  void  setMagSamples(hmc5883l_samples_t samples);
  hmc5883l_samples_t getMagSamples(void);
  void setMagEllipsoidCalib(float magEllipsoidCenter[3], float magEllipsoidTransform[3][3]);
  
private:
  // Accelerometer
  float accRaw[3];
  float accScaled[3];
  float accCalib[3];
  float accMaxValues[3];
  float accMinValues[3];
  float accOffsets[3];
  float accScales[3];
  adxl345_range_t _range;
  
  // Gyroscope
  float gyrRaw[3];
  float gyrScaled[3];
  float gyrCalib[3];
  float gyrOffsets[3];
  float temperature;
  
  // Magnetometer
  float magRaw[3];
  float magScaled[3];
  float magCalib[3];
  float magEllipsoidCenter[3];
  float magEllipsoidTransform[3][3];
  float mgPerDigit;
  
  // Read and write functions
  void writeRegister8(uint8_t reg, uint8_t value, uint8_t sensor);
  uint8_t readRegister8(uint8_t reg, uint8_t sensor);
  uint8_t fastRegister8(uint8_t reg, uint8_t sensor);
  int16_t readRegister16(uint8_t reg, uint8_t sensor);
  void writeRegisterBit(uint8_t reg, uint8_t pos, bool state, uint8_t sensor);
  bool readRegisterBit(uint8_t reg, uint8_t pos, uint8_t sensor);
};

#endif