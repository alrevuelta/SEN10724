/*
Author  : alrevuelta
Version : 1.0.0
File    : SEN10724.cpp
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

#include "Arduino.h"
#include "SEN10724.h"
#include <Wire.h>

//Config the 9DOF Sensor Stick
bool SEN10724::begin()
{
  // General variables
  accRaw[0] = 0;
  accRaw[1] = 0;
  accRaw[2] = 0;
  
  gyrRaw[0] = 0;
  gyrRaw[1] = 0;
  gyrRaw[2] = 0;
  
  magRaw[0] = 0;
  magRaw[1] = 0;
  magRaw[2] = 0;
  
  mgPerDigit = 0.92f;
  
  // Init calibration parameters
  accMaxValues[0] = 1;
  accMaxValues[1] = 1;
  accMaxValues[2] = 1;
  
  accMinValues[0] = -1;
  accMinValues[1] = -1;
  accMinValues[2] = -1;
  
  accOffsets[0] = 0;
  accOffsets[1] = 0;
  accOffsets[2] = 0;
  
  accScales[0] = 1;
  accScales[1] = 1;
  accScales[2] = 1;
  
  gyrOffsets[0] = 0;
  gyrOffsets[1] = 0;
  gyrOffsets[2] = 0;
  
  magEllipsoidCenter[0] = 0;
  magEllipsoidCenter[1] = 0;
  magEllipsoidCenter[2] = 0;
  
  // Identity matrix
  magEllipsoidTransform[0][0] = 1;
  magEllipsoidTransform[0][1] = 0;
  magEllipsoidTransform[0][2] = 0;
  magEllipsoidTransform[1][0] = 0;
  magEllipsoidTransform[1][1] = 1;
  magEllipsoidTransform[1][2] = 0;
  magEllipsoidTransform[2][0] = 0;
  magEllipsoidTransform[2][1] = 0;
  magEllipsoidTransform[2][2] = 1;
  
  Wire.begin();
  
  // Init accelerometer
  if (fastRegister8(ADXL345_REG_DEVID, ADXL345_ADDRESS) != 0xE5)
  {
    return false;
  }
  
  // Enable measurement mode
  writeRegister8(ADXL345_REG_POWER_CTL, 0x08, ADXL345_ADDRESS);
  
  clearAccSettings();
  
  // Init gyroscope
  if ((readRegister8(ITG3200_REG_WHO_AM_I, ITG3200_ADDRESS) & 0b011111110) != 0b01101000){
    return false;
  }
  
  // Reset to default
  writeRegisterBit(ITG3200_REG_PWR_MGM, 7, true, ITG3200_ADDRESS);
  delay(5);
  
  // Select range +- 2000 d/s. Unique value
  setGyrFs(ITG3200_FS_2000);
  
  // Select LP Filter to 42 Hz and F internal to 1 kHz
  setGyrLowPass(ITG3200_LOWPASS_42);
  
  // Select sample rate division, which is 100 Hz.
  setGyrRateDivider(9);
  
  // Configure to not sleep and clk sel
  setGyrClk(ITG3200_CLK_INTERNAL);
  
  // Set to not sleep
  writeRegisterBit(ITG3200_REG_PWR_MGM, 6, false, ITG3200_ADDRESS);
  
  // Init magnetometer
  if ((fastRegister8(HMC5883L_REG_IDENT_A, HMC5883L_ADDRESS) != 0x48)
  || (fastRegister8(HMC5883L_REG_IDENT_B, HMC5883L_ADDRESS) != 0x34)
  || (fastRegister8(HMC5883L_REG_IDENT_C, HMC5883L_ADDRESS) != 0x33))
  {
    return false;
  }
  
  setMagRange(HMC5883L_RANGE_1_3GA);
  setMagMeasurementMode(HMC5883L_CONTINOUS);
  setMagDataRate(HMC5883L_DATARATE_15HZ);
  setMagSamples(HMC5883L_SAMPLES_1);
  
  return true;
}

//-------------------------------------------------------------------------------------------
// Accelerometer functions                                                                 //
//-------------------------------------------------------------------------------------------

// Clear config
void SEN10724::clearAccSettings(void)
{
  setAccRange(ADXL345_RANGE_2G);
  setAccDataRate(ADXL345_DATARATE_100HZ);
  
  writeRegister8(ADXL345_REG_THRESH_TAP, 0x00, ADXL345_ADDRESS);
  writeRegister8(ADXL345_REG_DUR, 0x00, ADXL345_ADDRESS);
  writeRegister8(ADXL345_REG_LATENT, 0x00, ADXL345_ADDRESS);
  writeRegister8(ADXL345_REG_WINDOW, 0x00, ADXL345_ADDRESS);
  writeRegister8(ADXL345_REG_THRESH_ACT, 0x00, ADXL345_ADDRESS);
  writeRegister8(ADXL345_REG_THRESH_INACT, 0x00, ADXL345_ADDRESS);
  writeRegister8(ADXL345_REG_TIME_INACT, 0x00, ADXL345_ADDRESS);
  writeRegister8(ADXL345_REG_THRESH_FF, 0x00, ADXL345_ADDRESS);
  writeRegister8(ADXL345_REG_TIME_FF, 0x00, ADXL345_ADDRESS);
  
  uint8_t value;
  
  value = readRegister8(ADXL345_REG_ACT_INACT_CTL, ADXL345_ADDRESS);
  value &= 0b10001000;
  writeRegister8(ADXL345_REG_ACT_INACT_CTL, value, ADXL345_ADDRESS);
  
  value = readRegister8(ADXL345_REG_TAP_AXES, ADXL345_ADDRESS);
  value &= 0b11111000;
  writeRegister8(ADXL345_REG_TAP_AXES, value, ADXL345_ADDRESS);
}

// Read raw values
float* SEN10724::readAccRaw(void)
{
  accRaw[1] = readRegister16(ADXL345_REG_DATAX0, ADXL345_ADDRESS);
  accRaw[0] = readRegister16(ADXL345_REG_DATAY0, ADXL345_ADDRESS);
  accRaw[2] = readRegister16(ADXL345_REG_DATAZ0, ADXL345_ADDRESS);
  return accRaw;
}

// Read scaled values
float* SEN10724::readAccScaled(void)
{
  readAccRaw();
  accScaled[0] = accRaw[0] * 0.004;
  accScaled[1] = accRaw[1] * 0.004;
  accScaled[2] = accRaw[2] * 0.004;
  
  return accScaled;
}

// Read calibrated values
float* SEN10724::readAccCalib(void)
{
  readAccScaled();
  accCalib[0] = (accScaled[0] - accOffsets[0]) * accScales[0];
  accCalib[1] = (accScaled[1] - accOffsets[1]) * accScales[1];
  accCalib[2] = (accScaled[2] - accOffsets[2]) * accScales[2];
  
  return accCalib;
}

// Set accelerometer calibration parameters
void SEN10724::setAccMaxMinValues(float accMaxValues[3], float accMinValues[3])
{
  this->accMaxValues[0] = accMaxValues[0];
  this->accMaxValues[1] = accMaxValues[1];
  this->accMaxValues[2] = accMaxValues[2];
  
  this->accMinValues[0] = accMinValues[0];
  this->accMinValues[1] = accMinValues[1];
  this->accMinValues[2] = accMinValues[2];
  
  // recalculate the scale and offset for the new values
  this->accOffsets[0] = (accMinValues[0] + accMaxValues[0]) / 2.0f;
  this->accOffsets[1] = (accMinValues[1] + accMaxValues[1]) / 2.0f;
  this->accOffsets[2] = (accMinValues[2] + accMaxValues[2]) / 2.0f;
  this->accScales[0] = 1.0f / (accMaxValues[0] - accOffsets[0]);
  this->accScales[1] = 1.0f / (accMaxValues[1] - accOffsets[1]);
  this->accScales[2] = 1.0f / (accMaxValues[2] - accOffsets[2]);
}

// Set Range
void SEN10724::setAccRange(adxl345_range_t range)
{
  // Get actual value register
  uint8_t value = readRegister8(ADXL345_REG_DATA_FORMAT, ADXL345_ADDRESS);
  
  value &= 0xF0;
  value |= range;
  value |= 0x08;
  
  writeRegister8(ADXL345_REG_DATA_FORMAT, value, ADXL345_ADDRESS);
}

// Get Range
adxl345_range_t SEN10724::getAccRange(void)
{
  return (adxl345_range_t)(readRegister8(ADXL345_REG_DATA_FORMAT, ADXL345_ADDRESS) & 0x03);
}

// Set Data Rate
void SEN10724::setAccDataRate(adxl345_dataRate_t dataRate)
{
  writeRegister8(ADXL345_REG_BW_RATE, dataRate, ADXL345_ADDRESS);
}

// Get Data Rate
adxl345_dataRate_t SEN10724::getAccDataRate(void)
{
  return (adxl345_dataRate_t)(readRegister8(ADXL345_REG_BW_RATE, ADXL345_ADDRESS) & 0x0F);
}


//-------------------------------------------------------------------------------------------
// Gyroscope functions                                                                     //
//-------------------------------------------------------------------------------------------

// Read raw measurements
float* SEN10724::readGyrRaw(void)
{
  gyrRaw[1] = -readRegister16(ITG3200_REG_GYRO_XOUT_H, ITG3200_ADDRESS);
  gyrRaw[0] = -readRegister16(ITG3200_REG_GYRO_YOUT_H, ITG3200_ADDRESS);
  gyrRaw[2] = -readRegister16(ITG3200_REG_GYRO_ZOUT_H, ITG3200_ADDRESS);
  return gyrRaw;
}

// Read scaled measurements
float* SEN10724::readGyrScaled(void)
{
  readGyrRaw();
  gyrScaled[0] = gyrRaw[0] * 0.069565f;
  gyrScaled[1] = gyrRaw[1] * 0.069565f;
  gyrScaled[2] = gyrRaw[2] * 0.069565f;
  
  return gyrScaled;
}

// Read calibrated values
float* SEN10724::readGyrCalib(void)
{
  readGyrScaled();
  gyrCalib[0] = gyrScaled[0] - gyrOffsets[0];
  gyrCalib[1] = gyrScaled[1] - gyrOffsets[1];
  gyrCalib[2] = gyrScaled[2] - gyrOffsets[2];
  
  return gyrCalib;
}

// Set gyroscope calibration parameters
void SEN10724::setGyrOffsets(float gyrOffsets[3])
{
  this->gyrOffsets[0] = gyrOffsets[0];
  this->gyrOffsets[1] = gyrOffsets[1];
  this->gyrOffsets[2] = gyrOffsets[2];
}

// Read temperature
float SEN10724::readGyrTemperature(void){
  temperature = 0;
  temperature = ((float) readRegister16(ITG3200_REG_TEMP_OUT_H, ITG3200_ADDRESS))/(280.0f) + 82.14285f;
  
  return temperature;
}

// Set Fs. See datasheet
void SEN10724::setGyrFs(itg3200_fs_t fs)
{
  // Only one possibility. Other values are reserved
  writeRegisterBit(ITG3200_REG_DLPF_FS, 4, true, ITG3200_ADDRESS);
  writeRegisterBit(ITG3200_REG_DLPF_FS, 3, true, ITG3200_ADDRESS);
}

// Get Fs
itg3200_fs_t SEN10724::getGyrFs(void)
{
  return (itg3200_fs_t)(((readRegister8(ITG3200_REG_DLPF_FS, ITG3200_ADDRESS) & 0b00011000) >> 3 ));
}

// Set gyro Low Pass filter frequency
void SEN10724::setGyrLowPass(itg3200_lowPass_t lp)
{
  uint8_t value;
  value = readRegister8(ITG3200_REG_DLPF_FS, ITG3200_ADDRESS);
  value &= 0b11111000;
  value |= lp;
  writeRegister8(ITG3200_REG_DLPF_FS, value, ITG3200_ADDRESS);
}

// Get used frequency
itg3200_lowPass_t SEN10724::getGyrLowPass(void)
{
  uint8_t value;
  value = readRegister8(ITG3200_REG_DLPF_FS, ITG3200_ADDRESS);
  value &= 0b00000111;
  
  return (itg3200_lowPass_t)value;
}

// Set gyroscope clock
void SEN10724::setGyrClk(itg3200_clk_t clk)
{
  uint8_t value;
  value = readRegister8(ITG3200_REG_PWR_MGM, ITG3200_ADDRESS);
  value &= 0b11111000;
  value |= clk;
  writeRegister8(ITG3200_REG_PWR_MGM, value, ITG3200_ADDRESS);
}

// Get gyroscope clock value
itg3200_clk_t SEN10724::getGyrClk(void)
{
  uint8_t value;
  value = readRegister8(ITG3200_REG_PWR_MGM, ITG3200_ADDRESS);
  value &= 0b00000111;
  
  return (itg3200_clk_t)value;
}

// Set gyroscope rate divider
void SEN10724::setGyrRateDivider(uint8_t rate){
  writeRegister8(ITG3200_REG_SMPLRT_DIV, rate, ITG3200_ADDRESS);
}

// Get gyroscope rate divider
uint8_t SEN10724::getGyrRateDivider(void){
  return (uint8_t) readRegister8(ITG3200_REG_SMPLRT_DIV, ITG3200_ADDRESS);
}

//-------------------------------------------------------------------------------------------
// Magnetometer functions                                                                  //
//-------------------------------------------------------------------------------------------

// Read raw measurements
float* SEN10724::readMagRaw(void)
{
  magRaw[0] = readRegister16(HMC5883L_REG_OUT_X_M, HMC5883L_ADDRESS);
  magRaw[1] = -readRegister16(HMC5883L_REG_OUT_Y_M, HMC5883L_ADDRESS);
  magRaw[2] = -readRegister16(HMC5883L_REG_OUT_Z_M, HMC5883L_ADDRESS);
  
  return magRaw;
}

// Read scaled measurements
float* SEN10724::readMagScaled(void)
{
  magScaled[0] = ((float)readRegister16(HMC5883L_REG_OUT_X_M, HMC5883L_ADDRESS)) * mgPerDigit;
  magScaled[1] = ((float)readRegister16(HMC5883L_REG_OUT_Y_M, HMC5883L_ADDRESS)) * mgPerDigit;
  magScaled[2] = (float)readRegister16(HMC5883L_REG_OUT_Z_M, HMC5883L_ADDRESS) * mgPerDigit;
  
  return magScaled;
}

// Read calibrated measurements
float* SEN10724::readMagCalib(void)
{
  readMagScaled();
  
  // Run ellipsoid fit calibration
  float magnetom_tmp[3];
  
  magnetom_tmp[0] = magScaled[0] - magEllipsoidCenter[0];
  magnetom_tmp[1] = magScaled[1] - magEllipsoidCenter[1];
  magnetom_tmp[2] = magScaled[2] - magEllipsoidCenter[2];
  
  magCalib[0] = magEllipsoidTransform[0][0] * magnetom_tmp[0] + magEllipsoidTransform[0][1] * magnetom_tmp[1] + magEllipsoidTransform[0][2] * magnetom_tmp[2];
  magCalib[1] = magEllipsoidTransform[1][0] * magnetom_tmp[0] + magEllipsoidTransform[1][1] * magnetom_tmp[1] + magEllipsoidTransform[1][2] * magnetom_tmp[2];
  magCalib[2] = magEllipsoidTransform[2][0] * magnetom_tmp[0] + magEllipsoidTransform[2][1] * magnetom_tmp[1] + magEllipsoidTransform[2][2] * magnetom_tmp[2];
  
  return magCalib;
}

// Set ellipsoid calibration parameters
void SEN10724::setMagEllipsoidCalib(float magEllipsoidCenter[3], float magEllipsoidTransform[3][3])
{
  this->magEllipsoidCenter[0] = magEllipsoidCenter[0];
  this->magEllipsoidCenter[1] = magEllipsoidCenter[1];
  this->magEllipsoidCenter[2] = magEllipsoidCenter[2];
  
  this->magEllipsoidTransform[0][0] = magEllipsoidTransform[0][0];
  this->magEllipsoidTransform[0][1] = magEllipsoidTransform[0][1];
  this->magEllipsoidTransform[0][2] = magEllipsoidTransform[0][2];
  this->magEllipsoidTransform[1][0] = magEllipsoidTransform[1][0];
  this->magEllipsoidTransform[1][1] = magEllipsoidTransform[1][1];
  this->magEllipsoidTransform[1][2] = magEllipsoidTransform[1][2];
  this->magEllipsoidTransform[2][0] = magEllipsoidTransform[2][0];
  this->magEllipsoidTransform[2][1] = magEllipsoidTransform[2][1];
  this->magEllipsoidTransform[2][2] = magEllipsoidTransform[2][2];
}

// Set magnetometer range
void SEN10724::setMagRange(hmc5883l_range_t range)
{
  switch(range)
  {
    case HMC5883L_RANGE_0_88GA:
    mgPerDigit = 0.073f;
    break;
    
    case HMC5883L_RANGE_1_3GA:
    mgPerDigit = 0.92f;
    break;
    
    case HMC5883L_RANGE_1_9GA:
    mgPerDigit = 1.22f;
    break;
    
    case HMC5883L_RANGE_2_5GA:
    mgPerDigit = 1.52f;
    break;
    
    case HMC5883L_RANGE_4GA:
    mgPerDigit = 2.27f;
    break;
    
    case HMC5883L_RANGE_4_7GA:
    mgPerDigit = 2.56f;
    break;
    
    case HMC5883L_RANGE_5_6GA:
    mgPerDigit = 3.03f;
    break;
    
    case HMC5883L_RANGE_8_1GA:
    mgPerDigit = 4.35f;
    break;
    
    default:
    break;
  }
  
  writeRegister8(HMC5883L_REG_CONFIG_B, range << 5, HMC5883L_ADDRESS);
}

// Get magnetometer range
hmc5883l_range_t SEN10724::getMagRange(void)
{
  return (hmc5883l_range_t)((readRegister8(HMC5883L_REG_CONFIG_B, HMC5883L_ADDRESS) >> 5));
}

// Set magnetomer measurement mode
void SEN10724::setMagMeasurementMode(hmc5883l_mode_t mode)
{
  uint8_t value;
  
  value = readRegister8(HMC5883L_REG_MODE, HMC5883L_ADDRESS);
  value &= 0b11111100;
  value |= mode;
  
  writeRegister8(HMC5883L_REG_MODE, value, HMC5883L_ADDRESS);
}

// Get magnetometer measurement mode
hmc5883l_mode_t SEN10724::getMagMeasurementMode(void)
{
  uint8_t value;
  
  value = readRegister8(HMC5883L_REG_MODE, HMC5883L_ADDRESS);
  value &= 0b00000011;
  
  return (hmc5883l_mode_t)value;
}

// Set magnetometer data rate
void SEN10724::setMagDataRate(hmc5883l_dataRate_t dataRate)
{
  uint8_t value;
  
  value = readRegister8(HMC5883L_REG_CONFIG_A, HMC5883L_ADDRESS);
  value &= 0b11100011;
  value |= (dataRate << 2);
  
  writeRegister8(HMC5883L_REG_CONFIG_A, value, HMC5883L_ADDRESS);
}

// Get magnetometer data rate
hmc5883l_dataRate_t SEN10724::getMagDataRate(void)
{
  uint8_t value;
  
  value = readRegister8(HMC5883L_REG_CONFIG_A, HMC5883L_ADDRESS);
  value &= 0b00011100;
  value >>= 2;
  
  return (hmc5883l_dataRate_t)value;
}

// Set magnetometer samples
void SEN10724::setMagSamples(hmc5883l_samples_t samples)
{
  uint8_t value;
  
  value = readRegister8(HMC5883L_REG_CONFIG_A, HMC5883L_ADDRESS);
  value &= 0b10011111;
  value |= (samples << 5);
  
  writeRegister8(HMC5883L_REG_CONFIG_A, value, HMC5883L_ADDRESS);
}

// Get magnetometer samples
hmc5883l_samples_t SEN10724::getMagSamples(void)
{
  uint8_t value;
  
  value = readRegister8(HMC5883L_REG_CONFIG_A, HMC5883L_ADDRESS);
  value &= 0b01100000;
  value >>= 5;
  
  return (hmc5883l_samples_t)value;
}

//-------------------------------------------------------------------------------------------
// Read/Write functions                                                                    //
//-------------------------------------------------------------------------------------------

// Write byte to register
void SEN10724::writeRegister8(uint8_t reg, uint8_t value, uint8_t sensor)
{
  Wire.beginTransmission(sensor);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read byte to register
uint8_t SEN10724::fastRegister8(uint8_t reg, uint8_t sensor)
{
  uint8_t value;
  Wire.beginTransmission(sensor);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(sensor, 1);
  value = Wire.read();
  Wire.endTransmission();
  
  return value;
}

// Read byte from register
uint8_t SEN10724::readRegister8(uint8_t reg, uint8_t sensor)
{
  uint8_t value;
  Wire.beginTransmission(sensor);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.beginTransmission(sensor);
  Wire.requestFrom(sensor, 1);
  while(!Wire.available()) {};
  value = Wire.read();
  Wire.endTransmission();
  
  return value;
}

// Read word from register
int16_t SEN10724::readRegister16(uint8_t reg, uint8_t sensor)
{
  int16_t value;
  Wire.beginTransmission(sensor);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.beginTransmission(sensor);
  Wire.requestFrom(sensor, 2);
  while(!Wire.available()) {};
  
  uint8_t vla;
  uint8_t vha;
  if (sensor == ADXL345_ADDRESS){
    vla = Wire.read();
    vha = Wire.read();
  }
  else{
    vha = Wire.read();
    vla = Wire.read();
  }
  
  Wire.endTransmission();
  
  value = vha << 8 | vla;
  
  return value;
}

void SEN10724::writeRegisterBit(uint8_t reg, uint8_t pos, bool state, uint8_t sensor)
{
  uint8_t value;
  value = readRegister8(reg, sensor);
  
  if (state)
  {
    value |= (1 << pos);
  } else 
  {
    value &= ~(1 << pos);
  }
  
  writeRegister8(reg, value, sensor);
}

bool SEN10724::readRegisterBit(uint8_t reg, uint8_t pos, uint8_t sensor)
{
  uint8_t value;
  value = readRegister8(reg, sensor);
  return ((value >> pos) & 1);
}
