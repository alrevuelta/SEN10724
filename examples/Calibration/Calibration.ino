// Includes
#include "SEN10724.h"

// Object instantiation
SEN10724 sen10724;

// Time
unsigned long timestamp;
unsigned long timestamp_old;
float deltaT;

void setup()
{
  Serial.begin(115200);

  Serial.println("Initialize SEN 10724 Sensor Stick");
  if (!sen10724.begin())
  {
    Serial.println("Error");
    delay(500);
  }

  // Calibration values for accelerometer
  float accMaxValues[3] = {1.064, 1.016, 1.044};
  float accMinValues[3] = {-1.068, -1.08, -1.236};

  // Calibration values for gyroscope
  float gyrOffsets[3] = {-0.904, 5.426, 0.626};

  // Calbration values for magnetometer
  float magEllipsoidCenter[3] = {-178.266, 92.8594, -59.4538};
  float magEllipsoidTransform[3][3] = {{0.789315, -0.0151758, -0.0645366},
                                       {-0.0151758, 0.982619, 0.00424189},
                                       {-0.0645366, 0.00424189, 0.975379}};

  // Set the calibration values
  sen10724.setAccMaxMinValues(accMaxValues, accMinValues);
  sen10724.setGyrOffsets(gyrOffsets);
  sen10724.setMagEllipsoidCalib(magEllipsoidCenter, magEllipsoidTransform);

  // Accelerometer configuration
  sen10724.setAccRange(ADXL345_RANGE_16G);
  sen10724.setAccDataRate(ADXL345_DATARATE_400HZ);

  // Gyroscope configuration
  sen10724.setGyrFs(ITG3200_FS_2000);
  sen10724.setGyrLowPass(ITG3200_LOWPASS_42);
  sen10724.setGyrClk(ITG3200_CLK_INTERNAL);
  sen10724.setGyrRateDivider(9);

  // Magnetometer configuration
  sen10724.setMagRange(HMC5883L_RANGE_1_3GA);
  sen10724.setMagMeasurementMode(HMC5883L_CONTINOUS);
  sen10724.setMagDataRate(HMC5883L_DATARATE_15HZ);
  sen10724.setMagSamples(HMC5883L_SAMPLES_1);
  
  delay(50);
  Serial.println("SEN 10724 was initiated successfully");
}

void loop()
{
  // Calcualte time since last update
  timestamp_old = timestamp;
  timestamp = micros();
  if (timestamp > timestamp_old)
    deltaT = (float) (timestamp - timestamp_old) / 1000000.0f;
  else deltaT = 0;

  // Read values
  float* accCalib = sen10724.readAccCalib();
  float* gyrCalib = sen10724.readGyrCalib();
  float* magCalib = sen10724.readMagCalib();

  // Print values to serial port
  sendData(accCalib, gyrCalib, magCalib);
}

void sendData(float accData[3], float gyrData[3], float magData[3])
{
  Serial.print("Time-Acc-Gyr-Mag=");
  Serial.print(String(deltaT, 5));     Serial.print(",");
  Serial.print(String(accData[0], 3)); Serial.print(",");
  Serial.print(String(accData[1], 3)); Serial.print(",");
  Serial.print(String(accData[2], 3)); Serial.print(",");
  Serial.print(String(gyrData[0], 3)); Serial.print(",");
  Serial.print(String(gyrData[1], 3)); Serial.print(",");
  Serial.print(String(gyrData[2], 3)); Serial.print(",");
  Serial.print(String(magData[0], 3)); Serial.print(",");
  Serial.print(String(magData[1], 3)); Serial.print(",");
  Serial.println(String(magData[2], 3));
}
