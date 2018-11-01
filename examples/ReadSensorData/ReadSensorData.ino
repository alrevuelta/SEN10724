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
  float* accScaled = sen10724.readAccScaled();
  float* gyrScaled = sen10724.readGyrScaled();
  float* magScaled = sen10724.readMagScaled();

  // Print values to serial port
  sendData(accScaled, gyrScaled, magScaled);
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
