#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SimpleKalmanFilter.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

// Create a Kalman filter instance for each quaternion component
// Adjusted parameters for tuning
SimpleKalmanFilter kalmanW(1.0, 1.0, 0.03);  // Measurement Noise, Estimation Error, Process Noise
SimpleKalmanFilter kalmanX(1.0, 1.0, 0.03);
SimpleKalmanFilter kalmanY(1.0, 1.0, 0.03);
SimpleKalmanFilter kalmanZ(1.0, 1.0, 0.03);

void setup() {
  Serial.begin(115200);
  myIMU.begin();
  delay(1000);
  myIMU.setExtCrystalUse(true);
}

void loop() {
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);

  imu::Quaternion quat = myIMU.getQuat();

  // Apply the Kalman filter to each quaternion component
  float filteredW = kalmanW.updateEstimate(quat.w());
  float filteredX = kalmanX.updateEstimate(quat.x());
  float filteredY = kalmanY.updateEstimate(quat.y());
  float filteredZ = kalmanZ.updateEstimate(quat.z());

  // Print the filtered quaternion values
  Serial.print(filteredW);
  Serial.print(",");
  Serial.print(filteredX);
  Serial.print(",");
  Serial.print(filteredY);
  Serial.print(",");
  Serial.print(filteredZ);
  Serial.print(",");
  Serial.print(accel);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(mg);
  Serial.print(",");
  Serial.println(system);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
