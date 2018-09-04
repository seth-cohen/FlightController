#include "imu.h"

#include "stdlib.h"
#include "string.h"
#include "stm32f3_discovery.h"
#include "stm32f3_discovery_gyroscope.h"
#include "stm32f3_discovery_accelerometer.h"

#define ARM_MATH_CM4
#include "arm_math.h"

Imu::Imu() {
  memset(gyroAngVelocity, 0, sizeof(gyroAngVelocity));
  memset(accAngles, 0, sizeof(accAngles));
  memset(magnetometer, 0, sizeof(magnetometer));

  BSP_GYRO_Init();
  BSP_ACCELERO_Init();
}

Imu::~Imu() {
}

void Imu::GetFilteredAngles(float deltaTime, float *pfData) {
  LoadGyroData();
  LoadAccData();
  ComplementaryFilter(deltaTime, pfData);
}

void Imu::ComplementaryFilter(float deltaTime, float *pfData) {
  float pitchAcc, rollAcc;

  float *pitch = &pfData[PITCH_INDEX], *roll = &pfData[ROLL_INDEX], *yaw = &pfData[YAW_INDEX];
  if (deltaTime > 5) return;

  // Integrate the gyro data - integral(angularSpeed) = angle
  // but careful because it drifts hence the need for the filter
  *pitch += (gyroAngVelocity[PITCH_INDEX] * deltaTime);   // Angle around the X-axis
  *roll += (gyroAngVelocity[ROLL_INDEX] * deltaTime);    // Angle around the Y-axis
  *yaw += (gyroAngVelocity[YAW_INDEX] * deltaTime);     // Angle around the Z-axis

  // Unfortunately, due to the fact that gravity is in the downward Z-axis there
  // is no way to use the accelerometer to "fix" the gyro drift of Z-rotation

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int16_t absX = abs(accAngles[X_AXIS_INDEX]);
  int16_t absY = abs(accAngles[Y_AXIS_INDEX]);

  uint32_t forceMagnitudeApprox = absX + absY + abs(accAngles[Z_AXIS_INDEX]);
  // @TODO - save these noise levels as consts somewhere
  if (absX < 200)
    accAngles[X_AXIS_INDEX] = 0; // ignore noise
  if (absY < 200)
    accAngles[Y_AXIS_INDEX] = 0;
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768) {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float) accAngles[Y_AXIS_INDEX], (float) accAngles[Z_AXIS_INDEX]) * 180.0f / PI;
    *pitch = *pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Z axis results in a vector on the X-axis
    rollAcc = atan2f((float) accAngles[X_AXIS_INDEX], (float) accAngles[Z_AXIS_INDEX]) * 180.0f / PI;
    *roll = *roll * 0.98 + rollAcc * 0.02;

    // @TODO - combine magnetometer data with Gyro for filter yaw drift compensation
  }
}

void Imu::LoadGyroData() {
  BSP_GYRO_GetXYZ(gyroAngVelocity);
  for (int i = 0; i < AXIS_COUNT; i++) {
    gyroAngVelocity[i] *= 0.001f;
  }
}

void Imu::LoadAccData() {
  BSP_ACCELERO_GetXYZ(accAngles);
}
