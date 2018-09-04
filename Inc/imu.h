#ifndef IMU_H
#define IMU_H

#include "stdint.h"

#define PITCH_INDEX 0
#define ROLL_INDEX 1
#define YAW_INDEX 2
#define X_AXIS_INDEX 0
#define Y_AXIS_INDEX 1
#define Z_AXIS_INDEX 2
#define AXIS_COUNT 3

class Imu {
 public:
  Imu();
  ~Imu();

  void GetFilteredAngles(float deltaTime, float *pfData);

 protected:
  void LoadGyroData();
  void LoadAccData();
  void ComplementaryFilter(float deltaTime, float *pfData);
  
  float gyroAngVelocity[AXIS_COUNT];
  int16_t accAngles[AXIS_COUNT];
  int16_t magnetometer[AXIS_COUNT];
};

#endif /* IMU_H */
