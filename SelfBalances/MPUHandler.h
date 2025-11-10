#ifndef MPU_HANDLER_H
#define MPU_HANDLER_H

#include <MPU6050.h>
#include <Arduino.h>

extern MPU6050 mpu;
extern float prevAngle;

void initMPU() {
  mpu.initialize();
}

float getAngle() {
  int16_t ax, ay, az, gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float dt = 0.01;
  float angleAcc = atan2(ay, az) * 180.0 / PI;
  float angleGyro = prevAngle + (gx / 131.0) * dt;
  float angle = 0.98 * angleGyro + 0.02 * angleAcc;

  prevAngle = angle;
  return angle;
}

#endif
