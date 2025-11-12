#ifndef MPU_HANDLER_H
#define MPU_HANDLER_H

#include <MPU6050.h>
#include <Arduino.h>

extern MPU6050 mpu;
extern float prevAngle;

void initMPU() {
  mpu.initialize();
  mpu.setXAccelOffset(1032);
  mpu.setYAccelOffset(1404);
  mpu.setZAccelOffset(1059);

  mpu.setXGyroOffset(128);
  mpu.setYGyroOffset(-22);
  mpu.setZGyroOffset(32);
}

float getAngle() {
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  
  if (lastTime == 0) {
    lastTime = now;
    return prevAngle;
  }

  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (dt > 0.1) dt = 0.01;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float angleAcc = atan2(ay, az) * 180.0 / PI;
  float angleGyro = prevAngle + (gx / 131.0) * dt;
  float angle = 0.98 * angleGyro + 0.02 * angleAcc;

  prevAngle = angle;
  return angle;
}

#endif