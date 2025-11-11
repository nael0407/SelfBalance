#ifndef MPU_HANDLER_H
#define MPU_HANDLER_H

#include <MPU6050.h>
#include <Arduino.h>

extern MPU6050 mpu;
extern float prevAngle;

void initMPU() {
  mpu.initialize();
  mpu.setXAccelOffset(-1500);
  mpu.setYAccelOffset(200);
  mpu.setZAccelOffset(800);

  mpu.setXGyroOffset(50);
  mpu.setYGyroOffset(-10);
  mpu.setZGyroOffset(30);
}

float getAngle() {
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float angleAcc = atan2(ay, az) * 180.0 / PI;
  float angleGyro = prevAngle + (gx / 131.0) * dt;
  float angle = 0.98 * angleGyro + 0.02 * angleAcc;

  prevAngle = angle;
  return angle;
}

#endif
