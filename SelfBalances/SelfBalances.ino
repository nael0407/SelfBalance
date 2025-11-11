#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include "MPUHandler.h"
#include "MotorControllerCustom.h"
#include "PIDConfig.h"

MPU6050 mpu;

float prevAngle = 0;
unsigned long prevTime = 0;

double setPoint = 0;
double input;
double output;

double Kp = 0;
double Ki = 0;
double Kd = 0;

PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int ENB = 9;
int IN3 = 10;
int IN4 = 11;

LMotorController motor(ENA, IN1, IN2, ENB, IN3, IN4, 1.0, 1.0);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  initMPU();
  initPID();
  initMotor();

  prevTime = millis();
}

void loop() {
  if (millis() - prevTime >= 10) {
    prevTime = millis();

    input = getAngle();
    pid.Compute();
    moveMotor(output);
  }
}
