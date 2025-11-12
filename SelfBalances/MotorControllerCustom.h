#ifndef MOTOR_CONTROLLER_CUSTOM_H
#define MOTOR_CONTROLLER_CUSTOM_H

#include <LMotorController.h>

extern LMotorController motor;

extern int ENA;
extern int IN1;
extern int IN2;
extern int ENB;
extern int IN3;
extern int IN4;

void initMotor() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void moveMotor(double power) {
  power = constrain(power, -255, 255);
  if (abs(power) < 10) power = 0;
  motor.move((int)power);
}

#endif