#ifndef MOTOR_CONTROLLER_CUSTOM_H
#define MOTOR_CONTROLLER_CUSTOM_H

#include <LMotorController.h>

extern LMotorController motor;

void initMotor() {

}

void moveMotor(double power) {
  motor.move(power, 20);
}

#endif
