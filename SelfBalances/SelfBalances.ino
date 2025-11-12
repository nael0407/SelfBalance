#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include "MPUHandler.h"
#include "MotorControllerCustom.h"
#include "PIDConfig.h"

MPU6050 mpu;

float prevAngle = 0;

double setPoint = 0;
double input;
double output;

double Kp = 25;
double Ki = 0.5;
double Kd = 2;

PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

int ENA = 27;
int IN1 = 32;
int IN2 = 33;
int ENB = 14;
int IN3 = 25;
int IN4 = 26;

LMotorController motor(ENA, IN1, IN2, ENB, IN3, IN4, 1.0, 1.2);

const float FALL_ANGLE = 45.0;
const float STARTUP_ANGLE = 10.0;
bool isBalancing = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  initMPU();
  initPID();
  initMotor();

  prevAngle = 0;
  getAngle();
  delay(10);
  
  float angleSum = 0;
  for(int i = 0; i < 100; i++) {
    angleSum += getAngle();
    delay(10);
  }
  setPoint = angleSum / 100.0;

  isBalancing = true;
}

void loop() {
  input = getAngle();
  if (abs(input - setPoint) > FALL_ANGLE) {
    motor.stopMoving();
    isBalancing = false;
    pid.SetMode(MANUAL);
    
    while(abs(input - setPoint) > STARTUP_ANGLE) {
      input = getAngle();
      delay(100);
    }
    
    pid.SetMode(AUTOMATIC);
    isBalancing = true;
    delay(200);
  }
  
  if (isBalancing && pid.Compute()) {
    Serial.print("Angle: ");
    Serial.print(input);
    Serial.print(" | Output: ");
    Serial.println(output);
      
    moveMotor(output);
  } 
}
