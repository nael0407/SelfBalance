#include <PID_v1.h>

double originalSetPoint = ;
double setPoint = originalSetPoint;
double movingAngleOffSet = 0.1;
double input, ouput;

double Kp = ;
double Ki = ;
double Kd = ;

PID pid(&input, &putput, &setPoint Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-225, 225);
}

void loop() {
  
  pid.compute();
  delay(10)
}
