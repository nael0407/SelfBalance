#ifndef PID_CONFIG_H
#define PID_CONFIG_H

#include <PID_v1.h>

extern PID pid;
extern unsigned long prevTime;

void initPID() {
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
}

#endif
