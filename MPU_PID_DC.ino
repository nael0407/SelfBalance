#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 mpu;

int16_t ax_offset = 0;
int16_t ay_offset = 0;
int16_t az_offset = 0;
int16_t gx_offset = 0;
int16_t gy_offset = 0;
int16_t gz_offset = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;

float angleAcc, angleGyro, angle;
float prevAngle = 0;
unsigned long prevTime;

double setPoint = 0;
double input;
double output;

double Kp = ;
double Ki = ;
double Kd = ; 

PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

int ENA = ;
int IN1 = ;
int IN2 = ;

int ENB = ;
int IN3 = ;
int IN4 = ;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  prevTime = millis();
}

void setMotor(int pwm) {
  int speed = abs(pwm);
  if (speed > 255) speed = 255;

  if (pwm > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, speed);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, speed);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, speed);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, speed);
  }
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  angleAcc = atan2(ay, az) * 180 / PI;
  angleGyro = prevAngle + (gx / 131.0) * dt;
  angle = 0.98 * angleGyro + 0.02 * angleAcc;

  prevAngle = angle;
  input = angle;

  pid.Compute();
  setMotor(output);
  delay(10);
}
