#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float angleAcc, angleGyro, angle;
float prevAngle = 0;
unsigned long prevTime;
double input, output;

double setPoint = ; 
double Kp = ;
double Ki = ;
double Kd = ;

PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);

  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 terhubung!");
  } else {
    Serial.println("Gagal konek ke MPU6050!");
    while(1);
  }

  prevTime = millis();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  angleAcc = atan2(ay, az) * 180.0 / PI;

  angleGyro = prevAngle + (gx / 131.0) * dt;

  angle = 0.98 * angleGyro + 0.02 * angleAcc;

  prevAngle = angle;

  input = angle;
  pid.Compute();

  Serial.print("Sudut: ");
  Serial.print(angle);
  Serial.print(" | Output PID: ");
  Serial.println(output);

  delay(5);
}
