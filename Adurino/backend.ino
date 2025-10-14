#include <Adafruit_PWMServoDriver.h>
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define PWM_MODE_SPARKMAX      0
#define PWM_MODE_AM32          1      // 1–2 ms, 1500us neutral (bi-dir / 3D enabled)
#define PWM_MODE_AM32_TIGHT    2      // some AM32 drive firmwares ~1020/1500/1980
// ---- Use your pin layout, but with analogWrite PWM on ENA/ENB ----
// values[0] -> Motor A (in1/in2 + ena)
// values[1] -> Motor B (in3/in4 + enb)
// Range for each value: [-1.0, 1.0]
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t MAX_FLOATS = 11;
float joint_values[MAX_FLOATS];

//Drive Base
int leftDriveMotorPinFront = 1;
int leftDriveMotorPinBack = 2;
int rightDriveMotorPinFront = 3;
int rightDriveMotorPinBack = 4;

//Shooter Motors
int shooter_motor_1 = 5;
int shooter_motor_2 = 6;

//Pitch and Yaw Servos
int pitch_servo = 7;
int yaw_servo = 8;

//Intake Motors
int intake_motor = 999;
int intake_servo = 999;

//Stick
int stick = 999;

//Sensor
// int beambreak = 2;
int encoder = 3;

const float data_out[1];

double Kp=2, Ki=5, Kd=1;


void setup() {
  Serial.begin(115200);
  pinMode(encoder, INPUT_PULLUP);
  pwm.begin();
  
}
//    0                      1                 2              3              4                     5                           6             7               8             9          10 
//FL_drive_wheel_vel, BL_drive_wheel_vel, BR_wheel_vel, FR_wheel_vel, left_shooter_wheel_vel, right_shooter_wheel_vel, intake_motor, yaw_servo_pos, pitch_servo_pos, stick_servo, intake_servo
void loop() {
  // Expect at least 2 floats = 8 bytes from Serial
  if (Serial.available() >= MAX_FLOATS) {
    // Read the first two float values for the two motors
    for (uint8_t i = 0; i < 2; ++i) {
      uint8_t b[4];
      size_t got = Serial.readBytes(b, 4);
      if (got < 4) return; // Incomplete packet; try again next loop
      memcpy(&joint_values[i], b, 4);
    }
    driveTrainMotor(joint_values[0],joint_values[3],joint_values[2],joint_values[1]);
    shooterMotor(joint_values[4],joint_values[5]);
    turretCommand(joint_values[8],joint_values[7]);
    intakeMotorandServo(joint_values[9], joint_values[10]);
    stickCommand(joint_values[10]);
  }
}




void setPWMScaled(uint8_t channel, float value, uint8_t mode = PWM_MODE_SPARKMAX) {
  // ---- Choose profile parameters (all local to this function) ----
  int us_min, us_neu, us_max, deadband_us, freq_hz;
  switch (mode) {
    case PWM_MODE_AM32:
      us_min = 1000; us_neu = 1500; us_max = 2000; deadband_us = 30; freq_hz = 50;
      break;
    case PWM_MODE_AM32_TIGHT:
      us_min = 1020; us_neu = 1500; us_max = 1980; deadband_us = 50; freq_hz = 50;
      break;
    case PWM_MODE_SPARKMAX:
    default:
      us_min = 1000; us_neu = 1500; us_max = 2000; deadband_us = 50; freq_hz = 50;
      break;
  }

  // ---- Clamp input to [-1, +1] ----
  if (value >  1.0f) value =  1.0f;
  if (value < -1.0f) value = -1.0f;

  // ---- Deadband around neutral (convert µs deadband to a fractional width) ----
  int span_us = us_max - us_neu;                  // e.g., 2000-1500 = 500
  float db = (span_us > 0) ? (float)deadband_us / (float)span_us : 0.0f;
  float absval = (value >= 0.f) ? value : -value;
  if (absval < db) value = 0.0f;

  // ---- Map to microseconds (−1→us_min, 0→us_neu, +1→us_max) ----
  // integer rounding without lroundf to keep things Arduino-friendly
  int us = us_neu + (int)((value * span_us) + (value >= 0.f ? 0.5f : -0.5f));
  if (us < us_min) us = us_min;
  if (us > us_max) us = us_max;

  // ---- Output pulse ----
  pwm.writeMicroseconds(channel, us);
}

void driveTrainMotor(float FLcmd, float FRcmd, float BRcmd, float BLcmd) {
  setPWMScaled(leftDriveMotorPinFront,FLcmd, PWM_MODE_SPARKMAX);
  setPWMScaled(leftDriveMotorPinBack,BLcmd, PWM_MODE_SPARKMAX);
  setPWMScaled(rightDriveMotorPinFront,FRcmd), PWM_MODE_SPARKMAX;
  setPWMScaled(rightDriveMotorPinBack,BRcmd, PWM_MODE_SPARKMAX);
}

void shooterMotor(float cmd_1, float cmd_2){
  setPWMScaled(shooter_motor_1, cmd_1, PWM_MODE_AM32);
  setPWMScaled(shooter_motor_2, cmd_2, PWM_MODE_AM32);
}

void intakeMotorandServo(float motor_cmd, float servo_cmd){
  setPWMScaled(intake_motor,motor_cmd, PWM_MODE_AM32);
  setServoAngle(intake_servo,servo_cmd);
}


void turretCommand(float pitchAngle, float yawAngle) {
  setServoAngle(pitch_servo, pitchAngle);
  setServoAngle(yaw_servo, yawAngle);
}


void stickCommand(float cmd) {
  setServoAngle(stick, cmd);
}

void setServoAngle(uint8_t channel, float angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  int pulseWidth = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  pwm.writeMicroseconds(channel, pulseWidth);
}

float readAS5600_PWM_RPM(int PWMpin, bool reset = false)
{
  static float prevAngle = NAN;
  static uint32_t prevTime = 0;
  static float rpmEMA = 0;
  const float alpha = 0.25;  // smoothing factor

  if (reset) {
    prevAngle = NAN;
    prevTime = 0;
    rpmEMA = 0;
    return 0;
  }

  // Wait for start of PWM frame
  while (digitalRead(PWMpin) == HIGH);
  while (digitalRead(PWMpin) == LOW);
  uint32_t rise = micros();

  // Measure high and full periods
  while (digitalRead(PWMpin) == HIGH);
  uint32_t highPeriod = micros() - rise;
  while (digitalRead(PWMpin) == LOW);
  uint32_t fullPeriod = micros() - rise;

  // Decode angle
  float bitTime = fullPeriod / 4351.0f;
  float dataPeriod = highPeriod - 128.0f * bitTime;
  float angle = 360.0f * dataPeriod / (4095.0f * bitTime);
  if (angle < 0) angle = 0;
  if (angle > 360) angle = 360;

  // Compute RPM if we have a previous sample
  uint32_t now = micros();
  float rpm = 0;
  if (!isnan(prevAngle)) {
    float dt = (now - prevTime) * 1e-6f;
    if (dt > 0) {
      float d = angle - prevAngle;
      if (d > 180.0f) d -= 360.0f;
      if (d < -180.0f) d += 360.0f;
      rpm = (d / dt) / 6.0f;  // deg/s → RPM
      rpmEMA = (1.0f - alpha) * rpmEMA + alpha * rpm;
    }
  }

  prevAngle = angle;
  prevTime = now;
  return rpmEMA;
}

void sendSerialSensor(){
  float rpm = readAS5600_PWM_RPM(encoder);
  memcpy(&data_out[0], &rpm, 4);
  Serial.write((const uint8_t*)data_out, 1 * sizeof(float));  // raw bytes
}