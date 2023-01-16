#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

const int leftLedPin = 3;
const int rightLedPin = 9;

int m1Speed = 0;
int m2Speed = 0;

float kp = 11;
float ki = 0;
float kd = 2;
int p = 1;
int i = 0;
int d = 0;
int error = 0;
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 255;

QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };

unsigned long lastCalibrationMovement = 0;

int calibrationMovementInterval = 500;

bool motorsRunning = false;

int motor1CalibrationSpeed = 200;
int motor2CalibrationSpeed = 0;
const int motor1CalibrationSpeedForward = 255;
const int motor1CalibrationSpeedBackward = -200;

const int minLedValue = 0;
const int maxLedValue = 255;

const int minSensorValue = 0;
const int maxSensorValue = 5000;

const int minErrorValue = -100;
const int maxErrorValue = 100;

const int calibrationCount = 400;

void setup() {
  Serial.begin(9600);

  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  pinMode(leftLedPin, OUTPUT);
  pinMode(rightLedPin, OUTPUT);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  for (uint16_t i = 0; i < calibrationCount; i++) {
    qtr.calibrate();
    moveToCalibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  int error = map(qtr.readLineBlack(sensorValues), minSensorValue, maxSensorValue, minErrorValue, maxErrorValue);

  calculateMotorSpeed(error);
  changeLightBeacon();
  setMotorSpeed(m1Speed, m2Speed);
}

void moveToCalibrate(){
  if (millis() - lastCalibrationMovement >= calibrationMovementInterval) {
    if (motorsRunning) {
      setMotorSpeed(0, 0);
      motorsRunning = false;
    } else {
      setMotorSpeed(motor1CalibrationSpeed, motor2CalibrationSpeed);
      if (motor1CalibrationSpeed > 0) {
        //set motor movement backwards
        motor1CalibrationSpeed = motor1CalibrationSpeedBackward;
      } else {
        motor1CalibrationSpeed = motor1CalibrationSpeedForward;
      }
      motorsRunning = true;
    }
    lastCalibrationMovement = millis();
  }
}

void changeLightBeacon(){
  if (m1Speed > m2Speed) {
    analogWrite(leftLedPin, maxLedValue);
    analogWrite(rightLedPin, minLedValue);
  } else {
    analogWrite(leftLedPin, minLedValue);
    analogWrite(rightLedPin, maxLedValue);
  }
}

void calculateMotorSpeed(int error){
  int motorSpeed = pidControl(kp, ki, kd, error);  // = error in this case
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  if (error < 0) {
    m1Speed += motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
  }

  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);
}

// calculate PID value based on error, kp, kd, ki, p, i and d.
float pidControl(float kp, float ki, float kd, int error) {
  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;

  return kp * p + ki * i + kd * d;
}

void setMotorSpeed(int motor1Speed, int motor2Speed) {

  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}
