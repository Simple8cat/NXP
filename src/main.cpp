#include <Arduino.h>
#include <Wire.h>
#include <Pixy2.h>
#include <Servo.h>

#define ENC_LEFT_A   4
#define ENC_LEFT_B   5
#define ENC_RIGHT_A  3
#define ENC_RIGHT_B  2

#define PWM1 8
#define IN1 6
#define IN2 7
#define PWM2 18
#define IN3 20
#define IN4 19

#define LED 22

#define ECHO 15
#define TRIG 14

#define SERVO_PIN 23

#define CENTRE_ANGLE 135
#define MAX_RIGHT (CENTRE_ANGLE + 45)
#define MAX_LEFT  (CENTRE_ANGLE - 45)

#define TICKS_PER_REV  400.0f
#define WHEEL_DIAMETER 65.0f // mm

volatile long leftCount = 0;
volatile long rightCount = 0;

Servo steer_servo;
Pixy2 pixy;

void initPixy(){
  pixy.init();
  pixy.changeProg("line");
}

void leftEncoderISR() {
  if (digitalReadFast(ENC_LEFT_B) == digitalReadFast(ENC_LEFT_A))
    leftCount++;
  else
    leftCount--;
}

void rightEncoderISR() {
  if (digitalReadFast(ENC_RIGHT_B) == digitalReadFast(ENC_RIGHT_A))
    rightCount++;
  else
    rightCount--;
}

void steer(int angle)
{
  angle = constrain(angle, MAX_LEFT, MAX_RIGHT);
  steer_servo.write(angle);
}

void run(int speedLeft, int speedRight)
{
  speedLeft  = constrain(speedLeft,  -255, 255);
  speedRight = constrain(speedRight, -255, 255);

  // RIGHT MOTOR
  if (speedRight >= 0)
  {
    analogWrite(PWM2, speedRight);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    analogWrite(PWM2, -speedRight);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  // LEFT MOTOR
  if (speedLeft >= 0)
  {
    analogWrite(PWM1, speedLeft);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    analogWrite(PWM1, -speedLeft);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  analogWriteResolution(8);
  analogWriteFrequency(PWM1, 20000);
  analogWriteFrequency(PWM2, 20000);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LED, OUTPUT);

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);

  steer_servo.attach(SERVO_PIN);
  steer(CENTRE_ANGLE);

  initPixy();

  digitalWrite(LED, HIGH);
  run(0,0);
}

void loop()
{
}