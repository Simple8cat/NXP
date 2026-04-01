#include <Arduino.h>
#include <Wire.h>
#include <Pixy2.h>
#include <Servo.h>

#define I2C_SLAVE_ADDR 0x08

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

volatile int i2c_speed_left = 0;
volatile int i2c_speed_right = 0;
volatile int i2c_steering_angle = CENTRE_ANGLE;
volatile bool i2c_new_command = false;

Servo steer_servo;
Pixy2 pixy;

void onReceive(int howMany) {
  if (Wire.available() >= 3) {
    char cmd = Wire.read();
    byte val1 = Wire.read();
    byte val2 = Wire.read();

    if (cmd == 's') { // Speed command
      i2c_speed_left = map(val1, 0, 255, -255, 255);
      i2c_speed_right = map(val2, 0, 255, -255, 255);
      i2c_new_command = true;
    } else if (cmd == 't') { // Steering command
      i2c_steering_angle = map(val1, 0, 255, MAX_LEFT, MAX_RIGHT);
      i2c_new_command = true;
    }
  }
  // Discard extra bytes
  while (Wire.available() > 0) {
    Wire.read();
  }
}

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

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onReceive);

  steer_servo.attach(SERVO_PIN);
  steer(CENTRE_ANGLE);

  initPixy();

  digitalWrite(LED, HIGH);
  run(100,100);
  //run(0,0);
}

void loop()
{
  if (i2c_new_command) {
    run(i2c_speed_left, i2c_speed_right);
    steer(i2c_steering_angle);
    i2c_new_command = false;
  }
  // The existing line following code can be commented out or removed 
  // if you want to control the robot only via I2C.
  /*
  pixy.line.getMainFeatures();
  double angle = atan2(pixy.line.vectors[0].m_y1 - pixy.line.vectors[0].m_y0, pixy.line.vectors[0].m_x1 - pixy.line.vectors[0].m_x0) * 180.0 / PI;
  if (angle > 90) angle -= 180;
  if (angle < -90) angle += 180;
  Serial.print("Angle: ");
  Serial.println(angle);
  if(angle<=50 && angle>=15){
    int steering_angle = map(angle, 15, 50, CENTRE_ANGLE, MAX_RIGHT);
    steer(steering_angle);
  }else if(angle<15){
    int steering_angle = map(angle, -70, 15, MAX_LEFT, CENTRE_ANGLE);
    steer(steering_angle);
  }
  */
}