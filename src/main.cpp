#include <Arduino.h>
#include <Pixy2.h>
#include <Servo.h>
#include <Wire.h>

// CAMERA MAX X AND Y
// Max X0: 78, Max Y0: 51, Max X1: 78, Max Y1: 51

unsigned long startTime = 0;
constexpr uint8_t I2C_SLAVE_ADDR = 0x08;

#define MAX_VECTORS 20
#define MIN_VECTOR_LENGTH 10

#define ENC_LEFT_A 4
#define ENC_LEFT_B 5
#define ENC_RIGHT_A 3
#define ENC_RIGHT_B 2

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
#define MAX_RIGHT CENTRE_ANGLE + 45
#define MAX_LEFT CENTRE_ANGLE - 45

constexpr float PIXY_FORWARD_ANGLE = 270.0;
constexpr float PIXY_STEER_DEADBAND = 15.0; // 10 old
constexpr uint8_t STEERING_DELAY_CYCLES = 2; // parfait
constexpr float STEERING_QUEUE_THRESHOLD = 2.0;
constexpr float STEERING_CENTER_BIAS = 5.0;
constexpr float STEERING_SOFT_LIMIT_START = 15.0; // 25.0: cross mrigl, zigzag non;;; 15.0 corss non, zigzag mrigl;;; 20.0: 50% 50% sa3et yed5m mrigl sa3et le
constexpr float STEERING_SOFT_LIMIT_FACTOR = 0.5;

Servo steer_servo;
Pixy2 pixy;
static float lastSteeringAngle = CENTRE_ANGLE;
static float pendingSteeringAngle = CENTRE_ANGLE;
static uint8_t steeringDelayCounter = 0;

volatile long leftCount = 0;
volatile long rightCount = 0;

volatile int i2c_speed_left = 0;
volatile int i2c_speed_right = 0;
volatile int i2c_steering_angle = CENTRE_ANGLE;
volatile bool i2c_new_command = false;


long readDistance()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 5000); // timeout 5ms
  if (duration == 0)
  {
    return 99; // timeout, no object detected
  }
  long distance = duration * 0.034 / 2;       // cm

  return distance;
}

long readDistanceFiltered()
{
  const int numReadings = 3;
  long readings[numReadings];
  long sum = 0;

  for (int i = 0; i < numReadings; i++)
  {
    readings[i] = readDistance();
    sum += readings[i];
    delay(1);
  }

  return sum / numReadings;
}

void leftEncoderISR()
{
  if (digitalReadFast(ENC_LEFT_B) == digitalReadFast(ENC_LEFT_A))
  {
    leftCount++;
  }
  else
  {
    leftCount--;
  }
}

void rightEncoderISR()
{
  if (digitalReadFast(ENC_RIGHT_B) == digitalReadFast(ENC_RIGHT_A))
  {
    rightCount++;
  }
  else
  {
    rightCount--;
  }
}

void initPixy()
{
  pixy.init();
  pixy.changeProg("line");
}

void steer(int angle)
{
  angle = constrain(angle, MAX_LEFT, MAX_RIGHT);
  steer_servo.write(angle);
}

void run(int speedLeft, int speedRight)
{
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);

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

void onReceive(int howMany)
{
  if (howMany < 3)
  {
    while (Wire1.available() > 0)
    {
      Wire1.read();
    }
    return;
  }

  const char cmd = static_cast<char>(Wire1.read());
  const uint8_t val1 = Wire1.read();
  const uint8_t val2 = Wire1.read();

  if (cmd == 's')
  {
    i2c_speed_left = map(val1, 0, 255, -255, 255);
    i2c_speed_right = map(val2, 0, 255, -255, 255);
    i2c_new_command = true;
  }
  else if (cmd == 't')
  {
    i2c_steering_angle = map(val1, 0, 255, MAX_LEFT, MAX_RIGHT);
    i2c_new_command = true;
  }

  while (Wire1.available() > 0)
  {
    Wire1.read();
  }
}

void communication_setup()
{
  Wire1.begin(I2C_SLAVE_ADDR);
  Wire1.onReceive(onReceive);
}

void communication_loop()
{
  if (!i2c_new_command)
  {
    return;
  }

  Serial.printf("Received:\n Speed Left: %d Right: %d Angle: %d\n",
                i2c_speed_left,
                i2c_speed_right,
                i2c_steering_angle);
  run(i2c_speed_left, i2c_speed_right);
  steer(i2c_steering_angle);
  i2c_new_command = false;
}

float computeAngle(const Vector &vec)
{
  float res = atan2(vec.m_y1 - vec.m_y0, vec.m_x1 - vec.m_x0) * 180.0 / PI;
  while (res < 0)
  {
    res += 360;
  }
  while (res >= 360)
  {
    res -= 360;
  }
  return res;
}

float computeLength(const Vector &vec)
{
  return sqrt(pow(vec.m_x1 - vec.m_x0, 2) + pow(vec.m_y1 - vec.m_y0, 2));
}

float normalizeAngle(float angle)
{
  while (angle < 0.0)
  {
    angle += 360.0;
  }
  while (angle >= 360.0)
  {
    angle -= 360.0;
  }
  return angle;
}

bool isHorizontalAngle(float angle)
{
  angle = normalizeAngle(angle);
  return (angle <= PIXY_STEER_DEADBAND || angle >= 360.0 - PIXY_STEER_DEADBAND) || (fabs(angle - 180.0) <= PIXY_STEER_DEADBAND);
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax)
{
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float angleDelta(float from, float to)
{
  float delta = normalizeAngle(to) - normalizeAngle(from);
  while (delta <= -180.0)
  {
    delta += 360.0;
  }
  while (delta > 180.0)
  {
    delta -= 360.0;
  }
  return delta;
}

float computeSteeringAngle(const Vector *vectors, uint8_t numVectors)
{
  if (numVectors == 0)
  {
    return lastSteeringAngle;
  }

  float weightedDelta = 0.0;
  float totalWeight = 0.0;

  for (uint8_t i = 0; i < numVectors; i++)
  {
    float angle = computeAngle(vectors[i]);
    float length = computeLength(vectors[i]);
    float delta = angleDelta(PIXY_FORWARD_ANGLE, angle);

    if (fabs(delta) > 90.0)
    {
      delta = angleDelta(PIXY_FORWARD_ANGLE, normalizeAngle(angle + 180.0));
    }

    float weight = max(length, 1.0f);
    weightedDelta += delta * weight;
    totalWeight += weight;
  }

  float avgDelta = totalWeight > 0.0 ? weightedDelta / totalWeight : 0.0;
  avgDelta = constrain(avgDelta, -45.0, 45.0);
  return mapFloat(avgDelta, -45.0, 45.0, MAX_LEFT, MAX_RIGHT);
}

float biasSteeringTowardCenter(float steeringAngle)
{
  if (steeringAngle > CENTRE_ANGLE)
  {
    return max(CENTRE_ANGLE, steeringAngle - STEERING_CENTER_BIAS);
  }
  if (steeringAngle < CENTRE_ANGLE)
  {
    return min(CENTRE_ANGLE, steeringAngle + STEERING_CENTER_BIAS);
  }
  return steeringAngle;
}

float softenExtremeSteering(float steeringAngle)
{
  float delta = steeringAngle - CENTRE_ANGLE;
  float absDelta = fabs(delta);
  if (absDelta <= STEERING_SOFT_LIMIT_START)
  {
    return steeringAngle;
  }

  float sign = delta > 0 ? 1.0f : -1.0f;
  float extra = absDelta - STEERING_SOFT_LIMIT_START;
  float softenedExtra = extra * STEERING_SOFT_LIMIT_FACTOR;
  return CENTRE_ANGLE + sign * (STEERING_SOFT_LIMIT_START + softenedExtra);
}

void sortVectorsByLength(Vector *vectors, uint8_t numVectors)
{
  for (uint8_t i = 0; i < numVectors - 1; i++)
  {
    for (uint8_t j = 0; j < numVectors - i - 1; j++)
    {
      float length1 = computeLength(vectors[j]);
      float length2 = computeLength(vectors[j + 1]);
      if (length1 < length2)
      {
        Vector temp = vectors[j];
        vectors[j] = vectors[j + 1];
        vectors[j + 1] = temp;
      }
    }
  }
}

void printSortedVectors(Vector *vectors, uint8_t numVectors)
{
  for (uint8_t i = 0; i < numVectors; i++)
  {
    float angle = computeAngle(vectors[i]);
    float length = computeLength(vectors[i]);
    int x0 = vectors[i].m_x0;
    int y0 = vectors[i].m_y0;
    int x1 = vectors[i].m_x1;
    int y1 = vectors[i].m_y1;
    Serial.printf("Sorted Vector %d: Angle=%.2f, Length=%.2f, x0=%d, y0=%d, x1=%d, y1=%d\n", i, angle, length, x0, y0, x1, y1);
  }
  Serial.println("---------------------------------------------------------------");
}

float applyCorrection(const Vector *vectors, uint8_t numVectors, float targetSteering)
{
 if (numVectors == 1)
  {
    float mainVectorCenterX = (vectors[0].m_x0 + vectors[0].m_x1) / 2.0f; // (x0+x1)/2 mte3 atwel vecteur
    if (fabs(targetSteering - CENTRE_ANGLE) <= 10.0f && mainVectorCenterX >= 20.0f && mainVectorCenterX <= 60.0f)
    {
      float correction = 0.0f;
      if (mainVectorCenterX >= 20.0f && mainVectorCenterX < 40.0f)
      {
        // Small turn to the right
        correction = 15.0f;
        //Serial.printf("Small turn RIGHT: mainVectorCenterX=%.1f\n", mainVectorCenterX);
      }
      else if (mainVectorCenterX >= 40.0f && mainVectorCenterX <= 60.0f)
      {
        // Small turn to the left
        correction = -15.0f;
        //Serial.printf("Small turn LEFT: mainVectorCenterX=%.1f\n", mainVectorCenterX);
      }

      targetSteering = constrain(targetSteering + correction, MAX_LEFT, MAX_RIGHT);
    }
  }
  return targetSteering;
}

Vector sumVector(const Vector &v1, const Vector &v2)
{
  Vector result;
  result.m_x0 = v1.m_x0 + v2.m_x0;
  result.m_y0 = v1.m_y0 + v2.m_y0;
  result.m_x1 = v1.m_x1 + v2.m_x1;
  result.m_y1 = v1.m_y1 + v2.m_y1;
  return result;
}

float dotProduct(const Vector &v1, const Vector &v2)
{
  return (v1.m_x1 - v1.m_x0) * (v2.m_x1 - v2.m_x0) + (v1.m_y1 - v1.m_y0) * (v2.m_y1 - v2.m_y0);
}

float computeAngleBetweenVectors(const Vector &v1, const Vector &v2)
{
  float mag1 = computeLength(v1);
  float mag2 = computeLength(v2);
  if (mag1 == 0 || mag2 == 0) return 0;
  float dot = dotProduct(v1, v2);
  float cosTheta = dot / (mag1 * mag2);
  cosTheta = constrain(cosTheta, -1.0f, 1.0f);
  return acos(cosTheta) * 180.0 / PI;
}

int maxX0 = 0;
int maxY0 = 0;
int maxX1 = 0;
int maxY1 = 0;
void findMaxXandMaxY(){
  pixy.line.getAllFeatures();
  for(int i=0; i<pixy.line.numVectors; i++){
    if(pixy.line.vectors[i].m_x0 > maxX0){
      maxX0 = pixy.line.vectors[i].m_x0;
    }
    if(pixy.line.vectors[i].m_y0 > maxY0){
      maxY0 = pixy.line.vectors[i].m_y0;
    }
    if(pixy.line.vectors[i].m_x1 > maxX1){
      maxX1 = pixy.line.vectors[i].m_x1;
    }
    if(pixy.line.vectors[i].m_y1 > maxY1){
      maxY1 = pixy.line.vectors[i].m_y1;
    }
  }
  Serial.printf("Max X0: %d, Max Y0: %d, Max X1: %d, Max Y1: %d\n", maxX0, maxY0, maxX1, maxY1);
}


void stopBox(){
  // Stop distance need some tuning bech tji bel dhabt
  #define STOP_DISTANCE 30
  long distance = readDistanceFiltered();
  //Serial.printf("Distance: %ld\n", distance);
  if(distance < STOP_DISTANCE){
    run(0,0);
    //Serial.println("Object detected, stopping");
    while(true);
  }
}

bool finishLineDetected = false;

struct Point { float x, y; };

int identifyCase(Vector v1, Vector v2) {
    Point p1a = {(float)v1.m_x0, (float)v1.m_y0};
    Point p1b = {(float)v1.m_x1, (float)v1.m_y1};
    Point p2a = {(float)v2.m_x0, (float)v2.m_y0};
    Point p2b = {(float)v2.m_x1, (float)v2.m_y1};

    auto distSq = [](Point a, Point b) { 
        return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2); 
    };

    // Find which two ends are the corner
    float d00 = distSq(p1a, p2a);
    float d01 = distSq(p1a, p2b);
    float d10 = distSq(p1b, p2a);
    float d11 = distSq(p1b, p2b);

    Point corner, far1, far2;
    float minDist = std::min({d00, d01, d10, d11});

    if (minDist == d00) { corner = p1a; far1 = p1b; far2 = p2b; }
    else if (minDist == d01) { corner = p1a; far1 = p1b; far2 = p2a; }
    else if (minDist == d10) { corner = p1b; far1 = p1a; far2 = p2b; }
    else { corner = p1b; far1 = p1a; far2 = p2a; }

    // Create vectors pointing AWAY from the corner
    float dir1x = far1.x - corner.x;
    float dir1y = far1.y - corner.y;
    float dir2x = far2.x - corner.x;
    float dir2y = far2.y - corner.y;

    // Normalize them to give them equal weight
    float mag1 = std::sqrt(dir1x*dir1x + dir1y*dir1y);
    float mag2 = std::sqrt(dir2x*dir2x + dir2y*dir2y);
    
    if (mag1 < 0.1f || mag2 < 0.1f) return 0; // Avoid division by zero

    // Resultant vector points into the "open" part of the corner
    float resX = (dir1x / mag1) + (dir2x / mag2);
    float resY = (dir1y / mag1) + (dir2y / mag2);

    #define TOP_LEFT_CORNER 1
    #define TOP_RIGHT_CORNER 2
    #define BOTTOM_LEFT_CORNER 3
    #define BOTTOM_RIGHT_CORNER 4
    if (resX < 0 && resY < 0) return TOP_LEFT_CORNER; // TOP LEFT CORNER
    if (resX > 0 && resY < 0) return TOP_RIGHT_CORNER; // TOP RIGHT CORNER
    if (resX < 0 && resY > 0) return BOTTOM_LEFT_CORNER; // BOTTOM LEFT CORNER
    if (resX > 0 && resY > 0) return BOTTOM_RIGHT_CORNER; // BOTTOM RIGHT CORNER

    return 0;
}

void firasLogic(){
  Vector vectors[MAX_VECTORS];
  uint8_t numVectors = 0;
  Vector horizontalVectors[MAX_VECTORS];
  uint8_t numHorizontalVectors = 0;
  pixy.line.getAllFeatures();
  //Serial.printf("Detected %d vectors\n", pixy.line.numVectors);
  for (uint8_t i = 0; i < pixy.line.numVectors; i++)
  {
    float angle = computeAngle(pixy.line.vectors[i]);
    float length = computeLength(pixy.line.vectors[i]);
    int x0 = pixy.line.vectors[i].m_x0;
    int y0 = pixy.line.vectors[i].m_y0;
    int x1 = pixy.line.vectors[i].m_x1;
    int y1 = pixy.line.vectors[i].m_y1;
    float avgY = (y0 + y1) / 2.0f;

    // FINISH LINE DETECTION (need some tuning) First part
    if(isHorizontalAngle(angle) && length < 15.0f){
      horizontalVectors[numHorizontalVectors] = pixy.line.vectors[i];
      numHorizontalVectors++;
      //Serial.printf("Horizontal Vector %d: Angle=%.2f, Length=%.2f, x0=%d, y0=%d, x1=%d, y1=%d\n", i, angle, length, x0, y0, x1, y1);
    }

    if (numVectors < MAX_VECTORS
        // Ignore vectors that are too short
        && length > MIN_VECTOR_LENGTH
        // Ignore vectors that are too horizontal
        && !isHorizontalAngle(angle)
        // Ignore vectors that are too close to the top or bottom of the frame
        && !((avgY >= 0 && avgY <= 2) || (avgY >= 48 && avgY <= 50)))
    {
      vectors[numVectors] = pixy.line.vectors[i];
      numVectors++;
    }
  }

  // FINISH LINE DETECTION (need some tuning) Second part
  unsigned long elapsedTime = millis() - startTime;
  if(numHorizontalVectors >= 2 && !finishLineDetected && elapsedTime > 10000){
    sortVectorsByLength(horizontalVectors, numHorizontalVectors);
    float avgY_vec1 = (horizontalVectors[0].m_y0 + horizontalVectors[0].m_y1) / 2.0f;
    float avgY_vec2 = (horizontalVectors[1].m_y0 + horizontalVectors[1].m_y1) / 2.0f;
    // if les Y mte3hom ykounou 9ribin mel baadhhom (within 5 pixels), n3etberou finish line detected, w nkamlo douga douga
    if(fabs(avgY_vec1 - avgY_vec2) <= 5.0f){
      Serial.println("Finish line detected!");
      //run(130,130);
      finishLineDetected = true;
    }
  }

  // Sort vectors by length in descending order
  sortVectorsByLength(vectors, numVectors);

  float targetSteering = lastSteeringAngle;

  if (numVectors == 0)
  {
    targetSteering = lastSteeringAngle;
  }
  else if (numVectors == 1)
  {
    targetSteering = computeSteeringAngle(vectors, numVectors);
  }
  else if (numVectors >= 2)
  {
    // kana fama angle 90 binet les deux vecteurs, bech nwaslou l goddam (most likely fama intersection)
    float angleBetweenVectors = computeAngleBetweenVectors(vectors[0], vectors[1]);
    if ((abs(angleBetweenVectors) > 70 && abs(angleBetweenVectors) < 110))
    {
      float avg_x = (vectors[0].m_x0 + vectors[0].m_x1 + vectors[1].m_x0 + vectors[1].m_x1) / 4.0f;
      #define CROSS_CORRECTION_ANGLE 15.0f
      float correction_based_on_side = (avg_x < 40.0f) ? CROSS_CORRECTION_ANGLE : ((avg_x > 40.0f) ? - CROSS_CORRECTION_ANGLE : 0.0f); // nal3bou 3la el valeur 15???
      targetSteering = CENTRE_ANGLE + correction_based_on_side;
      int caseIntersection = identifyCase(vectors[0], vectors[1]);
      if(caseIntersection == TOP_LEFT_CORNER){
        Serial.println("Case: TOP LEFT CORNER");
      }
      else if(caseIntersection == TOP_RIGHT_CORNER){
        Serial.println("Case: TOP RIGHT CORNER");
      }
      else if(caseIntersection == BOTTOM_LEFT_CORNER){
        Serial.println("Case: BOTTOM LEFT CORNER");
      }
      else if(caseIntersection == BOTTOM_RIGHT_CORNER){
        Serial.println("Case: BOTTOM RIGHT CORNER");
      }
    }
    // mafamch intersection
    else
    {
      targetSteering = computeSteeringAngle(vectors, numVectors);
    }
  }

  targetSteering = applyCorrection(vectors, numVectors, targetSteering);
  targetSteering = biasSteeringTowardCenter(targetSteering);
  targetSteering = softenExtremeSteering(targetSteering);

  // if the new target steering is significantly different from the last one, queue it
  if (fabs(targetSteering - pendingSteeringAngle) > STEERING_QUEUE_THRESHOLD)
  {
    pendingSteeringAngle = targetSteering;
    steeringDelayCounter = STEERING_DELAY_CYCLES;
  }

  // decrement the delay if we have any
  if (steeringDelayCounter > 0){ steeringDelayCounter--; }

  // if the delay has elapsed and we have a pending steering angle different from the last applied one, apply it
  if (steeringDelayCounter == 0 && pendingSteeringAngle != lastSteeringAngle)
  {
    steer(pendingSteeringAngle);
    lastSteeringAngle = pendingSteeringAngle;
  }

  // Finish line detected, check el box
  if(finishLineDetected){
    stopBox();
  }
}


void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial monitor ready at 115200");

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

  //run(150, 150);
  run(0, 0);
  startTime = millis();
}
void loop()
{
  firasLogic();
}