#include <Servo.h>
#include <AFMotor.h>

/** Servo **/
#include <Servo.h>
Servo forwardServo;
int servoPos = 0;
int servoPosPin = 10;
const unsigned long SWEEP_DELAY = 200;
unsigned long timeOfLastSweep = 0;
boolean sweepDirection = true;
unsigned int angles[] = { 20, 55, 90, 125, 160 };
unsigned int forwardDetThreshold[] = { 20, 24, 28, 24, 20 };

unsigned long timeSinceLastForwardCheck = 0;
unsigned int FOR_CHECK_DELAY = 200;

/**
   Motor constants
*/
const int MOTOR_SPEED_LOW = 180;
const int MOTOR_SPEED_HIGH = 255;

/**
   Setup motors
*/
AF_DCMotor motorFrontRight(3);
AF_DCMotor motorBackRight(2);
AF_DCMotor motorFrontLeft(4);
AF_DCMotor motorBackLeft(1);

/**
   Other constants
*/
const int MAX_BACKUP_TIME_MAX = 250;
const int MAX_BACKUP_TIME_MIN = 100;
int backupTime = MAX_BACKUP_TIME_MAX;
const int MAX_ALIGNING_TIME = 700; // 800 is ~180 degrees
const int MIN_ALIGNING_TIME = 200;
const unsigned long SENSOR_MAX_TIMEOUT = 10000;
const int EDGE_DET_THRES = 40; //how close does the table have to be to be considered a table (cm)
const int VCC = 5;
const int R_DIV = 1023;
const float STRAIGHT_RESISTANCE = 273.0; // resistance when straight
const float BEND_RESISTANCE = 500.0; // resistance at 90 deg
float restingAngle;


/**
   Enum representing possible modes the robot can be in during the run loop
*/
enum class Mode
{
  MovingStraight,        // robot is moving forward along a clear path
  FindingClearPath,      // robot is searching for a clear path
  AligningWithClearPath, // robot is aligning itself with a found clear path
  EmergencyBackup,       // robot is backing up (reversing) because an obstacle is directly in front
  CheckForwardForObjects,
  Error,                 // robot is in an error state, not ready/not safe to drive
};

/**
   Enum representing the possible results the sensor data could indicate
*/
enum class SensorResult
{
  Object,
  Nothing,
  Error,
};

/**
   Enum representing the two directions the robot cares about for edge-avoidance purposes
*/
enum class Direction
{
  Left,
  Right
};

/**
   Global variables that change
*/
// state that persists across loops
int motorSpeed = MOTOR_SPEED_LOW;
Direction clearTurnDirection = Direction::Left;
Mode runningMode = Mode::FindingClearPath;
unsigned long tableDistances[5];

// state that changes between loops
int backupTimeElapsed = 0;   //how long have we been backing up in ms
int aligningTimeElapsed = 0; //how long have we been aligning with clear path in ms
int currentAligningValue = MAX_ALIGNING_TIME;

// pins
int sensorRightTriggerPin = 53;
int sensorRightEchoPin = 52;
int sensorLeftTriggerPin = 23;
int sensorLeftEchoPin = 22;
int sensorForwardTriggerPin = 50;
int sensorForwardEchoPin = 51;
int randomSeedPin = 15;
int flexPin = A8;

void setup()
{

  Serial.begin(9800);

  // setup motors
  motorFrontRight.setSpeed(motorSpeed);
  motorFrontRight.run(RELEASE);
  motorBackRight.setSpeed(motorSpeed);
  motorBackRight.run(RELEASE);
  motorFrontLeft.setSpeed(motorSpeed);
  motorFrontLeft.run(RELEASE);
  motorBackLeft.setSpeed(motorSpeed);
  motorBackLeft.run(RELEASE);

  // setup pins
  pinMode(sensorRightTriggerPin, OUTPUT);
  pinMode(sensorLeftTriggerPin, OUTPUT);
  pinMode(sensorForwardTriggerPin, OUTPUT);
  pinMode(sensorRightEchoPin, INPUT);
  pinMode(sensorLeftEchoPin, INPUT);
  pinMode(sensorForwardEchoPin, INPUT);
  pinMode(flexPin, INPUT);
  randomSeed(analogRead(randomSeedPin));

  forwardServo.attach(servoPosPin);
  servoPos = 90;
  timeOfLastSweep = millis();
  forwardServo.write(90);
  delay(4000);
  forwardServo.write(servoPos);

  Serial.println("Calibrating bump sensor...");
  float bumpSensorReadingSum = 0.0;
  for (int i = 0; i < 10; i++) {

    bumpSensorReadingSum += bumpSensorReading();

  }
  restingAngle = bumpSensorReadingSum / 10;
  Serial.print("Calibrated resting angle: "); Serial.println(restingAngle);

}

float bumpSensorReading() {
  // Read the ADC, and calculate voltage and resistance from it
  int flexADC = analogRead(flexPin);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
  return angle;
}

void loop()
{
  float angle = bumpSensorReading();

  SensorResult resultBump = SensorResult::Nothing;

  if (angle > restingAngle && angle - restingAngle > 5) {
    resultBump = SensorResult::Object;
  }

  long loopStart = millis(); // useful for timing of various processes in run loop

  SensorResult resultRight = getSensorResult(sensorRightTriggerPin, sensorRightEchoPin, SENSOR_MAX_TIMEOUT, EDGE_DET_THRES);
  SensorResult resultLeft = getSensorResult(sensorLeftTriggerPin, sensorLeftEchoPin, SENSOR_MAX_TIMEOUT, EDGE_DET_THRES);

  if (resultLeft == SensorResult::Error)
  {
    Serial.println("Left sensor not working!");
    runningMode = Mode::Error;
  }
  if (resultRight == SensorResult::Error)
  {
    Serial.println("Right sensor not working!");
    runningMode = Mode::Error;
  }

  if (resultLeft == SensorResult::Nothing) {
    clearTurnDirection = Direction::Left;
  }
  if (resultRight == SensorResult::Nothing) {
    clearTurnDirection = Direction::Right;
  }

  runningMode = getNewModeBasedOnSensorResults(runningMode, resultRight, resultLeft, resultBump);

  // Main switch case for determining what behaviour the robot should be doing, based on the current mode
  switch (runningMode)
  {
    case Mode::EmergencyBackup:
      { // reverse the robot for a certain amount of time, to ensure it is not going to go over the edge when it turns
        backupTimeElapsed += millis() - loopStart;

        if (backupTimeElapsed < backupTime)
        {
          backward();
        }
        else
        {
          Serial.print("EmergencyBackup: Emergency backup finished, beginning search for clear path in direction: ");
          if (clearTurnDirection == Direction::Right)
          {
            Serial.println("left");
          }
          else
          {
            Serial.println("right");
          }

          backupTimeElapsed = 0;
          runningMode = Mode::FindingClearPath;
        }
      }
      break;
    case Mode::FindingClearPath:
      { // turn in a circle to find a clear way forward
        if (clearTurnDirection == Direction::Right)
        {
          turnLeft();
        }
        else
        {
          turnRight();
        }
      }
      break;
    case Mode::AligningWithClearPath:
      { // turn to align the robot with the currently detected clear path
        aligningTimeElapsed += millis() - loopStart;

        if (aligningTimeElapsed < MAX_ALIGNING_TIME)
        {
          if (clearTurnDirection == Direction::Right)
          {
            turnLeft();
          }
          else
          {
            turnRight();
          }
        }
        else
        {
          Serial.println("AligningWithClearPath: Clear path alignment finished, starting to find a forward path");
          aligningTimeElapsed = 0;
          runningMode = Mode::MovingStraight;
          Serial.println("MovingStraight: Starting forward");
        }
      }
      break;
    case Mode::CheckForwardForObjects:
      {
        stop();
        bool clearPath = true;
        for (int i = 0; i < 5; i++) {

          forwardServo.write(angles[i]);
          delay(SWEEP_DELAY);
          SensorResult result = getSensorResult(sensorForwardTriggerPin, sensorForwardEchoPin, SENSOR_MAX_TIMEOUT, forwardDetThreshold[i]);

          if (result == SensorResult::Object) {
            clearPath = false;
          }

        }

        if (!clearPath) {
          Serial.println("CheckForwardForObjects: Object detected forward");
          runningMode = Mode::EmergencyBackup;
          backupTime = MAX_BACKUP_TIME_MIN;
          Serial.println("EmergencyBackup: Starting object avoidance");
        } else {
          Serial.println("CheckForwardForObjects: No object detected forward");
          runningMode = Mode::MovingStraight;
          Serial.println("MovingStraight: Path clear, starting forward motion");
        }

        timeSinceLastForwardCheck = millis();
      }
      break;
    case Mode::MovingStraight:
      { // way forward is clear, so go that way
        forward();

        if (millis() - timeSinceLastForwardCheck > FOR_CHECK_DELAY) {
          Serial.println("MovingStraight: Forward path no longer guaranteed clear, rechecking");
          runningMode = Mode::CheckForwardForObjects;
          Serial.println("CheckForwardForObjects: Rechecking...");
        }
      }
      break;
    case Mode::Error:
      { // something is wrong with Arduino or its peripheral devices, stop to keep the robot safe
        Serial.println("Error: Issue with setup, robot not safe to move!");
        stop();
      }
      break;
  }
}


SensorResult getSensorResult(int sonicPulsePin, int echoInputPin, int maxTimeout, int distanceThreshold)
{
  unsigned long distance = getSensorDistance(sonicPulsePin, echoInputPin, maxTimeout);
  //Serial.println(distance);

  if (distance == 0) {
    return SensorResult::Error;
  }

  if (distance >= distanceThreshold) {
    return SensorResult::Nothing;
  } else {
    return SensorResult::Object;
  }

}

unsigned long getSensorDistance(int sonicPulsePin, int echoInputPin, int maxTimeout) {
  triggerSonicPulse(sonicPulsePin);
  unsigned long duration = pulseIn(echoInputPin, HIGH, maxTimeout);
  unsigned long cm = microsecondsToCentimeters(duration);
  return cm;

}

unsigned long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

// trigger 10us voltage pulse so sensor knows to start sound pulse
void triggerSonicPulse(int triggerP)
{
  digitalWrite(triggerP, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerP, HIGH);
  delayMicroseconds(2);  // Only 2uS NOT 10!
  digitalWrite(triggerP, LOW);
  delayMicroseconds(2);
}

// use the previousMode and the sensor results to figure out what new Mode the robot should be in
Mode getNewModeBasedOnSensorResults(Mode previousMode, SensorResult resultRight, SensorResult resultLeft, SensorResult resultBump)
{
  switch (previousMode)
  {
    case Mode::EmergencyBackup:
      {
        // We don't care about the sensor data during this mode
      }
      break;
    case Mode::FindingClearPath:
      { // if we're looking for a clear path and both sensors read table, then clear path has been found, start aligning
        if (resultRight == SensorResult::Object && resultRight == SensorResult::Object)
        { // both sensors are now seeing table
          currentAligningValue = random(MIN_ALIGNING_TIME, MAX_ALIGNING_TIME);
          Serial.print("FindingClearPath: Both sensors see table, turning: to align ");
          Serial.print(currentAligningValue);
          Serial.println(" millis.");

          return Mode::AligningWithClearPath;
        }
        if (resultBump == SensorResult::Object) {
          if (clearTurnDirection == Direction::Left) {
            clearTurnDirection = Direction::Right;
          } else {
            clearTurnDirection = Direction::Left;
          }
          aligningTimeElapsed = 0;
          runningMode = Mode::EmergencyBackup;
          backupTime = MAX_BACKUP_TIME_MAX;
        }
      }
      break;
    case Mode::AligningWithClearPath:
      {
        if (resultBump == SensorResult::Object) {
          aligningTimeElapsed = 0;
          runningMode = Mode::EmergencyBackup;
          backupTime = MAX_BACKUP_TIME_MAX;
        }
      }
      break;
    case Mode::MovingStraight:
      { // if we're currently moving forward, and an edge is detected, initiate an emergency backup and store the direction the edge was detected from
        if (resultRight == SensorResult::Nothing)
        {
          Serial.println("MovingStraight: Right sensor has detected edge, initiating emergency backup");
          return Mode::EmergencyBackup;
          backupTime = MAX_BACKUP_TIME_MIN;
        }
        if (resultLeft == SensorResult::Nothing)
        {
          Serial.println("MovingStraight: Left sensor has detected edge, initiating emergency backup");
          return Mode::EmergencyBackup;
          backupTime = MAX_BACKUP_TIME_MIN;
        }

        if (resultBump == SensorResult::Object) {
          Serial.println("MovingStraight: Bump sensor has detected object, initiating emergency backup");
          return Mode::EmergencyBackup;
          backupTime = MAX_BACKUP_TIME_MAX;
        }
      }
      break;
    case Mode::Error:
      { // if we're currently in an error state, but both sensors have stopped erroring, we are no longer in an error state, so start looking for a clear path
        if (resultRight != SensorResult::Error && resultLeft != SensorResult::Error)
        {
          return Mode::FindingClearPath;
        }
      }
      break;
  }
  return previousMode; // if no mode has been returned by any of the cases above, just return the previousMode
}

void forward()
{
  motorFrontRight.run(FORWARD);
  motorBackRight.run(FORWARD);
  motorFrontLeft.run(FORWARD);
  motorBackLeft.run(FORWARD);
}

void backward()
{
  motorFrontRight.run(BACKWARD);
  motorBackRight.run(BACKWARD);
  motorFrontLeft.run(BACKWARD);
  motorBackLeft.run(BACKWARD);
}

void turnRight()
{
  motorFrontRight.run(BACKWARD);
  motorBackRight.run(BACKWARD);
  motorFrontLeft.run(FORWARD);
  motorBackLeft.run(FORWARD);
}

void turnLeft()
{
  motorFrontRight.run(FORWARD);
  motorBackRight.run(FORWARD);
  motorFrontLeft.run(BACKWARD);
  motorBackLeft.run(BACKWARD);
}

void stop()
{
  motorFrontRight.run(RELEASE);
  motorBackRight.run(RELEASE);
  motorFrontLeft.run(RELEASE);
  motorBackLeft.run(RELEASE);
}
