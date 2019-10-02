#include <Servo.h>
#include <AFMotor.h>

/** Servo **/
#include <Servo.h>
Servo forwardServo;
int servoPos = 0;
int servoPosPin = 26;
const unsigned long SWEEP_DELAY = 201;
unsigned long timeOfLastSweep = 0;
boolean sweepDirection = true;

unsigned long timeSinceLastForwardCheck = 0;
unsigned int FOR_CHECK_DELAY = 1000;

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
const int MAX_BACKUP_TIME = 300;
const int MAX_ALIGNING_TIME = 1200; // 800 is ~180 degrees
const int MIN_ALIGNING_TIME = 200;
const int SENSOR_MAX_TIMEOUT = 500;
const int EDGE_DET_THRES = 40; //how close does the table have to be to be considered a table (cm)

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
  randomSeed(analogRead(randomSeedPin));

  forwardServo.attach(servoPosPin);
  servoPos = 90;
  timeOfLastSweep = millis();
  forwardServo.write(0);
  delay(15);
  forwardServo.write(servoPos);
}

void loop()
{

  long loopStart = millis(); // useful for timing of various processes in run loop

  SensorResult resultRight = getSensorResult(sensorRightTriggerPin, sensorRightEchoPin, SENSOR_MAX_TIMEOUT, EDGE_DET_THRES);
  SensorResult resultLeft = getSensorResult(sensorLeftTriggerPin, sensorLeftEchoPin, SENSOR_MAX_TIMEOUT, EDGE_DET_THRES);
  SensorResult resultForward = getSensorResult(sensorForwardTriggerPin, sensorForwardEchoPin, SENSOR_MAX_TIMEOUT, 10);

  if (resultForward == SensorResult::Object) {
    Serial.println("Object detected by servo sensor");
  }

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
  if (resultForward == SensorResult::Error) {
    Serial.println("Forward sensor not working!");
    runningMode = Mode::Error;
  }

  if (resultRight == SensorResult::Nothing)
  {
    clearTurnDirection = Direction::Right;
  }
  else if (resultLeft == SensorResult::Nothing)
  {
    clearTurnDirection = Direction::Left;
  }

  runningMode = getNewModeBasedOnSensorResults(runningMode, resultRight, resultLeft, resultForward);

  // Main switch case for determining what behaviour the robot should be doing, based on the current mode
  switch (runningMode)
  {
  case Mode::EmergencyBackup:
  { // reverse the robot for a certain amount of time, to ensure it is not going to go over the edge when it turns
    backupTimeElapsed += millis() - loopStart;

    if (backupTimeElapsed < MAX_BACKUP_TIME)
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
      runningMode = Mode::CheckForwardForObjects;
      Serial.println("CheckForwardForObjects: Beginning Check");
    }
  }
  break;
  case Mode::CheckForwardForObjects: 
  {
    stop();

    // check object left
    forwardServo.write(20);
    delay(SWEEP_DELAY);
    SensorResult resultLeftForward = getSensorResult(
      sensorForwardTriggerPin, 
      sensorForwardEchoPin, 
      SENSOR_MAX_TIMEOUT,
      32);

    // left of centre
    forwardServo.write(55);
    delay(SWEEP_DELAY);
    SensorResult resultLeftCentreForward = getSensorResult(
      sensorForwardTriggerPin, 
      sensorForwardEchoPin, 
      SENSOR_MAX_TIMEOUT,
      32);

    // check object center
    forwardServo.write(90);
    delay(SWEEP_DELAY);
    SensorResult resultCentreForward = getSensorResult(
      sensorForwardTriggerPin, 
      sensorForwardEchoPin, 
      SENSOR_MAX_TIMEOUT,
      40);

    // right of centre
    forwardServo.write(125);
    delay(SWEEP_DELAY);
    SensorResult resultRightCentreForward = getSensorResult(
      sensorForwardTriggerPin, 
      sensorForwardEchoPin, 
      SENSOR_MAX_TIMEOUT,
      32);

    // check object right
    forwardServo.write(160);
    delay(SWEEP_DELAY);
    SensorResult resultRightForward = getSensorResult(
      sensorForwardTriggerPin, 
      sensorForwardEchoPin, 
      SENSOR_MAX_TIMEOUT,
      32);


      if (resultLeftForward == SensorResult::Nothing && 
         resultLeftCentreForward == SensorResult::Nothing &&
         resultRightForward == SensorResult::Nothing &&
         resultRightCentreForward == SensorResult::Nothing &&
         resultRightForward == SensorResult::Nothing) {

        Serial.println("CheckForwardForObjects: Forward path clear, starting to move forward");
        runningMode = Mode::MovingStraight;
        Serial.println("MovingStraight: Starting to move forward");

      } else {

        Serial.println("CheckForwardForObjects: Forward path unclear, restarting search for clear path");
        runningMode = Mode::FindingClearPath;
        Serial.println("FindingClearPath: Restarting...");

      }
      timeSinceLastForwardCheck = millis();
      forwardServo.write(20);
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

void sweep(long loopStart) {

  if (millis() - timeOfLastSweep >= SWEEP_DELAY) {
    timeOfLastSweep = millis();
    if (servoPos >= 180) {
      sweepDirection = false;
    } else if (servoPos <= 0) {
      sweepDirection = true;
    }

    if (sweepDirection) {
      servoPos += 1;
      forwardServo.write(servoPos);
    } else {
      servoPos -= 1;
      forwardServo.write(servoPos);
    }

  }
}



SensorResult getSensorResult(int sonicPulsePin, int echoInputPin, int maxTimeout, int distanceThreshold)
{
  triggerSonicPulse(sonicPulsePin);
  long timeSpentHigh = waitForSignalUntilTimeout(echoInputPin, HIGH, 1000);

  if (timeSpentHigh >= maxTimeout)
  {
    return SensorResult::Error; //sensor not responding to trigger pulse
  }

  long echoStart = micros();
  long timeSpentLow = waitForSignalUntilTimeout(echoInputPin, LOW, 1000);

  if (timeSpentHigh + timeSpentLow >= maxTimeout)
  {
    return SensorResult::Error; //sensor did not send echo signal back
  }

  long echoTime = micros() - echoStart;
  int distance = echoTime / 58; //centimeters

  if (distance > distanceThreshold)
  {
    return SensorResult::Nothing;
  }
  else
  {
    return SensorResult::Object;
  }
}

// trigger 10us voltage pulse so sensor knows to start sound pulse
void triggerSonicPulse(int triggerP)
{
  digitalWrite(triggerP, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerP, LOW);
}

// busy wait on a signal until it changes
long waitForSignalUntilTimeout(int pin, int level, int maxTimeout)
{
  long startTime = millis();
  long elapsedTime = 0;
  while (digitalRead(pin) != level)
  {
    elapsedTime = millis() - startTime;
    if (elapsedTime >= maxTimeout)
    {
      break;
    }
  }
  return elapsedTime;
}

// use the previousMode and the sensor results to figure out what new Mode the robot should be in
Mode getNewModeBasedOnSensorResults(Mode previousMode, SensorResult resultRight, SensorResult resultLeft, SensorResult resultForward)
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
  }
  break;
  case Mode::AligningWithClearPath:
  {
    // We don't care about the sensor data during this mode
  }
  break;
  case Mode::MovingStraight:
  { // if we're currently moving forward, and an edge is detected, initiate an emergency backup and store the direction the edge was detected from
    if (resultRight == SensorResult::Nothing)
    {
      Serial.println("MovingStraight: Right sensor has detected edge, initiating emergency backup");
      return Mode::EmergencyBackup;
    }
    else if (resultLeft == SensorResult::Nothing)
    {
      Serial.println("MovingStraight: Left sensor has detected edge, initiating emergency backup");
      return Mode::EmergencyBackup;
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
