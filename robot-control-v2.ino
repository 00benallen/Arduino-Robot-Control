#include <AFMotor.h>

/**
   Motor constants
*/
const int MOTOR_SPEED_LOW = 120;
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
const int MAX_BACKUP_TIME = 150;
const int MAX_ALIGNING_TIME = 800;
const int SENSOR_MAX_TIMEOUT = 500;
const int EDGE_DET_THRES = 5; //how close does the table have to be to be considered a table (cm)

/**
   Enum representing possible modes the robot can be in during the run loop
*/
enum class Mode
{
  MovingStraight,        // robot is moving forward along a clear path
  FindingClearPath,      // robot is searching for a clear path
  AligningWithClearPath, // robot is aligning itself with a found clear path
  EmergencyBackup,       // robot is backing up (reversing) because an obstacle is directly in front
  Error,                 // robot is in an error state, not ready/not safe to drive
};

/**
   Enum representing the possible results the sensor data could indicate
*/
enum class SensorResult
{
  Edge,
  Table,
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
Direction lastDetectedEdgeDirection = Direction::Left;
Mode runningMode = Mode::FindingClearPath;

// state that changes between loops
int backupTimeElapsed = 0;   //how long have we been backing up in ms
int aligningTimeElapsed = 0; //how long have we been aligning with clear path in ms

// pins
int sensorRightTriggerPin = 53;
int sensorRightEchoPin = 52;
int sensorLeftTriggerPin = 23;
int sensorLeftEchoPin = 22;

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
  pinMode(sensorRightEchoPin, INPUT);
  pinMode(sensorLeftEchoPin, INPUT);
}

void loop()
{

  long loopStart = millis(); // useful for timing of various processes in run loop

  SensorResult resultRight = getSensorResult(sensorRightTriggerPin, sensorRightEchoPin, SENSOR_MAX_TIMEOUT);
  SensorResult resultLeft = getSensorResult(sensorLeftTriggerPin, sensorLeftEchoPin, SENSOR_MAX_TIMEOUT);

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

  if (resultRight == SensorResult::Edge)
  {
    lastDetectedEdgeDirection = Direction::Right;
  }
  else if (resultLeft == SensorResult::Edge)
  {
    lastDetectedEdgeDirection = Direction::Left;
  }

  runningMode = getNewModeBasedOnSensorResults(runningMode, resultRight, resultLeft);

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
      if (lastDetectedEdgeDirection == Direction::Right)
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
    if (lastDetectedEdgeDirection == Direction::Right)
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
      if (lastDetectedEdgeDirection == Direction::Right)
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
      Serial.println("AligningWithClearPath: Clear path alignment finished, starting to move forward");
      aligningTimeElapsed = 0;
      runningMode = Mode::MovingStraight;
      Serial.println("MovingStraight: Moving forward");
    }
  }
  break;
  case Mode::MovingStraight:
  { // way forward is clear, so go that way
    forward();
  }
  break;
  case Mode::Error:
  { // something is wrong with Arduino or its peripheral devices, stop to keep the robot safe
    Serial.println("Error: Issue with setup, robot not safe to move!");
    stop();
  }
  break;
  }
  // forward();
}

SensorResult getSensorResult(int sonicPulsePin, int echoInputPin, int maxTimeout)
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

  if (distance > EDGE_DET_THRES)
  {
    return SensorResult::Edge;
  }
  else
  {
    return SensorResult::Table;
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
Mode getNewModeBasedOnSensorResults(Mode previousMode, SensorResult resultRight, SensorResult resultLeft)
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
    if (resultRight == SensorResult::Table && resultRight == SensorResult::Table)
    { // both sensors are now seeing table
      Serial.println("FindingClearPath: Both sensors see table, turning more to align");
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
    if (resultRight == SensorResult::Edge)
    {
      Serial.println("MovingStraight: Right sensor has detected edge, initiating emergency backup");
      return Mode::EmergencyBackup;
    }
    else if (resultLeft == SensorResult::Edge)
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
