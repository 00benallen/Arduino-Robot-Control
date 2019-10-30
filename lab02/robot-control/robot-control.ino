/** Servo **/
#include <Servo.h>
Servo forwardServo;
int servoPos = 0;
int servoPosPin = 10;
const unsigned long SWEEP_DELAY = 40;
unsigned long timeOfLastSweep = 0;
unsigned int servoAngle = 0;

unsigned long timeSinceLastForwardCheck = 0;
unsigned int FOR_CHECK_DELAY_NORMAL = 350;
unsigned int FOR_CHECK_DELAY_LINE = 500;

/** Motors **/

#include <AFMotor.h>
AF_DCMotor motorFrontRight(3);
AF_DCMotor motorBackRight(2);
AF_DCMotor motorFrontLeft(4);
AF_DCMotor motorBackLeft(1);

const int MOTOR_SPEED_LOW = 150;
const int MOTOR_SPEED_HIGH = 255;

int randomTurnAmount = 90;

/** Timing Constants **/
const int MAX_BACKUP_TIME_MAX = 300;
const int MAX_BACKUP_TIME_MIN = 100;
int backupTime = MAX_BACKUP_TIME_MAX;
const int MAX_ALIGNING_TIME = 1200;
const int MIN_ALIGNING_TIME = 200;

/** Sensors **/
#include "Sensors.h"
#include "Ultrasonic.h"
int sonicSensorDownTriggerPin = 26;
int sonicSensorDownEchoPin = 23;
int IRSensorLineLeftOutPin1 = 33;
int IRSensorLineRightOutPin1 = 53;
int IRSensorLineLeftOutPin2 = 35;
int IRSensorLineRightOutPin2 = 30;

const int EDGE_DET_THRES = 40;
const unsigned long SENSOR_MAX_TIMEOUT = 2915;

IRSensor forwardLineRightSensor1(IRSensorLineLeftOutPin1);
IRSensor forwardLineLeftSensor1(IRSensorLineRightOutPin1);
IRSensor forwardLineRightSensor2(IRSensorLineLeftOutPin2);
IRSensor forwardLineLeftSensor2(IRSensorLineRightOutPin2);

Ultrasonic forwardEdgeSensor(sonicSensorDownTriggerPin, sonicSensorDownEchoPin, SENSOR_MAX_TIMEOUT);

#include <VL53L0X.h>
VL53L0X forwardObjectSensor;

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
float lastDetectedHeading;

#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


/**
   Enum representing possible modes the robot can be in during the run loop
*/
enum class Mode
{
  MovingStraight,        // robot is moving forward along a clear path
  AligningWithClearPath, // robot is aligning itself with a found clear path
  EmergencyBackup,       // robot is backing up (reversing) because an obstacle is directly in front
  CheckForwardForObjects,
  FollowingLine,
  FindingLine,
  Error,                 // robot is in an error state, not ready/not safe to drive
};

/**
   Enum representing the two directions the robot cares about for edge-avoidance purposes
*/
enum class Direction
{
  Left,
  Right
};

/** Line Detection **/
Direction turningDirection = Direction::Left;
unsigned long timeElapsedLineForward = 0;
unsigned long timeElapsedLineTurn = 0;
bool followingLine = false;
bool lineDetected = false;
bool ignoreLine = false;
unsigned int lineTurnAmount = 0;
unsigned int lineSearchTurns = 0;
Direction lastLineSearchDirection = Direction::Left;

/**
   Global variables that change
*/
// state that persists across loops
int motorSpeed = MOTOR_SPEED_LOW;
Direction clearTurnDirection = Direction::Left;
Mode runningMode = Mode::FollowingLine;

// state that changes between loops
int backupTimeElapsed = 0;   //how long have we been backing up in ms
float aligningStartHeading;
int currentAligningValue = MAX_ALIGNING_TIME;

// auxilliary pins
int randomSeedPin = 15;

// Debug
bool motorsEnabled = false;

void setup()
{
  Serial.begin(9800);
  Serial.println("[CPS603-Robot Control Program] starting up");

  // setup motors
  Serial.println("Initializing motors");
  motorFrontRight.setSpeed(motorSpeed);
  motorFrontRight.run(RELEASE);
  motorBackRight.setSpeed(motorSpeed);
  motorBackRight.run(RELEASE);
  motorFrontLeft.setSpeed(motorSpeed);
  motorFrontLeft.run(RELEASE);
  motorBackLeft.setSpeed(motorSpeed);
  motorBackLeft.run(RELEASE);

  Serial.println("Initializing BNO055 9-axis IMU");
  while (!bno.begin())
  {
    Serial.println("No BNO055 detected");
    delay(1000);
  }
  Serial.println("BNO055 9-axis IMU detected and initialized");

  Serial.println("Initializing VL53L0X laser TOF sensor");
  Serial.println("Adafruit VL53L0X test");
  while (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1000);
  }
  Serial.println("VL53L0X laser TOF sensor detected");

  Serial.println("Initializing IR sensors");
  forwardLineRightSensor1.init();
  forwardLineLeftSensor1.init();
  forwardLineRightSensor2.init();
  forwardLineLeftSensor2.init();

  Serial.println("Initializing auxilliary pins");
  pinMode(randomSeedPin, INPUT);
  randomSeed(analogRead(randomSeedPin));

  Serial.println("Initializing forward servo");
  forwardServo.attach(servoPosPin);
  timeOfLastSweep = millis();
  forwardServo.write(90);

  Serial.println("Waiting before start...");
  delay(10000);

}

void loop()
{

  long loopStart = millis(); // useful for timing of various processes in run loop
  pollIMU();
  if (!ignoreLine) {
    pollLineSensor();
  }

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
            Serial.println("right");
          }
          else
          {
            Serial.println("left");
          }

          backupTimeElapsed = 0;
          Serial.println("EmergencyBackup: Complete, beginning turn to find clear path");
          aligningStartHeading = lastDetectedHeading;
          runningMode = Mode::AligningWithClearPath;
          randomTurnAmount = random(10, 90);

        }
      }
      break;
    case Mode::AligningWithClearPath:
      { // turn to align the robot with the currently detected clear path
        if (motorsEnabled) {

          if (followingLine && !ignoreLine) { // line detected
            // drive forward a certain amount
            aligningStartHeading = lastDetectedHeading;
            Serial.println("AligningWithClearPath: Line detected");
            runningMode = Mode::FollowingLine;
            break;
          }


          float amountTurned = 180 - abs(abs(lastDetectedHeading - aligningStartHeading) - 180); 
          if (amountTurned <= 90)
          {
            if (clearTurnDirection == Direction::Right)
            {
              turnRight();
            }
            else
            {
              turnLeft();
            }
          }
          else
          {
            Serial.println("AligningWithClearPath: Clear path alignment finished, starting to find a forward path");
            runningMode = Mode::CheckForwardForObjects;
            ignoreLine = false;
          }
        } else {
          Serial.println("AligningWithClearPath: Clear path alignment finished (motors disabled), starting to find a forward path");
          runningMode = Mode::CheckForwardForObjects;
          ignoreLine = false;
        }
      }
      break;
    case Mode::CheckForwardForObjects:
      {
        Serial.println("CheckForwardForObjects: Forward sweep starting, stopping robot until path forward is guaranteed clear");
        stop();
        servoAngle = 0;
        bool edgeDetected = false;
        while (servoAngle <= 180) {
          forwardServo.write(servoAngle);
          servoAngle += 5;

          unsigned long distanceCM = forwardEdgeSensor.read(CM);

          if (distanceCM > EDGE_DET_THRES) {
            if (servoAngle <= 90) {
              clearTurnDirection = Direction::Left; // turn the other way
              Serial.println("CheckForwardForObjects: Edge on right side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
            } else {
              Serial.println("CheckForwardForObjects: Edge on left side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
              clearTurnDirection = Direction::Right; // turn the other way
            }
            runningMode = Mode::EmergencyBackup;
            edgeDetected = true;
            ignoreLine = true;
            followingLine = false;
            break;
          }

//          bool laserError = false;
//          unsigned long distanceS = 0;
//          for (int i = 0; i < 5; i++) {
//            VL53L0X_RangingMeasurementData_t measure;
//            lox.rangingTest(&measure, true); // pass in 'true' to get debug data printout!
//            
//            if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//              distanceS += measure.RangeMilliMeter;
//            } else {
//              laserError = true;
//            }
//          }
//
//          if (distanceS / 5 <= 200 && !laserError) {
//            Serial.print("Sum of distances was: "); Serial.print(distanceS); Serial.print(" Average distance was: "); Serial.println(distanceS / 3);
//            if (servoAngle <= 90) {
//              clearTurnDirection = Direction::Left; // turn the other way
//              Serial.println("CheckForwardForObjects: Object on right side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
//            } else {
//              Serial.println("CheckForwardForObjects: Object on left side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
//              clearTurnDirection = Direction::Right; // turn the other way
//            }
//            aligningStartHeading = lastDetectedHeading;
//            runningMode = Mode::AligningWithClearPath;
//            randomTurnAmount = random(10, 90);
//            edgeDetected = true;
//            ignoreLine = true;
//            followingLine = false;
//            break;
//          }


          VL53L0X_RangingMeasurementData_t measure;
          lox.rangingTest(&measure, true); // pass in 'true' to get debug data printout!
//          int maxSamples = 0;
//          while(measure.RangeStatus != 0 && maxSamples < 10) {
//            lox.rangingTest(&measure, true); // pass in 'true' to get debug data printout!
//            maxSamples++;
//          }

          if (measure.RangeStatus == 2 && measure.RangeMilliMeter <= 200) {
            if (servoAngle <= 90) {
              clearTurnDirection = Direction::Left; // turn the other way
              Serial.println("CheckForwardForObjects: Object on right side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
            } else {
              Serial.println("CheckForwardForObjects: Object on left side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
              clearTurnDirection = Direction::Right; // turn the other way
            }
            aligningStartHeading = lastDetectedHeading;
            runningMode = Mode::AligningWithClearPath;
            randomTurnAmount = random(10, 90);
            edgeDetected = true;
            ignoreLine = true;
            followingLine = false;
            break;
          }

          delay(SWEEP_DELAY);
        }

        servoAngle = 0;
        forwardServo.write(servoAngle);
        timeSinceLastForwardCheck = millis();

        if (!edgeDetected && !followingLine) {
          Serial.println("CheckForwardForObjects: Sweep complete, path forward clear, continuing forward");
          runningMode = Mode::MovingStraight; 
        } else if (followingLine && !ignoreLine) {
          Serial.println("CheckForwardForObjects: Line detected");
          runningMode = Mode::FollowingLine;
        }
      }
      break;
    case Mode::FollowingLine: {

        if (millis() - timeSinceLastForwardCheck > FOR_CHECK_DELAY_LINE) {
          Serial.println("MovingStraight: Forward path no longer guaranteed clear, rechecking");
          runningMode = Mode::CheckForwardForObjects;
          break;
        }
      
        if (lineDetected) {
          Serial.println("FollowingLine: Line detected, following.");
          forward();
        } else {
          Serial.println("FollowingLine: Line lost");
          aligningStartHeading = lastDetectedHeading;
          lineTurnAmount = 45;
          turningDirection = lastLineSearchDirection;
          runningMode = Mode::FindingLine;
        }
      }
      break;
    case Mode::FindingLine:
      {
        
        if (lineDetected) {
          stop();
          Serial.println("FindingLine: Line detected, following.");
          runningMode = Mode::FollowingLine;
          break;
        }

        float amountTurned = 180 - abs(abs(lastDetectedHeading - aligningStartHeading) - 180);

        Serial.print("Turned: "); Serial.print(amountTurned); Serial.print(" out of: "); Serial.println(lineTurnAmount);
        if (amountTurned <= lineTurnAmount) {
          if (turningDirection == Direction::Left) {
            Serial.println("FindingLine: Line not detected, looking, turning left");
            lastLineSearchDirection = Direction::Left;
            turnLeft();
          } else {
            Serial.println("FindingLine: Line not detected, looking, turning right");
            lastLineSearchDirection = Direction::Right;
            turnRight();
          }
        } else {
          lineSearchTurns++;
          if (lineSearchTurns == 1) {
            Serial.println("FindingLine: Line not detected, looking, switching turn direction");
            if (turningDirection == Direction::Right) {
              turningDirection = Direction::Left;
            } else {
              turningDirection = Direction::Right;
            }
            lineTurnAmount = 90;
            aligningStartHeading = lastDetectedHeading;
          } else if (lineSearchTurns == 2) {
            Serial.println("FindingLine: Line not detected, line must've ended");
            runningMode = Mode::CheckForwardForObjects;
            followingLine = false;
            lineSearchTurns = 0;
          }
        }
      }
      break;
    case Mode::MovingStraight:
      { // way forward is clear, so go that way
        forward();

        if (millis() - timeSinceLastForwardCheck > FOR_CHECK_DELAY_NORMAL) {
          Serial.println("MovingStraight: Forward path no longer guaranteed clear, rechecking");
          runningMode = Mode::CheckForwardForObjects;
        }

        if (followingLine) {
          aligningStartHeading = lastDetectedHeading;
          Serial.println("MovingStraight: Line detected");
          runningMode = Mode::FollowingLine;
        }
      }
      break;
    case Mode::Error:
      { // something is wrong with Arduino or its peripheral devices, stop to keep the robot safe
        Serial.println("Error: Issue with setup, robot not safe to move! Polling sensors...");
        stop();
        delay(1000);
      }
      break;
  }
}

void pollIMU() {
  unsigned long tStart = micros();
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  lastDetectedHeading = orientationData.orientation.x;
  //  Serial.print("Heading: ");
  //  Serial.println(lastDetectedHeading);
}

void pollLineSensor() {
  
  SensorResult lineRight1 = forwardLineRightSensor1.getResult();
  SensorResult lineLeft1 = forwardLineLeftSensor1.getResult();
  SensorResult lineRight2 = forwardLineRightSensor2.getResult();
  SensorResult lineLeft2 = forwardLineLeftSensor2.getResult();

  lineDetected = false;
  if (lineRight1 == SensorResult::Nothing || lineLeft1 == SensorResult::Nothing || lineRight2 == SensorResult::Nothing || lineRight2 == SensorResult::Nothing) {
    lineDetected = true;
    followingLine = true;
  }
  
//  if (lineDetected) {
//    Serial.println("Unhandled line");
//  } else {
//    Serial.println("No line");
//  }
}

void forward()
{
  if (motorsEnabled) {
    motorFrontRight.run(FORWARD);
    motorBackRight.run(FORWARD);
    motorFrontLeft.run(FORWARD);
    motorBackLeft.run(FORWARD);
  }
}

void backward()
{
  if (motorsEnabled) {
    motorFrontRight.run(BACKWARD);
    motorBackRight.run(BACKWARD);
    motorFrontLeft.run(BACKWARD);
    motorBackLeft.run(BACKWARD);
  }
}

void turnRight()
{
  if (motorsEnabled) {
    motorFrontRight.run(BACKWARD);
    motorBackRight.run(BACKWARD);
    motorFrontLeft.run(FORWARD);
    motorBackLeft.run(FORWARD);
  }
}

void turnLeft()
{
  if (motorsEnabled) {
    motorFrontRight.run(FORWARD);
    motorBackRight.run(FORWARD);
    motorFrontLeft.run(BACKWARD);
    motorBackLeft.run(BACKWARD);
  }
}

void stop()
{
  if (motorsEnabled) {
    motorFrontRight.run(RELEASE);
    motorBackRight.run(RELEASE);
    motorFrontLeft.run(RELEASE);
    motorBackLeft.run(RELEASE);
  }
}
