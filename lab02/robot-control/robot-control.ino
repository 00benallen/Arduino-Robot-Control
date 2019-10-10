/** Servo **/
#include <Servo.h>
Servo forwardServo;
int servoPos = 0;
int servoPosPin = 10;
const unsigned long SWEEP_DELAY = 200;
unsigned long timeOfLastSweep = 0;
unsigned int servoAngle = 0;

unsigned long timeSinceLastForwardCheck = 0;
unsigned int FOR_CHECK_DELAY = 2000;

/** Motors **/

#include <AFMotor.h>
AF_DCMotor motorFrontRight(3);
AF_DCMotor motorBackRight(2);
AF_DCMotor motorFrontLeft(4);
AF_DCMotor motorBackLeft(1);

const int MOTOR_SPEED_LOW = 180;
const int MOTOR_SPEED_HIGH = 255;

/** Timing Constants **/
const int MAX_BACKUP_TIME_MAX = 300;
const int MAX_BACKUP_TIME_MIN = 100;
int backupTime = MAX_BACKUP_TIME_MAX;
const int MAX_ALIGNING_TIME = 1200;
const int MIN_ALIGNING_TIME = 200;

/** Sensors **/
#include "Sensors.h"
#include "Ultrasonic.h"
int sonicSensorDownTriggerPin = 52;
int sonicSensorDownEchoPin = 53;
int IRSensorForwardOutPin = 23;

const int EDGE_DET_THRES = 40;
const unsigned long SENSOR_MAX_TIMEOUT = 2915;

//SonicSensor forwardDownSensor(sonicSensorDownTriggerPin, sonicSensorDownEchoPin, EDGE_DET_THRES, SENSOR_MAX_TIMEOUT);
Ultrasonic forwardDownSensor(sonicSensorDownTriggerPin, sonicSensorDownEchoPin, SENSOR_MAX_TIMEOUT);
IRSensor forwardObjectSensor(IRSensorForwardOutPin);

#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
float lastDetectedHeading;




/**
   Enum representing possible modes the robot can be in during the run loop
*/
enum class Mode
{
  MovingStraight,        // robot is moving forward along a clear path
  AligningWithClearPath, // robot is aligning itself with a found clear path
  EmergencyBackup,       // robot is backing up (reversing) because an obstacle is directly in front
  CheckForwardForObjects,
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

/**
   Global variables that change
*/
// state that persists across loops
int motorSpeed = MOTOR_SPEED_LOW;
Direction clearTurnDirection = Direction::Left;
Mode runningMode = Mode::CheckForwardForObjects;

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

  Serial.println("Initializing VL53LDK laser TOF sensor");
  while (!lox.begin()) {
    Serial.println(F("No VL53LDK detected"));
    delay(1000);
  }
  Serial.println("VL53LDK laser TOF sensor detected");

  Serial.println("Initializing sonic sensors");
  //forwardDownSensor.init();

  Serial.println("Initializing IR sensors");
  forwardObjectSensor.init();

  Serial.println("Initializing auxilliary pins");
  pinMode(IRSensorForwardOutPin, INPUT);
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
          Serial.println("EmergencyBackup: Complete, beginning turn to find clear path");
          aligningStartHeading = lastDetectedHeading;
          runningMode = Mode::AligningWithClearPath;

        }
      }
      break;
    case Mode::AligningWithClearPath:
      { // turn to align the robot with the currently detected clear path
        if (motorsEnabled) {
          float amountTurned = 180 - abs(abs(lastDetectedHeading - aligningStartHeading) - 180);
          if (amountTurned <= 90)
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
            runningMode = Mode::CheckForwardForObjects;
          }
        } else {
          Serial.println("AligningWithClearPath: Clear path alignment finished (motors disabled), starting to find a forward path");
          runningMode = Mode::CheckForwardForObjects;
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
          servoAngle += 10;
          unsigned long downDistance = forwardDownSensor.read();

          if (downDistance >= EDGE_DET_THRES) {
            Serial.println("CheckForwardForObjects: Edge detected during sweep, reseting servo and initiating emergency backup");
            runningMode = Mode::EmergencyBackup;
            edgeDetected = true;
            break;
          }

          SensorResult objectResult = forwardObjectSensor.getResult();

          if (objectResult == SensorResult::Object) {
            Serial.println("CheckForwardForObjects: Object detected during sweep by IR sensor, reseting servo and initiating emergency backup");
            runningMode = Mode::EmergencyBackup;
            edgeDetected = true;
            break;
          }

          VL53L0X_RangingMeasurementData_t measure;

          lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

          if (measure.RangeStatus == VL53L0X_ERROR_NONE && measure.RangeMilliMeter <= 300) {
            Serial.println("CheckForwardForObjects: Object detected during sweep by laser TOF sensor, reseting servo and initiating emergency backup");
            runningMode = Mode::EmergencyBackup;
            edgeDetected = true;
            break;
          }

          delay(500);
        }

        Serial.println("CheckForwardForObjects: Resetting forward servo");
        while (servoAngle > 0) {
          servoAngle -= 10;
          forwardServo.write(servoAngle);
          delay(500);
        }
        timeSinceLastForwardCheck = millis();

        if (!edgeDetected) {
          Serial.println("CheckForwardForObjects: Sweep complete, path forward clear, continuing forward");
          runningMode = Mode::MovingStraight;
        }
      }
      break;
    case Mode::MovingStraight:
      { // way forward is clear, so go that way
        forward();

        if (millis() - timeSinceLastForwardCheck > FOR_CHECK_DELAY) {
          Serial.println("MovingStraight: Forward path no longer guaranteed clear, rechecking");
          runningMode = Mode::CheckForwardForObjects;
        }
      }
      break;
    case Mode::Error:
      { // something is wrong with Arduino or its peripheral devices, stop to keep the robot safe
        Serial.println("Error: Issue with setup, robot not safe to move! Polling sensors...");
        stop();
        if (forwardDownSensor.read() != 0) {
          Serial.println("Error: Forward sonic sensor working again!");
          aligningStartHeading = lastDetectedHeading;
          runningMode = Mode::AligningWithClearPath;
        }
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
