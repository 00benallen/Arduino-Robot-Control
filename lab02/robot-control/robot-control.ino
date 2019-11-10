/** Servo **/
#include <Servo.h>
Servo forwardServo;
int servoPosPin = 10;

/** Servo Constants **/
unsigned int FOR_CHECK_DELAY_NORMAL = 500;
unsigned int FOR_CHECK_DELAY_LINE = 1000;
const unsigned long SWEEP_DELAY = 40;

/** Motors **/
#include <AFMotor.h>
AF_DCMotor motorFrontRight(3);
AF_DCMotor motorBackRight(2);
AF_DCMotor motorFrontLeft(4);
AF_DCMotor motorBackLeft(1);

/** Motor Constants **/
const int MOTOR_SPEED_LOW = 150;
const int MOTOR_SPEED_HIGH = 255;
const int BACKUP_TIME = 300;
const int MAX_ALIGNING_TIME = 1200;
const int MIN_ALIGNING_TIME = 200;

/** Sensors **/

/** Sensor Constants **/
const int EDGE_DET_THRES = 40;
const unsigned long SENSOR_MAX_TIMEOUT = 2915;

#include "Sensors.h"

// Ultrasonic Sensor
#include "Ultrasonic.h"
int sonicSensorDownTriggerPin = 26;
int sonicSensorDownEchoPin = 23;
Ultrasonic forwardEdgeSensor(sonicSensorDownTriggerPin, sonicSensorDownEchoPin, SENSOR_MAX_TIMEOUT);

// 9-Axis IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// IR TOF Sensor
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// QTR Sensor
#include <QTRSensors.h>
QTRSensors qtr;

// Photoresistor Sensor
int photoPin = A8;

/** State Machine **/
#include "StateMachine.h"
StateMachine stateMach(State::FollowingLine);

/** Miscellaneous **/
int randomSeedPin = 15;

void setup()
{
  Serial.begin(9800);
  Serial.println("[CPS603-Robot Control Program] starting up");

  initializeMotors();

  initializeIMU();

  initializeTOF();

  //  intializeIRs();

  initializeQTR();

  Serial.println("Initializing auxilliary pins");
  pinMode(randomSeedPin, INPUT);
  randomSeed(analogRead(randomSeedPin));

  pinMode(photoPin, INPUT);

  initializeServo();

  Serial.println("Waiting before start...");
  delay(10000);

}

void initializeMotors() {
  Serial.println("Initializing motors");
  motorFrontRight.setSpeed(MOTOR_SPEED_LOW);
  motorFrontRight.run(RELEASE);
  motorBackRight.setSpeed(MOTOR_SPEED_LOW);
  motorBackRight.run(RELEASE);
  motorFrontLeft.setSpeed(MOTOR_SPEED_LOW);
  motorFrontLeft.run(RELEASE);
  motorBackLeft.setSpeed(MOTOR_SPEED_LOW);
  motorBackLeft.run(RELEASE);
}

void initializeIMU() {
  Serial.println("Initializing BNO055 9-axis IMU");
  while (!bno.begin())
  {
    Serial.println("No BNO055 detected");
    delay(1000);
  }
  Serial.println("BNO055 9-axis IMU detected and initialized");
}

void initializeTOF() {
  Serial.println("Initializing VL53L0X laser TOF sensor");
  Serial.println("Adafruit VL53L0X test");
  while (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1000);
  }
  Serial.println("VL53L0X laser TOF sensor detected");
}

//void intializeIRs() {
//  Serial.println("Initializing IR sensors");
//  forwardLineRightSensor1.init();
//  forwardLineLeftSensor1.init();
//  forwardLineRightSensor2.init();
//  forwardLineLeftSensor2.init();
//}

void initializeServo() {
  Serial.println("Initializing forward servo");
  forwardServo.attach(servoPosPin);
  stateMach.servoData.timeOfLastSweep = millis();
  forwardServo.write(90);
}

void initializeQTR() {

  int sensorCount = 8;
  Serial.println("Initializing QTC Reflectance Array");
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    53, 51, 49, 47, 45, 43, 41, 39
  }, sensorCount);

  Serial.println("Calibrating... sweep over line please");

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  // print the calibration minimum values measured when emitters were on
  Serial.println("Minimum reflectance values");
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  Serial.println("Maximum reflectance values");
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void loop()
{

  long loopStart = millis(); // useful for timing of various processes in run loop
  pollIMU();
  if (!stateMach.lineData.ignoreLine) {
    pollLineSensor();
  }
  pollPhotoresistor();

  // Main switch case for determining what behaviour the robot should be doing, based on the current state
  switch (stateMach.currentState)
  {
    case State::EmergencyBackup:
      { // reverse the robot for a certain amount of time, to ensure it is far enough away from a hazard to turn safely
        stateMach.moveData.backupTimeElapsed += millis() - loopStart;

        if (stateMach.moveData.backupTimeElapsed < BACKUP_TIME)
        {
          backward();
        }
        else
        {
          stateMach.emergencyBackupComplete();
        }
      }
      break;
    case State::AligningWithClearPath:
      { // turn to align the robot with the currently detected clear path
        if (stateMach.moveData.motorsEnabled) {

          if (stateMach.lineData.followingLine) {
            stateMach.lineDetected();
            break;
          }


          float amountTurned = calculateAmountTurned();
          if (amountTurned <= 90)
          {
            if (stateMach.moveData.clearTurnDirection == Direction::Right)
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
            stateMach.aligningWithClearPathComplete();
          }
        } else {
          stateMach.aligningWithClearPathComplete();
        }
      }
      break;
    case State::CheckForwardForObjects:
      {
        Serial.println("[CheckForwardForObjects]: Forward sweep starting, stopping robot until path forward is guaranteed clear");
        stop();
        stateMach.servoData.servoAngle = 0;
        bool edgeDetected = false;
        bool lightDetected = false;
        while (stateMach.servoData.servoAngle <= 180) {
          moveServo();
          stateMach.servoData.servoAngle += 5;

          pollPhotoresistor();
          if (stateMach.lightData.lightReading <= 515) {
            lightDetected = true;
            if (stateMach.lightData.lightReading < stateMach.lightData.lightForwardReading || stateMach.lightData.lightForwardReading == 0) {
              stateMach.lightData.lightForwardReading = stateMach.lightData.lightReading;
              stateMach.lightData.lightAngle = stateMach.servoData.servoAngle;
            }
          }

          unsigned long distanceCM = forwardEdgeSensor.read(CM);
          if (distanceCM > EDGE_DET_THRES) {
            stateMach.edgeDetected();
            edgeDetected = true;
            break;
          }

          VL53L0X_RangingMeasurementData_t measure;
          lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
          if (measure.RangeStatus == 2 && measure.RangeMilliMeter <= 200) {
            stateMach.objectDetected();
            edgeDetected = true;
          }

          delay(SWEEP_DELAY);
        }

        if (!edgeDetected && !stateMach.lineData.followingLine && !lightDetected) {
          stateMach.checkForwardForObjectsClear();
        } else if (stateMach.lineData.followingLine && !stateMach.lineData.ignoreLine && !lightDetected) {
          stateMach.lineDetected();
        } else if (lightDetected) {
          stateMach.lightDetected();
          moveServo();
          delay(500);
          stateMach.moveData.timeSinceLastForwardCheck = millis();
          break;
        }

        stateMach.servoData.servoAngle = 0;
        moveServo();
        stateMach.moveData.timeSinceLastForwardCheck = millis();
      }
      break;
    case State::FollowingLine: {

        if (millis() - stateMach.moveData.timeSinceLastForwardCheck > FOR_CHECK_DELAY_LINE) {
          stateMach.forwardCheckTimeout();
          break;
        }

        if (stateMach.lineData.lineDetected) {
          Serial.println("FollowingLine: Line detected, following.");
          forward();

        } else {
          stateMach.lineLostWhileFollowing();
        }
      }
      break;
    case State::FindingLine:
      {
        float amountTurned = calculateAmountTurned();

        if (amountTurned < 90) {
          if (stateMach.lineData.linePosition <= 0) {
            turnLeft();
          } else if (stateMach.lineData.linePosition >= 7000) {
            turnRight();
          } else {
            stateMach.lineDetected();
            break;
          }
        } else {
          stateMach.endOfLineDetected();
        }
      }
      break;
    case State::FollowingLight:
      {
        
        if (abs(stateMach.lightData.lightReading - stateMach.lightData.lightForwardReading) < 2 || stateMach.lightData.lightReading < stateMach.lightData.lightForwardReading) {
          if (stateMach.lightData.lightReading < 475) {
            Serial.println("Light too close");
            stop();
          } else {
            Serial.println("Light lined up and not too close, light may be extinguished");
            Serial.println(stateMach.lightData.lightReading);
            Serial.println(stateMach.lightData.lightForwardReading);
            stateMach.lightData.lightForwardReading = 0;
            stateMach.checkForwardForObjectsClear();
          }
        } else {
          float amountTurned = calculateAmountTurned();
          if (amountTurned < 90) {
  
            if (stateMach.lightData.turnDirection == Direction::Left) {
              Serial.println("Turning left to light");
              turnLeft();
            } else {
              Serial.println("Turning right to light");
              turnRight();
            }
          } else {
            Serial.println("Light lost, giving up");
            stateMach.lightData.lightForwardReading = 0;
            stateMach.lightLost();
          }
        }
      }
      break;
    case State::MovingStraight:
      { // way forward is clear, so go that way
        forward();

        if (millis() - stateMach.moveData.timeSinceLastForwardCheck > FOR_CHECK_DELAY_NORMAL) {
          stateMach.forwardCheckTimeout();
        }

        if (stateMach.lineData.followingLine) {
          stateMach.lineDetected();
        }
      }
      break;
    case State::Error:
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
  stateMach.imuData.lastDetectedHeading = orientationData.orientation.x;
}

void pollLineSensor() {

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  int sensorCount = 8;
  uint16_t sensorValues[sensorCount];
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  stateMach.lineData.lineDetected = false;
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    uint16_t curReading = sensorValues[i];
    if (curReading > 500) {
      stateMach.lineData.lineDetected = true;
      stateMach.lineData.followingLine = true;
    }
  }
  stateMach.lineData.linePosition = position;
}

void pollPhotoresistor() {
  stateMach.lightData.lightReading = analogRead(photoPin);
  Serial.print("Light Resistance: "); Serial.println(stateMach.lightData.lightReading);
}

float calculateAmountTurned() {
  return 180 - abs(abs(stateMach.imuData.lastDetectedHeading - stateMach.moveData.aligningStartHeading) - 180);
}

void moveServo() {
  if (stateMach.servoData.servoEnabled) {
    forwardServo.write(stateMach.servoData.servoAngle);
  }
}

void forward()
{
  if (stateMach.moveData.motorsEnabled) {
    motorFrontRight.run(FORWARD);
    motorBackRight.run(FORWARD);
    motorFrontLeft.run(FORWARD);
    motorBackLeft.run(FORWARD);
  }
}

void backward()
{
  if (stateMach.moveData.motorsEnabled) {
    motorFrontRight.run(BACKWARD);
    motorBackRight.run(BACKWARD);
    motorFrontLeft.run(BACKWARD);
    motorBackLeft.run(BACKWARD);
  }
}

void turnRight()
{
  if (stateMach.moveData.motorsEnabled) {
    motorFrontRight.run(BACKWARD);
    motorBackRight.run(BACKWARD);
    motorFrontLeft.run(FORWARD);
    motorBackLeft.run(FORWARD);
  }
}

void turnLeft()
{
  if (stateMach.moveData.motorsEnabled) {
    motorFrontRight.run(FORWARD);
    motorBackRight.run(FORWARD);
    motorFrontLeft.run(BACKWARD);
    motorBackLeft.run(BACKWARD);
  }
}

void stop()
{
  if (stateMach.moveData.motorsEnabled) {
    motorFrontRight.run(RELEASE);
    motorBackRight.run(RELEASE);
    motorFrontLeft.run(RELEASE);
    motorBackLeft.run(RELEASE);
  }
}
