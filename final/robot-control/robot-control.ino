/** Motors **/
#include <AFMotor.h>
AF_DCMotor motorFrontRight(3);
AF_DCMotor motorBackRight(2);
AF_DCMotor motorFrontLeft(4);
AF_DCMotor motorBackLeft(1);

/** Motor Constants **/
const int MOTOR_SPEED_LOW = 120;
const int MOTOR_SPEED_HIGH = 255;

/** Sensors **/

// 9-Axis IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// IR TOF Sensor
//#include "Adafruit_VL53L0X.h"
//Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// QTR Sensor
#include <QTRSensors.h>
QTRSensors qtr;

// Auxilliary Line Sensors
int leftLineAuxPin = 22;
int rightLineAuxPin = 52;

// Fire Sensors
int leftFirePin = 30;
int rightFirePin = 31;
int forwardFirePin = 29;



/** State Machine **/
#include "StateMachine.h"
StateMachine stateMach(State::FollowingLine);

/** Navigation **/
#include "Navigator.h"
Navigator nav(true);

/** Miscellaneous **/
int randomSeedPin = 15;

void setup()
{
  Serial.begin(9800);
  Serial.println("[CPS603-Robot Control Program] starting up");

  initializeMotors();

  initializeIMU();

  initializeAuxIRs();

  initializeFires();

  //  initializeTOF();

  initializeQTR();
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

void initializeAuxIRs() {
  Serial.println("Initializing Auxillary IR Sensors");
  pinMode(leftLineAuxPin, INPUT);
  pinMode(rightLineAuxPin, INPUT);
}

void initializeFires() {
  Serial.println("Flame Sensors");
  pinMode(leftFirePin, INPUT);
  pinMode(rightFirePin, INPUT);
  pinMode(forwardFirePin, INPUT);
}

//void initializeTOF() {
//  Serial.println("Initializing VL53L0X laser TOF sensor");
//  Serial.println("Adafruit VL53L0X test");
//  while (!lox.begin()) {
//    Serial.println(F("Failed to boot VL53L0X"));
//    delay(1000);
//  }
//  Serial.println("VL53L0X laser TOF sensor detected");
//}

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
  for (uint16_t i = 0; i < 100; i++)
  {
    if (i <= 25) {
      turnLeft();
    } else if (i <= 75) {
      turnRight();
    } else {
      turnLeft();
    }

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
  pollLineSensors();
  pollFlameSensors();

  if (stateMach.flameData.leftFlameDet || stateMach.flameData.rightFlameDet || stateMach.flameData.forwardFlameDet || stateMach.flameData.aligningWithFlame) {

    if (!stateMach.flameData.aligningWithFlame) {
      stop();
      Serial.println("Fire detected, stop");
      delay(500);
    } else {
      if (stateMach.flameData.leftFlameDet) {
        turnLeft();
      } else if (stateMach.flameData.rightFlameDet) {
        turnRight();
      }

      if (stateMach.flameData.forwardFlameDet) {
        stop();
        delay(500);
        stateMach.flameData.aligningWithFlame = false;
        return;
      }
    }
    stateMach.flameData.aligningWithFlame = true;
  } else {

    if (stateMach.lineData.lineLeftEnded || stateMach.lineData.lineRightEnded) {

      stateMach.lineData.intersectionHandled = false;
      stateMach.lineData.pulledAwayFromIntersection = false;
      stateMach.saveAligningStartHeading();
      Serial.print("Forward: "); Serial.println(stateMach.lineData.foundLineForward);
      Serial.print("Left: "); Serial.println(stateMach.lineData.foundLineLeft);
      Serial.print("Right: "); Serial.println(stateMach.lineData.foundLineRight);

      if (stateMach.lineData.foundLineForward) {

        if (stateMach.lineData.foundLineRight && stateMach.lineData.foundLineLeft) {

          nav.setLastIntersection(Intersection::FourWay);
          stateMach.lineData.foundLineLeft = false;
          stateMach.lineData.foundLineRight = false;
          stateMach.lineData.foundLineForward = false;
          stateMach.lineData.lineRightEnded = false;
          stateMach.lineData.lineLeftEnded = false;

        } else if (stateMach.lineData.foundLineRight) {

          nav.setLastIntersection(Intersection::ThreeWayRight);
          stateMach.lineData.foundLineLeft = false;
          stateMach.lineData.foundLineRight = false;
          stateMach.lineData.foundLineForward = false;
          stateMach.lineData.lineRightEnded = false;
          stateMach.lineData.lineLeftEnded = false;

        } else if (stateMach.lineData.foundLineLeft) {

          nav.setLastIntersection(Intersection::ThreeWayLeft);
          stateMach.lineData.foundLineLeft = false;
          stateMach.lineData.foundLineRight = false;
          stateMach.lineData.foundLineForward = false;
          stateMach.lineData.lineRightEnded = false;
          stateMach.lineData.lineLeftEnded = false;

        } else {
          nav.setLastIntersection(Intersection::Booth);
          stateMach.lineData.foundLineLeft = false;
          stateMach.lineData.foundLineRight = false;
          stateMach.lineData.foundLineForward = false;
          stateMach.lineData.lineRightEnded = false;
          stateMach.lineData.lineLeftEnded = false;
        }

      } else {

        if (stateMach.lineData.foundLineLeft && stateMach.lineData.foundLineRight) {

          nav.setLastIntersection(Intersection::ThreeWayTee);
          stateMach.lineData.foundLineLeft = false;
          stateMach.lineData.foundLineRight = false;
          stateMach.lineData.foundLineForward = false;
          stateMach.lineData.lineRightEnded = false;
          stateMach.lineData.lineLeftEnded = false;

        } else if (stateMach.lineData.foundLineRight) {

          nav.setLastIntersection(Intersection::RightCorner);
          stateMach.lineData.foundLineLeft = false;
          stateMach.lineData.foundLineRight = false;
          stateMach.lineData.foundLineForward = false;
          stateMach.lineData.lineRightEnded = false;
          stateMach.lineData.lineLeftEnded = false;

        } else if (stateMach.lineData.foundLineLeft) {

          nav.setLastIntersection(Intersection::LeftCorner);
          stateMach.lineData.foundLineLeft = false;
          stateMach.lineData.foundLineRight = false;
          stateMach.lineData.foundLineForward = false;
          stateMach.lineData.lineRightEnded = false;
          stateMach.lineData.lineLeftEnded = false;

        } else {

          nav.setLastIntersection(Intersection::Booth);
          stateMach.lineData.foundLineLeft = false;
          stateMach.lineData.foundLineRight = false;
          stateMach.lineData.foundLineForward = false;
          stateMach.lineData.lineRightEnded = false;
          stateMach.lineData.lineLeftEnded = false;

        }
      }
    }

    //Get turn recommendation
    if (!stateMach.lineData.intersectionHandled) {
      TurnDirection newTurn = nav.getNextTurn();

      if (newTurn != TurnDirection::None) {
        float amountTurned = calculateAmountTurned();
        Serial.println("Cur amount turned: "); Serial.print(amountTurned);
        if (newTurn == TurnDirection::Left) {
          Serial.println("Turning left");
          turnLeft();
        } else if (newTurn == TurnDirection::Right) {
          Serial.println("Turning right");
          turnRight();
        }

        if (amountTurned >= 45 && !stateMach.lineData.doneIntersectionFirstTurnStep) {
          stateMach.lineData.doneIntersectionFirstTurnStep = true;
          forward();
          delay(400);
        }

        if (stateMach.lineData.doneIntersectionFirstTurnStep) {
          Serial.println("30 degrees done, continuing turn");

          if ((int)(amountTurned) % 1 == 0) {
            stop();
            delayMicroseconds(3);
          }

          if (stateMach.lineData.lineDetected) {
            Serial.println("Line found, ending turn");
            stateMach.lineData.doneIntersectionFirstTurnStep = false;
            stateMach.lineData.intersectionHandled = true;
            nav.setLastTurn(newTurn);
          }
        }
      } else {
        Serial.println("No turn recommended, continuing straight");
        stop();
        stateMach.lineData.intersectionHandled = true;
        nav.setLastTurn(newTurn);
      }
    } else if (stateMach.lineData.lineDetected) {
      Serial.println("Following line normally");
      if (stateMach.lineData.linePosition < 1000) {
        Serial.println("Turning Right");
        turnRight();
      } else if (stateMach.lineData.linePosition > 6000) {
        Serial.println("Turning Left");
        turnLeft();
      } else {
        stateMach.lineData.pulledAwayFromIntersection = true;
        forward();
      }
    } else {
      forward();
    }

  }
}

void pollIMU() {
  unsigned long tStart = micros();
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  stateMach.imuData.lastDetectedHeading = orientationData.orientation.x;
}

void pollLineSensors() {

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  int sensorCount = 8;
  unsigned int sensorValues[8];
  uint16_t position = qtr.readLineBlack(sensorValues);
  memcpy(stateMach.lineData.rawLineValues, sensorValues, 8 * sizeof(unsigned int));

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  stateMach.lineData.lineDetected = false;
  for (uint8_t i = 0; i < 8; i++)
  {
    unsigned int curReading = sensorValues[i];
    Serial.print(curReading); Serial.print(" ");
    if (curReading > 700) {
      stateMach.lineData.lineDetected = true;
      stateMach.lineData.followingLine = true;

      if (i >= 2 && i <= 5) {
        stateMach.lineData.foundLineForward = true;
      }
    }
  }

  Serial.print(position);
  Serial.println();

  stateMach.lineData.linePosition = position;

  if (stateMach.lineData.intersectionHandled && stateMach.lineData.pulledAwayFromIntersection) {
    if (digitalRead(leftLineAuxPin) == HIGH) {
      Serial.println("FOUND LINE LEFT =========================================");
      stateMach.lineData.foundLineLeft = true;
    } else {
      stateMach.lineData.lineLeftEnded = stateMach.lineData.foundLineLeft;
    }

    if (digitalRead(rightLineAuxPin) == HIGH) {
      Serial.println("Found line right =========================================");
      stateMach.lineData.foundLineRight = true;
    } else {
      stateMach.lineData.lineRightEnded = stateMach.lineData.foundLineRight;
    }
  }
}

void pollFlameSensors() {
  if (digitalRead(leftFirePin) == LOW) {
    Serial.println("Left fire detected");
    stateMach.flameData.leftFlameDet = true;
  } else {
    stateMach.flameData.leftFlameDet = false;
  }

  if (digitalRead(rightFirePin) == LOW) {
    Serial.println("Right fire detected");
    stateMach.flameData.rightFlameDet = true;
  } else {
    stateMach.flameData.rightFlameDet = false;
  }

  if (digitalRead(forwardFirePin) == LOW) {
    Serial.println("Right fire detected");
    stateMach.flameData.forwardFlameDet = true;
  } else {
    stateMach.flameData.forwardFlameDet = false;
  }
}

float calculateAmountTurned() {
  return 180 - abs(abs(stateMach.imuData.lastDetectedHeading - stateMach.moveData.aligningStartHeading) - 180);
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
