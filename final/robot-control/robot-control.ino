/** Servo **/
#include <Servo.h>
Servo forwardServo;
int servoPosPin = 10;

/** Servo Constants **/
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

// Ultrasonic Sensor
//#include "Ultrasonic.h"
//int sonicSensorDownTriggerPin = 26;
//int sonicSensorDownEchoPin = 23;
//Ultrasonic forwardEdgeSensor(sonicSensorDownTriggerPin, sonicSensorDownEchoPin, SENSOR_MAX_TIMEOUT);

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

// Photoresistor Sensor
//int photoPin = A8;

int leftLineAuxPin = 22;
int rightLineAuxPin = 52;

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

  //  initializeTOF();

  initializeQTR();

  //  Serial.println("Initializing auxilliary pins");
  //  pinMode(randomSeedPin, INPUT);
  //  randomSeed(analogRead(randomSeedPin));
  //
  //  pinMode(photoPin, INPUT);
  //
  //  initializeServo();
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

//void initializeTOF() {
//  Serial.println("Initializing VL53L0X laser TOF sensor");
//  Serial.println("Adafruit VL53L0X test");
//  while (!lox.begin()) {
//    Serial.println(F("Failed to boot VL53L0X"));
//    delay(1000);
//  }
//  Serial.println("VL53L0X laser TOF sensor detected");
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

  //  long loopStart = millis(); // useful for timing of various processes in run loop
  //  pollIMU();
  //  if (!stateMach.lineData.ignoreLine) {
  //    pollLineSensor();
  //  }
  //  pollPhotoresistor();
  //
  //  // Main switch case for determining what behaviour the robot should be doing, based on the current state
  //  switch (stateMach.currentState)
  //  {
  //    case State::EmergencyBackup:
  //      { // reverse the robot for a certain amount of time, to ensure it is far enough away from a hazard to turn safely
  //        stateMach.moveData.backupTimeElapsed += millis() - loopStart;
  //
  //        if (stateMach.moveData.backupTimeElapsed < BACKUP_TIME)
  //        {
  //          backward();
  //        }
  //        else
  //        {
  //          stateMach.emergencyBackupComplete();
  //        }
  //      }
  //      break;
  //    case State::AligningWithClearPath:
  //      { // turn to align the robot with the currently detected clear path
  //        if (stateMach.moveData.motorsEnabled) {
  //
  //          if (stateMach.lineData.followingLine) {
  //            stateMach.lineDetected();
  //            break;
  //          }
  //
  //
  //          float amountTurned = calculateAmountTurned();
  //          if (amountTurned <= stateMach.moveData.randomTurnAmount)
  //          {
  //            if (stateMach.moveData.clearTurnDirection == Direction::Right)
  //            {
  //              turnRight();
  //            }
  //            else
  //            {
  //              turnLeft();
  //            }
  //          }
  //          else
  //          {
  //            stateMach.aligningWithClearPathComplete();
  //          }
  //        } else {
  //          stateMach.aligningWithClearPathComplete();
  //        }
  //      }
  //      break;
  //    case State::CheckForwardForObjects:
  //      {
  //        Serial.println("[CheckForwardForObjects]: Forward sweep starting, stopping robot until path forward is guaranteed clear");
  //        stop();
  //        stateMach.servoData.servoAngle = 45;
  //        bool edgeDetected = false;
  //        bool lightDetected = false;
  //        while (stateMach.servoData.servoAngle <= 135) {
  //          moveServo();
  //          stateMach.servoData.servoAngle += 5;
  //
  //          pollPhotoresistor();
  //          if (stateMach.lightData.lightReading <= 515) {
  //            lightDetected = true;
  //            if (stateMach.lightData.lightReading < stateMach.lightData.lightForwardReading || stateMach.lightData.lightForwardReading == 0) {
  //              stateMach.lightData.lightForwardReading = stateMach.lightData.lightReading;
  //              stateMach.lightData.lightAngle = stateMach.servoData.servoAngle;
  //            }
  //          }
  //
  //          unsigned long distanceCM = forwardEdgeSensor.read(CM);
  //          if (distanceCM > EDGE_DET_THRES) {
  //            stateMach.edgeDetected();
  //            edgeDetected = true;
  //            break;
  //          }
  //
  //          VL53L0X_RangingMeasurementData_t measure;
  //          lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  //          if (measure.RangeStatus == 2 && measure.RangeMilliMeter <= 170) {
  //            stateMach.objectDetected();
  //            edgeDetected = true;
  //            break;
  //          }
  //
  //          delay(SWEEP_DELAY);
  //        }
  //
  //        if (stateMach.lineData.followingLine && !stateMach.lineData.ignoreLine && !lightDetected) {
  //          stateMach.lineDetected();
  //        } else if (lightDetected && !edgeDetected) {
  //          stateMach.lightDetected();
  //          moveServo();
  //          delay(500);
  //          stateMach.moveData.timeSinceLastForwardCheck = millis();
  //          break;
  //        } else if (!edgeDetected) {
  //          stateMach.checkForwardForObjectsClear();
  //        }
  //
  //        stateMach.servoData.servoAngle = 45;
  //        moveServo();
  //        stateMach.moveData.timeSinceLastForwardCheck = millis();
  //      }
  //      break;
  //    case State::FollowingLine: {
  //
  //        if (millis() - stateMach.moveData.timeSinceLastForwardCheck > stateMach.moveData.forwardCheckDelay) {
  //          stateMach.forwardCheckTimeout();
  //          break;
  //        }
  //
  //        if (stateMach.lineData.lineDetected) {
  //          Serial.println("FollowingLine: Line detected, following.");
  //          forward();
  //
  //        } else {
  //          stateMach.lineLostWhileFollowing();
  //        }
  //      }
  //      break;
  //    case State::FindingLine:
  //      {
  //        float amountTurned = calculateAmountTurned();
  //
  //        if (amountTurned < 90) {
  //          if (stateMach.lineData.linePosition <= 0) {
  //            turnLeft();
  //          } else if (stateMach.lineData.linePosition >= 7000) {
  //            turnRight();
  //          } else {
  //            stateMach.lineDetected();
  //            break;
  //          }
  //        } else {
  //          stateMach.endOfLineDetected();
  //        }
  //      }
  //      break;
  //    case State::FollowingLight:
  //      {
  //        if (stateMach.lightData.lightReading > 530) {
  //          Serial.println("Light extinguished");
  //          stateMach.lightData.found = false;
  //        }
  //
  //        if (stateMach.lightData.lightReading < 500 || stateMach.lightData.found) {
  //          Serial.println("Light too close");
  //          stateMach.lightData.found = true;
  //          stop();
  //          break;
  //        }
  //
  //        if (abs(stateMach.lightData.lightReading - stateMach.lightData.lightForwardReading) < 10 || stateMach.lightData.lightReading < stateMach.lightData.lightForwardReading) {
  //          Serial.println("Light lined up and not too close, light may be extinguished");
  //          Serial.println(stateMach.lightData.lightReading);
  //          Serial.println(stateMach.lightData.lightForwardReading);
  //          stateMach.lightData.lightForwardReading = 0;
  //          stateMach.checkForwardForObjectsClear();
  //        } else {
  //          float amountTurned = calculateAmountTurned();
  //          if (stateMach.lightData.linedUp) {
  //            Serial.println("Light within deadzone");
  //            stateMach.lightData.lightForwardReading = 0;
  //            stateMach.checkForwardForObjectsClear();
  //          }
  //          else if (amountTurned < 90) {
  //
  //            if (stateMach.lightData.turnDirection == Direction::Left) {
  //              Serial.println("Turning left to light");
  //              turnLeft();
  //            } else {
  //              Serial.println("Turning right to light");
  //              turnRight();
  //            }
  //          } else {
  //            Serial.println("Light lost, giving up");
  //            stateMach.lightData.lightForwardReading = 0;
  //            stateMach.lightLost();
  //          }
  //        }
  //      }
  //      break;
  //    case State::MovingStraight:
  //      { // way forward is clear, so go that way
  //        forward();
  //
  //        if (millis() - stateMach.moveData.timeSinceLastForwardCheck > stateMach.moveData.forwardCheckDelay) {
  //          stateMach.forwardCheckTimeout();
  //        }
  //
  //        if (stateMach.lineData.followingLine) {
  //          stateMach.lineDetected();
  //        }
  //      }
  //      break;
  //    case State::Error:
  //      { // something is wrong with Arduino or its peripheral devices, stop to keep the robot safe
  //        Serial.println("Error: Issue with setup, robot not safe to move! Polling sensors...");
  //        stop();
  //        delay(1000);
  //      }
  //      break;
  //  }

  long loopStart = millis(); // useful for timing of various processes in run loop
  pollIMU();
  pollLineSensors();

  if (stateMach.lineData.identifyingIntersection) {
    forward();
//    unsigned long timeElapsedIdentIntersection = millis() - stateMach.lineData.timeStartIdentIntersection;
    Serial.print("Forward: "); Serial.println(stateMach.lineData.foundLineForward);
    Serial.print("Left: "); Serial.println(stateMach.lineData.foundLineLeft);
    Serial.print("Right: "); Serial.println(stateMach.lineData.foundLineRight);

    // Identify intersection
    if (stateMach.lineData.foundLineForward) {
      nav.setLastIntersection(Intersection::Booth);

      if (stateMach.lineData.foundLineRight) {

        nav.setLastIntersection(Intersection::ThreeWayRight);

      } else if (stateMach.lineData.foundLineLeft) {

        nav.setLastIntersection(Intersection::ThreeWayLeft);

      }

//      if (timeElapsedIdentIntersection > 1500) {
//        Serial.println("Identification timeout");
//
//        if (stateMach.lineData.foundLineRight && stateMach.lineData.foundLineLeft) {
//          nav.setLastIntersection(Intersection::FourWay); // Shouldn't happen
//        }
//
//        // Cleanup
//        stateMach.lineData.identifyingIntersection = false;
//        stateMach.lineData.foundLineForward = false;
//        stateMach.lineData.foundLineLeft = false;
//        stateMach.lineData.foundLineRight = false;
//
//      }

    } else {
      if (stateMach.lineData.foundLineRight) {

        nav.setLastIntersection(Intersection::RightCorner);

      } else if (stateMach.lineData.foundLineLeft) {

        nav.setLastIntersection(Intersection::LeftCorner);

      }

//      if (timeElapsedIdentIntersection > 1500) {
//        Serial.println("Identification timeout");
//
//        if (stateMach.lineData.foundLineRight && stateMach.lineData.foundLineLeft) {
//          nav.setLastIntersection(Intersection::ThreeWayTee); // Shouldn't happen
//        }
//
//        // Cleanup
//        stateMach.lineData.identifyingIntersection = false;
//        stateMach.lineData.foundLineForward = false;
//        stateMach.lineData.foundLineLeft = false;
//        stateMach.lineData.foundLineRight = false;
//
//      }
    }

  } else {
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

        if (amountTurned >= 30) {
           stateMach.lineData.doneIntersectionFirstTurnStep = true;
        }

        if (stateMach.lineData.doneIntersectionFirstTurnStep) {
          Serial.println("30 degrees done, continuing turn");
          if (stateMach.lineData.lineDetected) {
            stop();
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

//      if (newTurn == TurnDirection::Left) {
//        turnLeft();
//      } else if (newTurn == TurnDirection::Right) {
//        turnRight();
//      } else {
//        forward();
//      }
//
//      if (stateMach.lineData.lineDetected) {
//        forward();
//        stateMach.lineData.intersectionHandled = true;
//        nav.setLastTurn(newTurn);
//      }
    } else {
      Serial.println("Following line normally");
      if (stateMach.lineData.linePosition < 2000) {
        Serial.println("Turning Right");
        turnRight();
      } else if (stateMach.lineData.linePosition > 5000) {
        Serial.println("Turning Left");
        turnLeft();
      } else {
        forward();
      }
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
  if (!stateMach.lineData.lineDetected && stateMach.lineData.followingLine && !stateMach.lineData.identifyingIntersection && stateMach.lineData.intersectionHandled) {
    Serial.println("Line lost, identifying intersection");
    stateMach.lineData.foundLineForward = false;
    stateMach.lineData.foundLineLeft = false;
    stateMach.lineData.foundLineRight = false;
    stateMach.lineData.identifyingIntersection = true;
    stateMach.lineData.intersectionHandled = false;
    stateMach.lineData.bestGuess = Intersection::Booth;
    stateMach.lineData.timeStartIdentIntersection = millis();
  }

  stateMach.lineData.linePosition = position;

  bool leftEnded = false;
  if (digitalRead(leftLineAuxPin) == HIGH) {
//    Serial.println("FOUND LINE LEFT =========================================");
    stateMach.lineData.foundLineLeft = true;
  } else {
    leftEnded = stateMach.lineData.foundLineLeft;
  }
  
  bool rightEnded = false;
  if (digitalRead(rightLineAuxPin) == HIGH) {
//    Serial.println("Found line right");
    stateMach.lineData.foundLineRight = true;
  } else {
    rightEnded = stateMach.lineData.foundLineRight;
  }

  if (leftEnded && rightEnded) {
    stateMach.saveAligningStartHeading();
    stateMach.lineData.identifyingIntersection = false;
    stateMach.lineData.foundLineForward = false;
    stateMach.lineData.foundLineLeft = false;
    stateMach.lineData.foundLineRight = false;
  }

  // assumption: we're hitting intersections reasonably straight, if not foundLine* could be true in future
  if ((leftEnded && !stateMach.lineData.foundLineRight) || (rightEnded && !stateMach.lineData.foundLineLeft)) {
    stateMach.saveAligningStartHeading();
    stateMach.lineData.identifyingIntersection = false;
    stateMach.lineData.foundLineForward = false;
    stateMach.lineData.foundLineLeft = false;
    stateMach.lineData.foundLineRight = false;
  }

}

//void pollPhotoresistor() {
//  stateMach.lightData.lightReading = analogRead(photoPin);
//  Serial.print("Light Resistance: "); Serial.println(stateMach.lightData.lightReading);
//}

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
