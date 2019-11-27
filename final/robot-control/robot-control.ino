/** Motors **/
#include "Atm_motors.h"
Atm_motors motors(3, 4, 2, 1);

/** Motor Constants **/
const int MOTOR_SPEED_LOW = 120;
const int MOTOR_SPEED_HIGH = 255;

/** Sensors **/

// 9-Axis IMU
#include "Atm_imu.h"
Atm_imu IMU;

// IR TOF Sensor
//#include "Adafruit_VL53L0X.h"
//Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#include "Atm_line_navigator.h"
Atm_line_navigator lineNav;

#include "Atm_flame_follower.h"
Atm_flame_follower flameFollower;

//// Fire Sensors
//int leftFirePin = 30;
//int rightFirePin = 31;
//int forwardFirePin = 29;

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

  motors.begin(MOTOR_SPEED_LOW).trace( Serial );
//  motors.enable(); // comment out to disable motors

  IMU.begin().onTurnend(lineNav, Atm_line_navigator::EVT_INTERSECTION_TURN_COMPLETE).trace( Serial );

  //  initializeAuxIRs();

//  initializeFires();

  //  initializeTOF();

  //  initializeQTR();

  flameFollower.begin(30, 31, 29)
  .trace ( Serial )
  .onMotorChange( [] (int idx, int v, int up) {
    switch (v) {
      case Atm_line_navigator::MOTOR_LEFT:
        motors.left();
        return;
      case Atm_line_navigator::MOTOR_RIGHT:
        motors.right();
        return;
      case Atm_line_navigator::MOTOR_FORWARD:
        motors.forward();
        return;
      case Atm_line_navigator::MOTOR_STOP:
        motors.stop();
        return;
    }
    return;
  })
  .onFlamedetected( lineNav, Atm_line_navigator::EVT_STOP )
  .onFlamehandled( lineNav, Atm_line_navigator::EVT_START )
  .trigger ( Atm_flame_follower::EVT_START );

  lineNav.begin((const uint8_t[]) {
    53, 51, 49, 47, 45, 43, 41, 39
  }, 52, 22)
  .onMotorChange( [] (int idx, int v, int up) {
    switch (v) {
      case Atm_line_navigator::MOTOR_LEFT:
        motors.left();
        return;
      case Atm_line_navigator::MOTOR_RIGHT:
        motors.right();
        return;
      case Atm_line_navigator::MOTOR_FORWARD:
        motors.forward();
        return;
      case Atm_line_navigator::MOTOR_STOP:
        motors.stop();
        return;
    }
    return;
  })
  .onTurnStart([] (int idx, int v, int up) {
    IMU.track_turn(v);
  })
  .trace( Serial )
  .start();

  
}

//void initializeFires() {
//  Serial.println("Flame Sensors");
//  pinMode(leftFirePin, INPUT);
//  pinMode(rightFirePin, INPUT);
//  pinMode(forwardFirePin, INPUT);
//}

//void initializeTOF() {
//  Serial.println("Initializing VL53L0X laser TOF sensor");
//  Serial.println("Adafruit VL53L0X test");
//  while (!lox.begin()) {
//    Serial.println(F("Failed to boot VL53L0X"));
//    delay(1000);
//  }
//  Serial.println("VL53L0X laser TOF sensor detected");
//}

void loop()
{
//  motors.disable(); // comment out if you want to move during loops
  automaton.run();

  long loopStart = millis(); // useful for timing of various processes in run loop
//  pollFlameSensors();

//  if (stateMach.flameData.leftFlameDet || stateMach.flameData.rightFlameDet || stateMach.flameData.forwardFlameDet || stateMach.flameData.aligningWithFlame) {
//
//    if (!stateMach.flameData.aligningWithFlame) {
//      motors.stop();
//      Serial.println("Fire detected, stop");
//      delay(500);
//    } else {
//      if (stateMach.flameData.leftFlameDet) {
//        motors.left();
//      } else if (stateMach.flameData.rightFlameDet) {
//        motors.right();
//      }
//
//      if (stateMach.flameData.forwardFlameDet) {
//        motors.stop();
//        delay(500);
//        stateMach.flameData.aligningWithFlame = false;
//        return;
//      }
//    }
//    stateMach.flameData.aligningWithFlame = true;
//  }
  //  else {

  //    if (stateMach.lineData.lineLeftEnded || stateMach.lineData.lineRightEnded) {
  //
  //      stateMach.lineData.intersectionHandled = false;
  //      stateMach.lineData.pulledAwayFromIntersection = false;
  //      stateMach.saveAligningStartHeading();
  //      Serial.print("Forward: "); Serial.println(stateMach.lineData.foundLineForward);
  //      Serial.print("Left: "); Serial.println(stateMach.lineData.foundLineLeft);
  //      Serial.print("Right: "); Serial.println(stateMach.lineData.foundLineRight);
  //
  //      if (stateMach.lineData.foundLineForward) {
  //
  //        if (stateMach.lineData.foundLineRight && stateMach.lineData.foundLineLeft) {
  //
  //          nav.setLastIntersection(Intersection::FourWay);
  //          stateMach.lineData.foundLineLeft = false;
  //          stateMach.lineData.foundLineRight = false;
  //          stateMach.lineData.foundLineForward = false;
  //          stateMach.lineData.lineRightEnded = false;
  //          stateMach.lineData.lineLeftEnded = false;
  //
  //        } else if (stateMach.lineData.foundLineRight) {
  //
  //          nav.setLastIntersection(Intersection::ThreeWayRight);
  //          stateMach.lineData.foundLineLeft = false;
  //          stateMach.lineData.foundLineRight = false;
  //          stateMach.lineData.foundLineForward = false;
  //          stateMach.lineData.lineRightEnded = false;
  //          stateMach.lineData.lineLeftEnded = false;
  //
  //        } else if (stateMach.lineData.foundLineLeft) {
  //
  //          nav.setLastIntersection(Intersection::ThreeWayLeft);
  //          stateMach.lineData.foundLineLeft = false;
  //          stateMach.lineData.foundLineRight = false;
  //          stateMach.lineData.foundLineForward = false;
  //          stateMach.lineData.lineRightEnded = false;
  //          stateMach.lineData.lineLeftEnded = false;
  //
  //        } else {
  //          nav.setLastIntersection(Intersection::Booth);
  //          stateMach.lineData.foundLineLeft = false;
  //          stateMach.lineData.foundLineRight = false;
  //          stateMach.lineData.foundLineForward = false;
  //          stateMach.lineData.lineRightEnded = false;
  //          stateMach.lineData.lineLeftEnded = false;
  //        }
  //
  //      } else {
  //
  //        if (stateMach.lineData.foundLineLeft && stateMach.lineData.foundLineRight) {
  //
  //          nav.setLastIntersection(Intersection::ThreeWayTee);
  //          stateMach.lineData.foundLineLeft = false;
  //          stateMach.lineData.foundLineRight = false;
  //          stateMach.lineData.foundLineForward = false;
  //          stateMach.lineData.lineRightEnded = false;
  //          stateMach.lineData.lineLeftEnded = false;
  //
  //        } else if (stateMach.lineData.foundLineRight) {
  //
  //          nav.setLastIntersection(Intersection::RightCorner);
  //          stateMach.lineData.foundLineLeft = false;
  //          stateMach.lineData.foundLineRight = false;
  //          stateMach.lineData.foundLineForward = false;
  //          stateMach.lineData.lineRightEnded = false;
  //          stateMach.lineData.lineLeftEnded = false;
  //
  //        } else if (stateMach.lineData.foundLineLeft) {
  //
  //          nav.setLastIntersection(Intersection::LeftCorner);
  //          stateMach.lineData.foundLineLeft = false;
  //          stateMach.lineData.foundLineRight = false;
  //          stateMach.lineData.foundLineForward = false;
  //          stateMach.lineData.lineRightEnded = false;
  //          stateMach.lineData.lineLeftEnded = false;
  //
  //        } else {
  //
  //          nav.setLastIntersection(Intersection::Booth);
  //          stateMach.lineData.foundLineLeft = false;
  //          stateMach.lineData.foundLineRight = false;
  //          stateMach.lineData.foundLineForward = false;
  //          stateMach.lineData.lineRightEnded = false;
  //          stateMach.lineData.lineLeftEnded = false;
  //
  //        }
  //      }
  //    }
  //
  //    //Get turn recommendation
  //    if (!stateMach.lineData.intersectionHandled) {
  //      TurnDirection newTurn = nav.getNextTurn();
  //
  //      if (newTurn != TurnDirection::None) {
  //        if (IMU.turnComplete) { // TODO replace with better inter-automata comms later
  //          IMU.track_turn(45);
  //        }
  //
  //        if (newTurn == TurnDirection::Left) {
  //          Serial.println("Turning left");
  //          motors.left();
  //        } else if (newTurn == TurnDirection::Right) {
  //          Serial.println("Turning right");
  //          motors.right();
  //        }
  //
  //        if (IMU.turnComplete && !stateMach.lineData.doneIntersectionFirstTurnStep) {
  //          stateMach.lineData.doneIntersectionFirstTurnStep = true;
  //          motors.forward();
  //          delay(400);
  //        }
  //
  //        if (stateMach.lineData.doneIntersectionFirstTurnStep) {
  //          Serial.println("45 degrees done, continuing turn");
  //
  //          if ((int)(IMU.angleAbsDiff()) % 1 == 0) {
  //            motors.stop();
  //            delayMicroseconds(3);
  //          }
  //
  //          if (stateMach.lineData.lineDetected) {
  //            Serial.println("Line found, ending turn");
  //            stateMach.lineData.doneIntersectionFirstTurnStep = false;
  //            stateMach.lineData.intersectionHandled = true;
  //            nav.setLastTurn(newTurn);
  //          }
  //        }
  //      } else {
  //        Serial.println("No turn recommended, continuing straight");
  //        motors.stop();
  //        stateMach.lineData.intersectionHandled = true;
  //        nav.setLastTurn(newTurn);
  //      }
  //    } else if (stateMach.lineData.lineDetected) {
  //      Serial.println("Following line normally");
  //      if (stateMach.lineData.linePosition < 1000) {
  //        Serial.println("Turning Right");
  //        motors.right();
  //      } else if (stateMach.lineData.linePosition > 6000) {
  //        Serial.println("Turning Left");
  //        motors.left();
  //      } else {
  //        stateMach.lineData.pulledAwayFromIntersection = true;
  //        motors.forward();
  //      }
  //    } else {
  //      motors.forward();
  //    }
  //
  //  }
}

//void pollLineSensors() {
//
//  // read calibrated sensor values and obtain a measure of the line position
//  // from 0 to 5000 (for a white line, use readLineWhite() instead)
//  int sensorCount = 8;
//  unsigned int sensorValues[8];
//  uint16_t position = qtr.readLineBlack(sensorValues);
//  memcpy(stateMach.lineData.rawLineValues, sensorValues, 8 * sizeof(unsigned int));
//
//  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
//  // reflectance and 1000 means minimum reflectance, followed by the line
//  // position
//  stateMach.lineData.lineDetected = false;
//  for (uint8_t i = 0; i < 8; i++)
//  {
//    unsigned int curReading = sensorValues[i];
//    Serial.print(curReading); Serial.print(" ");
//    if (curReading > 700) {
//      stateMach.lineData.lineDetected = true;
//      stateMach.lineData.followingLine = true;
//
//      if (i >= 2 && i <= 5) {
//        stateMach.lineData.foundLineForward = true;
//      }
//    }
//  }
//
//  Serial.print(position);
//  Serial.println();
//
//  stateMach.lineData.linePosition = position;
//
//  if (stateMach.lineData.intersectionHandled && stateMach.lineData.pulledAwayFromIntersection) {
//    if (digitalRead(leftLineAuxPin) == HIGH) {
//      Serial.println("FOUND LINE LEFT =========================================");
//      stateMach.lineData.foundLineLeft = true;
//    } else {
//      stateMach.lineData.lineLeftEnded = stateMach.lineData.foundLineLeft;
//    }
//
//    if (digitalRead(rightLineAuxPin) == HIGH) {
//      Serial.println("Found line right =========================================");
//      stateMach.lineData.foundLineRight = true;
//    } else {
//      stateMach.lineData.lineRightEnded = stateMach.lineData.foundLineRight;
//    }
//  }
//}

//void pollFlameSensors() {
//  if (digitalRead(leftFirePin) == LOW) {
//    Serial.println("Left fire detected");
//    stateMach.flameData.leftFlameDet = true;
//  } else {
//    stateMach.flameData.leftFlameDet = false;
//  }
//
//  if (digitalRead(rightFirePin) == LOW) {
//    Serial.println("Right fire detected");
//    stateMach.flameData.rightFlameDet = true;
//  } else {
//    stateMach.flameData.rightFlameDet = false;
//  }
//
//  if (digitalRead(forwardFirePin) == LOW) {
//    Serial.println("Right fire detected");
//    stateMach.flameData.forwardFlameDet = true;
//  } else {
//    stateMach.flameData.forwardFlameDet = false;
//  }
//}

float calculateAmountTurned() {
  return 180 - abs(abs(stateMach.imuData.lastDetectedHeading - stateMach.moveData.aligningStartHeading) - 180);
}
