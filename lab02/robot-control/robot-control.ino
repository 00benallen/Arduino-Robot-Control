/** Servo **/
#include <Servo.h>
Servo forwardServo;
int servoPosPin = 10;

/** Servo Constants **/
unsigned int FOR_CHECK_DELAY_NORMAL = 350;
unsigned int FOR_CHECK_DELAY_LINE = 500;
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

// IR Sensors
#include "Sensors.h"
int IRSensorLineLeftOutPin1 = 33;
int IRSensorLineRightOutPin1 = 53;
int IRSensorLineLeftOutPin2 = 35;
int IRSensorLineRightOutPin2 = 30;

IRSensor forwardLineRightSensor1(IRSensorLineLeftOutPin1);
IRSensor forwardLineLeftSensor1(IRSensorLineRightOutPin1);
IRSensor forwardLineRightSensor2(IRSensorLineLeftOutPin2);
IRSensor forwardLineLeftSensor2(IRSensorLineRightOutPin2);

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

  intializeIRs();

  Serial.println("Initializing auxilliary pins");
  pinMode(randomSeedPin, INPUT);
  randomSeed(analogRead(randomSeedPin));

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

void intializeIRs() {
  Serial.println("Initializing IR sensors");
  forwardLineRightSensor1.init();
  forwardLineLeftSensor1.init();
  forwardLineRightSensor2.init();
  forwardLineLeftSensor2.init();
}

void initializeServo() {
  Serial.println("Initializing forward servo");
  forwardServo.attach(servoPosPin);
  stateMach.servoData.timeOfLastSweep = millis();
  forwardServo.write(90);
}

void loop()
{

  long loopStart = millis(); // useful for timing of various processes in run loop
  pollIMU();
  if (!stateMach.lineData.ignoreLine) {
    pollLineSensor();
  }

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

          if (stateMach.lineData.followingLine && !stateMach.lineData.ignoreLine) { // line detected, and we aren't supposed to ignore it
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
        Serial.println("CheckForwardForObjects: Forward sweep starting, stopping robot until path forward is guaranteed clear");
        stop();
        stateMach.servoData.servoAngle = 0;
        bool edgeDetected = false;
        while (stateMach.servoData.servoAngle <= 180) {
          forwardServo.write(stateMach.servoData.servoAngle);
          stateMach.servoData.servoAngle += 5;

          unsigned long distanceCM = forwardEdgeSensor.read(CM);

          if (distanceCM > EDGE_DET_THRES) {
            stateMach.edgeDetected();
            edgeDetected = true;
            break;
          }

          VL53L0X_RangingMeasurementData_t measure;
          lox.rangingTest(&measure, true); // pass in 'true' to get debug data printout!

          if (measure.RangeStatus == 2 && measure.RangeMilliMeter <= 200) {
            stateMach.objectDetected();
            edgeDetected = true;
            break;
          }

          delay(SWEEP_DELAY);
        }

        stateMach.servoData.servoAngle = 0;
        forwardServo.write(stateMach.servoData.servoAngle);
        stateMach.moveData.timeSinceLastForwardCheck = millis();

        if (!edgeDetected && !stateMach.lineData.followingLine) {
          stateMach.checkForwardForObjectsClear();
        } else if (stateMach.lineData.followingLine && !stateMach.lineData.ignoreLine) {
          stateMach.lineDetected();
        }
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

        if (stateMach.lineData.lineDetected) {
          stop();
          stateMach.lineDetected();
          break;
        }

        float amountTurned = calculateAmountTurned();

        if (amountTurned <= stateMach.lineData.lineTurnAmount) {
          if (stateMach.lineData.currentTurningDirection == Direction::Left) {
            Serial.println("FindingLine: Line not detected, looking, turning left");
            stateMach.lineData.lastLineSearchDirection = Direction::Left;
            turnLeft();
          } else {
            Serial.println("FindingLine: Line not detected, looking, turning right");
            stateMach.lineData.lastLineSearchDirection = Direction::Right;
            turnRight();
          }
        } else {
          stateMach.lineData.lineSearchTurns++;
          if (stateMach.lineData.lineSearchTurns == 1) {
            Serial.println("FindingLine: Line not detected, looking, switching turn direction");
            if (stateMach.lineData.currentTurningDirection == Direction::Right) {
              stateMach.lineData.currentTurningDirection = Direction::Left;
            } else {
              stateMach.lineData.currentTurningDirection = Direction::Right;
            }
            stateMach.lineData.lineTurnAmount = 90;
            stateMach.moveData.aligningStartHeading = stateMach.imuData.lastDetectedHeading;
          } else if (stateMach.lineData.lineSearchTurns == 2) {
            stateMach.endOfLineDetected();
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

  SensorResult lineRight1 = forwardLineRightSensor1.getResult();
  SensorResult lineLeft1 = forwardLineLeftSensor1.getResult();
  SensorResult lineRight2 = forwardLineRightSensor2.getResult();
  SensorResult lineLeft2 = forwardLineLeftSensor2.getResult();

  stateMach.lineData.lineDetected = false;
  if (lineRight1 == SensorResult::Nothing || lineLeft1 == SensorResult::Nothing || lineRight2 == SensorResult::Nothing || lineRight2 == SensorResult::Nothing) {
    stateMach.lineData.lineDetected = true;
    stateMach.lineData.followingLine = true;
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
