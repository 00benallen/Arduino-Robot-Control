/**
 * Setup motors
 */

#include <AFMotor.h>
AF_DCMotor motorFrontRight(2);
AF_DCMotor motorBackRight(1);
AF_DCMotor motorFrontLeft(4);
AF_DCMotor motorBackLeft(3);

const int MOTOR_SPEED_LOW = 170;
const int MOTOR_SPEED_HIGH = 255;

/**
 * Other constants
 */
const int MAX_BACKUP_TIME = 500;
const int SENSOR_MAX_TIMEOUT = 500;

/**
 * Global variables
 */
int motorSpeed = MOTOR_SPEED_LOW;
int backupTimeElapsed = 0;
int edgeDetectionThreshold = 10;
int sensorTriggerPin = 53;
int sensorEchoPin = 52;

/**
 * Enum representing possible modes the robot can be in during the run loop
 */
enum class Mode {
  MovingStraight,
  FindingClearPath,
  EmergencyBackup,
  Error,
};
Mode runningMode = Mode::FindingClearPath;

/**
 * Enum representing the possible results the sensor data could indicate
 */
enum class SensorResult {
  Edge,
  Table,
  Error,
};

void setup() {

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
  pinMode(53, OUTPUT);
  pinMode(52, INPUT);
}

void loop() {

  long loopStart = millis(); // useful for timing of various processes in run loop
  
  SensorResult result = getSensorResult(sensorTriggerPin, sensorEchoPin, SENSOR_MAX_TIMEOUT);
  processSensorResult(result);

  // Main switch case for determining what behaviour the robot should be engaging in, see processSensorResult() as an example of some of the mode changes
  switch(runningMode) {
    case Mode::EmergencyBackup: { // reverse the robot for a certain amount of time, to ensure it is not going to go over the edge when it turns
      Serial.print("Backing up elasped time: ");
      Serial.print(backupTimeElapsed);
      Serial.println(" millis");
      backupTimeElapsed += millis() - loopStart;

      if (backupTimeElapsed < MAX_BACKUP_TIME) {
        backward();
      } else {
        Serial.println("Emergency backup finished, beginning search for clear path");
        backupTimeElapsed = 0;
        runningMode = Mode::FindingClearPath; 
      }
    }
    break;
    case Mode::FindingClearPath: { // turn in a circle to find a clear way forward
      turnRight(1); // TODO make random turn amount
    }
    break;
    case Mode::MovingStraight: { // way forward is clear, so go that way
      forward(); // todo make random
    }
    break;
    case Mode::Error: { // something is wrong with Arduino or its peripheral devices, stop to keep the robot safe
      Serial.println("Issue with setup, robot not safe to move!");
      stop();
    }
    break;
  }
}

SensorResult getSensorResult(int sonicPulsePin, int echoInputPin, int maxTimeout) {
  triggerSonicPulse(sonicPulsePin);
  long timeSpentHigh = waitForSignalUntilTimeout(echoInputPin, HIGH, 1000);

  if (timeSpentHigh >= maxTimeout) {
    return SensorResult::Error; //sensor not responding to trigger pulse
  }
  
  long echoStart = micros();
  long timeSpentLow = waitForSignalUntilTimeout(52, LOW, 1000);

  if (timeSpentHigh + timeSpentLow >= maxTimeout) {
    return SensorResult::Error; //sensor did not send echo signal back
  }
  
  long echoTime = micros() - echoStart;
  int distance = echoTime / 58; //centimeters

  if (distance > edgeDetectionThreshold) {
    return SensorResult::Edge;
  } else {
    return SensorResult::Table;
  }
}

// trigger 10us voltage pulse so sensor knows to start sound pulse
void triggerSonicPulse(int triggerP) {
  digitalWrite(triggerP, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerP, LOW);
}

// busy wait on a signal until it changes
long waitForSignalUntilTimeout(int pin, int level, int maxTimeout) {
  long startTime = millis();
  long elapsedTime = 0;
  while(digitalRead(pin) != level) {
    elapsedTime = millis() - startTime;
    if (elapsedTime >= maxTimeout) {break;}
  }
  return elapsedTime;
}

void processSensorResult(SensorResult result) {
  switch(result) {
    case SensorResult::Edge: {
      if (runningMode != Mode::FindingClearPath) {
        Serial.println("Edge detected, emergency backup commencing"); 
        runningMode = Mode::EmergencyBackup;
      }
    }
    break;
    case SensorResult::Table: {
      if (runningMode != Mode::MovingStraight) {
        Serial.println("Table detected, starting to move straight");
        runningMode = Mode::MovingStraight;
      }
    }
    break;
    case SensorResult::Error: {
      Serial.println("Forward sensor not working!");
      runningMode = Mode::Error;
    }; 
    break;
  }
}

void forward() {
  motorFrontRight.run(FORWARD);
  motorBackRight.run(FORWARD);
  motorFrontLeft.run(FORWARD);
  motorBackLeft.run(FORWARD);
}

void backward() {
  motorFrontRight.run(BACKWARD);
  motorBackRight.run(BACKWARD);
  motorFrontLeft.run(BACKWARD);
  motorBackLeft.run(BACKWARD);
}

void turnRight(int amount) {
  motorFrontRight.run(BACKWARD);
  motorBackRight.run(BACKWARD);
  motorFrontLeft.run(FORWARD);
  motorBackLeft.run(FORWARD);
  // TODO use amount
}

void turnLeft(int amount) {
  motorFrontRight.run(FORWARD);
  motorBackRight.run(FORWARD);
  motorFrontLeft.run(BACKWARD);
  motorBackLeft.run(BACKWARD);
}

void stop() {
  motorFrontRight.run(RELEASE);
  motorBackRight.run(RELEASE);
  motorFrontLeft.run(RELEASE);
  motorBackLeft.run(RELEASE);
}
