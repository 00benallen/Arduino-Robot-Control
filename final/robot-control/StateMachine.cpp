#include "StateMachine.h"

unsigned int FOR_CHECK_DELAY_NORMAL = 300;
unsigned int FOR_CHECK_DELAY_LINE = 1000;
unsigned int FOR_CHECK_DELAY_LIGHT = 100;

StateMachine::StateMachine(State initialState) {
  this->currentState = initialState;
  this->moveData.forwardCheckDelay = FOR_CHECK_DELAY_NORMAL;
}

void StateMachine::emergencyBackupComplete() {
  String transitionMessage = "Emergency backup finished, beginning search for clear path in direction: ";
  if (this->moveData.clearTurnDirection == Direction::Right)
  {
    transitionMessage.concat("right");
  }
  else
  {
    transitionMessage.concat("left");
  }
  this->printStateTransition(transitionMessage);

  this->moveData.backupTimeElapsed = 0;
  this->saveAligningStartHeading();
  this->currentState = State::AligningWithClearPath;
  this->moveData.randomTurnAmount = random(10, 90);
}

void StateMachine::lineDetected() {
  this->printStateTransition("Line detected");
  this->saveAligningStartHeading();
  this->lineData.lineSearchTurns = 0;
  this->currentState = State::FollowingLine;
  this->moveData.forwardCheckDelay = FOR_CHECK_DELAY_LINE;
}

void StateMachine::aligningWithClearPathComplete() {
  this->moveData.randomTurnAmount = random(10, 170);
  this->printStateTransition("Clear path alignment finished, starting to find a forward path");
  this->currentState = State::CheckForwardForObjects;
  this->lineData.ignoreLine = false;
  this->moveData.forwardCheckDelay = FOR_CHECK_DELAY_NORMAL;
}

void StateMachine::edgeDetected() {
  if (this->servoData.servoAngle <= 90) {
    this->printStateTransition("Edge on right side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
    this->moveData.clearTurnDirection = Direction::Left; // turn the other way
  } else {
    this->printStateTransition("Edge on left side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
    this->moveData.clearTurnDirection = Direction::Right; // turn the other way
  }
  this->currentState = State::EmergencyBackup;
  this->lineData.ignoreLine = true;
  this->lineData.followingLine = false;
}

void StateMachine::objectDetected() {
  if (this->servoData.servoAngle <= 90) {
    this->moveData.clearTurnDirection = Direction::Left; // turn the other way
    this->printStateTransition("Object on right side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
  } else {
    this->moveData.clearTurnDirection = Direction::Right; // turn the other way
    this->printStateTransition("Object on left side detected during sweep by IR sensor, reseting servo and initiating emergency backup");
  }
  this->saveAligningStartHeading();
  this->currentState = State::AligningWithClearPath;
  this->moveData.randomTurnAmount = random(10, 90);
  this->lineData.ignoreLine = true;
  this->lineData.followingLine = false;
}

void StateMachine::checkForwardForObjectsClear() {
  this->printStateTransition("Sweep complete, path forward clear, continuing forward");
  this->currentState = State::MovingStraight; 
}

void StateMachine::forwardCheckTimeout() {
  this->printStateTransition("Forward path no longer guaranteed clear, rechecking");
  this->currentState = State::CheckForwardForObjects;
}

void StateMachine::lineLostWhileFollowing() {
  this->printStateTransition("Line lost");
  this->saveAligningStartHeading();
  this->lineData.lineTurnAmount = 45;
  this->lineData.currentTurningDirection = this->lineData.lastLineSearchDirection;
  this->currentState = State::FindingLine;
}

void StateMachine::endOfLineDetected() {
  this->printStateTransition("Line not detected, line must've ended");
  this->currentState = State::CheckForwardForObjects;
  this->lineData.followingLine = false;
  this->lineData.lineSearchTurns = 0;
}

void StateMachine::lightDetected() {
  this->printStateTransition("Light detected");
  Serial.println(this->lightData.lightAngle);
  if (this->lightData.lightAngle >= 80 && this->lightData.lightAngle <= 100) {
    this->lightData.linedUp = true;
  }
  else if (this->lightData.lightAngle > 90) {
    this->lightData.turnDirection = Direction::Left; // turn the other way
    this->lightData.linedUp = false;
  } else {
    this->lightData.turnDirection = Direction::Right; // turn the other way
    this->lightData.linedUp = false;
  }
  this->currentState = State::FollowingLight;
  this->servoData.servoAngle = 90;
  this->lineData.ignoreLine = true;
  this->saveAligningStartHeading();
  this->moveData.forwardCheckDelay = FOR_CHECK_DELAY_LIGHT;
}

void StateMachine::lightLost() {
  this->printStateTransition("Sweep complete, path forward clear, continuing forward");
  this->currentState = State::CheckForwardForObjects; 
  this->lineData.ignoreLine = false;
}

void StateMachine::saveAligningStartHeading() {
  this->moveData.aligningStartHeading = this->imuData.lastDetectedHeading;
}

void StateMachine::printStateTransition(String message) {
  Serial.print(this->getCurrentStateLabel()); Serial.println(message);
}

String StateMachine::getCurrentStateLabel() {
  switch(this->currentState) {
    case State::MovingStraight: return "[MovingStraight]: ";
    case State::AligningWithClearPath: return "[AligningWithClearPath]: ";
    case State::EmergencyBackup: return "[EmergencyBackup]: ";
    case State::CheckForwardForObjects: return "[CheckForwardForObjects]: ";
    case State::FollowingLine: return "[FollowingLine]: ";
    case State::FindingLine: return "[FindingLine]: ";
    case State::FollowingLight: return "[FollowingLight]: ";
    case State::Error: return "[Error]: ";
  }
}

String intersectionToString(Intersection i) {
  switch (i) {
    case Intersection::Booth: {
       return "Booth";
    }
    break;
    case Intersection::RightCorner: {
       return "RightCorner";
    }
    break;
    case Intersection::LeftCorner: {
       return "LeftCorner";
    }
    break;
    case Intersection::ThreeWayLeft: {
       return "ThreeWayLeft";
    }
    break;
    case Intersection::ThreeWayRight: {
       return "ThreeWayRight";
    }
    break;
    case Intersection::ThreeWayTee: {
       return "ThreeWayTee";
    }
    break;
    case Intersection::FourWay: {
       return "FourWay";
    }
    break;
  }
}
