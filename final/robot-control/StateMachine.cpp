#include "StateMachine.h"

unsigned int FOR_CHECK_DELAY_NORMAL = 300;
unsigned int FOR_CHECK_DELAY_LINE = 1000;
unsigned int FOR_CHECK_DELAY_LIGHT = 100;

StateMachine::StateMachine(State initialState) {
  this->currentState = initialState;
}

void StateMachine::lineDetected() {
  this->printStateTransition("Line detected");
  this->saveAligningStartHeading();
  this->currentState = State::FollowingLine;
}

void StateMachine::lineLostWhileFollowing() {
  this->printStateTransition("Line lost");
  this->saveAligningStartHeading();
  this->currentState = State::FindingLine;
}

void StateMachine::endOfLineDetected() {
  this->printStateTransition("Line not detected, line must've ended");
  this->currentState = State::CheckForwardForObjects;
  this->lineData.followingLine = false;
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
