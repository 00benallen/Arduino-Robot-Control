#include "Navigator.h"

Navigator::Navigator(bool debug) {
  this->debug = debug;

  if (this->debug) {
    this->printDebugMessage("initialized");
  }
}

void Navigator::setLastIntersection(Intersection last) {
  
  this->lastDetectedIntersection = last;
  this->printDebugMessage("New intersection detected:");
  this->printDebugMessage(intersectionToString(last));
}

TurnDirection Navigator::getNextTurn() {
  TurnDirection recommendation = TurnDirection::None;
  switch (this->lastDetectedIntersection) {
    case Intersection::LeftCorner: {
      
      recommendation = TurnDirection::Left;
      
    } 
    break;
    case Intersection::RightCorner: {
      
      recommendation = TurnDirection::Right;
      
    }
    break;
    case Intersection::ThreeWayRight: {

      recommendation = TurnDirection::Right;
      
    }
    break;
    case Intersection::ThreeWayLeft: {

      recommendation = TurnDirection::Left;
      
    }
    break;
    case Intersection::ThreeWayTee: {

      if (lastTurn == TurnDirection::Left) {
        recommendation = TurnDirection::Right;
      } else {
        recommendation = TurnDirection::Left;
      }
      
    }
    break;
    case Intersection::FourWay: {

      if (lastTurn == TurnDirection::Left) {
        recommendation = TurnDirection::Right;
      } else {
        recommendation = TurnDirection::Left;
      }
      
    }
    break;
  }
  this->printDebugMessage("New turn recommendation: ");
  this->printDebugMessage(turnToString(recommendation));
  return recommendation;
}

void Navigator::printDebugMessage(String message) {
  if (this->debug) {
    Serial.print("[Navigator]: "); Serial.println(message);
  }
}

void Navigator::setLastTurn(TurnDirection d) {
  this->lastTurn = d;
}

String turnToString(TurnDirection d) {
  switch (d) {
    case TurnDirection::Left: {
       return "Left";
    }
    break;
    case TurnDirection::Right: {
       return "Right";
    }
    break;
    case TurnDirection::None: {
       return "None";
    }
    break;
  }
}
