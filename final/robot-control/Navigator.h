/*
  File for State Machine APIs
*/

#ifndef Navigator_h
#define Navigator_h

#include "Arduino.h"
#include "StateMachine.h"

enum class TurnDirection
{
  Left,
  Right,
  None,
};

String turnToString(TurnDirection d);

class Navigator {

  public:
  
    Navigator(bool debug);
    void setLastIntersection(Intersection last);
    TurnDirection getNextTurn();

  private:
    bool debug;
    Intersection lastDetectedIntersection;
    void printDebugMessage(String message);
  
};

#endif
