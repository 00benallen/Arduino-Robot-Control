/*
  File for State Machine APIs
*/

#ifndef StateMachine_h
#define StateMachine_h

#include "Arduino.h"

/**
   Enum representing the possible results the sensor data could indicate
*/
enum class State
{
  MovingStraight,        // robot is moving forward along a clear path
  AligningWithClearPath, // robot is aligning itself with a found clear path
  EmergencyBackup,       // robot is backing up (reversing) because an obstacle is directly in front
  CheckForwardForObjects,
  FollowingLine,
  FindingLine,
  FollowingLight,
  Error,                 // robot is in an error state, not ready/not safe to drive
};

/**
   Enum representing the two directions the robot cares about for edge-avoidance purposes
*/
enum class Direction
{
  Left,
  Right
};

enum class Intersection {
  Booth,
  RightCorner,
  LeftCorner,
  ThreeWayLeft,
  ThreeWayRight,
  ThreeWayTee,
  FourWay
};

String intersectionToString(Intersection i);

class IMUData {
  public:
    float lastDetectedHeading = 0;
};

class LineFollowingData {
  public:
    unsigned long timeStartIdentIntersection = 0;
    unsigned long timeElapsedLineTurn = 0;
    bool followingLine = false;
    bool lineDetected = false;
    unsigned int linePosition = 0;
    unsigned int rawLineValues[8];
    bool intersectionHandled = true;
    bool foundLineLeft = false;
    bool foundLineRight = false;
    bool foundLineForward = false;
    bool doneIntersectionFirstTurnStep = false;
    bool lineLeftEnded = false;
    bool lineRightEnded = false;
    bool pulledAwayFromIntersection = false;
};

class FlameFollowingData {
  public:
    bool leftFlameDet = false;
    bool rightFlameDet = false;
    bool forwardFlameDet = false;
    bool aligningWithFlame = false;
};

class MovementData {
  public:
    float aligningStartHeading = 0;
    bool motorsEnabled = true;
};

class StateMachine
{
  public:
    StateMachine(State initialState);

    // State data
    State currentState;
    IMUData imuData;
    LineFollowingData lineData;
    MovementData moveData;
    FlameFollowingData flameData;

    // Transition Events
    void lineDetected();
    void lineLostWhileFollowing();
    void endOfLineDetected();
    void saveAligningStartHeading();

  private:
    
    void printStateTransition(String message);
    String getCurrentStateLabel();
};

#endif
