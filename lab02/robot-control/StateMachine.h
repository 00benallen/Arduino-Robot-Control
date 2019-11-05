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

class IMUData {
  public:
    float lastDetectedHeading = 0;
};

class LineFollowingData {
  public:
    Direction currentTurningDirection = Direction::Left;
    Direction lastLineSearchDirection = Direction::Left;
    unsigned long timeElapsedLineForward = 0;
    unsigned long timeElapsedLineTurn = 0;
    bool followingLine = false;
    bool lineDetected = false;
    bool ignoreLine = false;
    unsigned int lineTurnAmount = 0;
    unsigned int lineSearchTurns = 0;
};

class MovementData {
  public:
    unsigned long timeSinceLastForwardCheck = 0;
    Direction clearTurnDirection = Direction::Left;
    int backupTimeElapsed = 0;
    float aligningStartHeading = 0;
    int currentAligningValue = 0;
    bool motorsEnabled = false;
    int randomTurnAmount = 90;
};

class ServoData {
  public:
    unsigned long timeOfLastSweep = 0;
    unsigned int servoAngle = 0;
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
    ServoData servoData;

    // Transition Events
    void emergencyBackupComplete();
    void lineDetected();
    void aligningWithClearPathComplete();
    void edgeDetected();
    void objectDetected();
    void lightDetected();
    void checkForwardForObjectsClear();
    void forwardCheckTimeout();
    void lineLostWhileFollowing();
    void endOfLineDetected();

  private:
    void saveAligningStartHeading();
    void printStateTransition(String message);
    String getCurrentStateLabel();
};

#endif
