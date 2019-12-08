#pragma once

#include <Automaton.h>
#include <Smoothed.h>

class Atm_flame_follower: public Machine {

 public:
  enum { CALIBRATING, IDLE, HALTING_FOR_FLAME, TURNING_TOWARDS_FLAME, APPROACHING_FLAME, AVOIDING_BOOTH, DISABLED }; // STATES
  enum { EVT_START, EVT_STOP, EVT_SIDE_FLAME_DETECTED, EVT_HALTING_COMPLETE, EVT_FORWARD_FLAME_DETECTED, EVT_BOOTH_AVOIDED, EVT_BOOTH_TOO_CLOSE_RIGHT, EVT_BOOTH_TOO_CLOSE_LEFT, EVT_DONE_CALIBRATING, ELSE }; // EVENTS
  enum Direction { D_LEFT, D_RIGHT, D_FORWARD, D_NONE };
  enum { MOTOR_LEFT, MOTOR_RIGHT, MOTOR_FORWARD, MOTOR_STOP }; // Helpful enum for onMotorChange event processing
  Atm_flame_follower( void ) : Machine() {};
  Atm_flame_follower& begin( int leftPin, int rightPin, int forwardPin );
  Atm_flame_follower& trace( Stream & stream );
  Atm_flame_follower& trigger( int event );
  int state( void );
  Atm_flame_follower& onFlamedetected( Machine& machine, int event = 0 );
  Atm_flame_follower& onFlamedetected( atm_cb_push_t callback, int idx = 0 );
  Atm_flame_follower& onFlamehandled( Machine& machine, int event = 0 );
  Atm_flame_follower& onFlamehandled( atm_cb_push_t callback, int idx = 0 );
  Atm_flame_follower& onMotorChange( Machine& machine, int event = 0 );
  Atm_flame_follower& onMotorChange( atm_cb_push_t callback, int idx = 0 );
  Atm_flame_follower& onApproachingBall( Machine& machine, int event = 0 );
  Atm_flame_follower& onApproachingBall( atm_cb_push_t callback, int idx = 0 );
  Atm_flame_follower& start( void );
  Atm_flame_follower& stop( void );

 private:
  enum { LP_IDLE, ENT_CALIBRATING, ENT_HALTING_FOR_FLAME, LP_TURNING_TOWARDS_FLAME, ENT_APPROACHING_FLAME, LP_APPROACHING_FLAME, EXT_APPROACHING_FLAME, LP_AVOIDING_BOOTH, ENT_DISABLED }; // ACTIONS
  enum { ON_FLAMEDETECTED, ON_FLAMEHANDLED, ON_MOTOR_CHANGE, ON_APPROACHING_BALL, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id ); 
  int leftPin, rightPin, forwardPin;
  void pollFlameSensors();
  void calibrateSensors();
  bool forwardFlameDet, rightFlameDet, leftFlameDet;
  bool doneCalibrating;
  Direction flameSide = D_NONE;
  atm_timer_millis halt_timer;
  Smoothed <float> smoothedRight;
  Smoothed <float> smoothedLeft;
  Smoothed <float> smoothedForward;
  float ambientMinLeft = 1000, ambientMinRight = 1000, ambientMinForward = 1000;
  float ambientMaxLeft = 0, ambientMaxRight = 0, ambientMaxForward = 0;
//  float approachingPeak = 0;
//  Direction approachingTurn;
//  float rawForwardValues[5];
//  int rawForwardValuesInd;
//  bool peak = true;

};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_flame_follower">
    <states>
      <IDLE index="0" on_loop="LP_IDLE">
        <EVT_SIDE_FLAME_DETECTED>HALTING_FOR_FLAME</EVT_SIDE_FLAME_DETECTED>
      </IDLE>
      <HALTING_FOR_FLAME index="1" on_enter="ENT_HALTING_FOR_FLAME">
        <EVT_HALTING_COMPLETE>TURNING_TOWARDS_FLAME</EVT_HALTING_COMPLETE>
      </HALTING_FOR_FLAME>
      <TURNING_TOWARDS_FLAME index="2" on_loop="LP_TURNING_TOWARDS_FLAME">
        <EVT_FORWARD_FLAME_DETECTED>STOPPED_IN_FRONT_OF_FLAME</EVT_FORWARD_FLAME_DETECTED>
        <EVT_BOOTH_TOO_CLOSE_RIGHT>AVOIDING_BOOTH</EVT_BOOTH_TOO_CLOSE_RIGHT>
        <EVT_BOOTH_TOO_CLOSE_LEFT>AVOIDING_BOOTH</EVT_BOOTH_TOO_CLOSE_LEFT>
      </TURNING_TOWARDS_FLAME>
      <STOPPED_IN_FRONT_OF_FLAME index="3" on_enter="ENT_STOPPED_IN_FRONT_OF_FLAME">
      </STOPPED_IN_FRONT_OF_FLAME>
      <AVOIDING_BOOTH index="4" on_loop="LP_AVOIDING_BOOTH">
        <EVT_BOOTH_AVOIDED>TURNING_TOWARDS_FLAME</EVT_BOOTH_AVOIDED>
      </AVOIDING_BOOTH>
    </states>
    <events>
      <EVT_SIDE_FLAME_DETECTED index="0" access="MIXED"/>
      <EVT_HALTING_COMPLETE index="1" access="MIXED"/>
      <EVT_FORWARD_FLAME_DETECTED index="2" access="MIXED"/>
      <EVT_BOOTH_AVOIDED index="3" access="MIXED"/>
      <EVT_BOOTH_TOO_CLOSE_RIGHT index="4" access="MIXED"/>
      <EVT_BOOTH_TOO_CLOSE_LEFT index="5" access="MIXED"/>
    </events>
    <connectors>
      <FLAMEDETECTED autostore="0" broadcast="0" dir="PUSH" slots="1"/>
      <FLAMEHANDLED autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/
