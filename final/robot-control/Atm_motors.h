#pragma once

#include <Automaton.h>
#include <AFMotor.h>

class Atm_motors: public Machine {

 public:
  enum { S_FORWARD, TURNING_LEFT, TURNING_RIGHT, S_BACKWARD, S_IDLE }; // STATES
  enum { EVT_FORWARD, EVT_BACKWARD, EVT_STOP, EVT_LEFT, EVT_RIGHT, ELSE }; // EVENTS
  Atm_motors( int f_right_p, int f_left_p, int b_right_p, int b_left_p) : 
    Machine(), 
    motorFrontRight(f_right_p), 
    motorBackRight(b_right_p),
    motorFrontLeft(f_left_p), 
    motorBackLeft(b_left_p) {};
  Atm_motors& begin( int motor_speed );
  Atm_motors& trace( Stream & stream );
  Atm_motors& trigger( int event );
  int state( void );
  Atm_motors& forward( void );
  Atm_motors& backward( void );
  Atm_motors& stop( void );
  Atm_motors& left( void );
  Atm_motors& right( void );
  Atm_motors& disable( void );
  Atm_motors& enable( void );

 private:
  enum { ENT_FORWARD, ENT_TURNING_LEFT, ENT_TURNING_RIGHT, ENT_BACKWARD, ENT_IDLE }; // ACTIONS
  int event( int id ); 
  void action( int id ); 

  // AFMotors to use as action targets
  AF_DCMotor motorFrontRight;
  AF_DCMotor motorBackRight;
  AF_DCMotor motorFrontLeft;
  AF_DCMotor motorBackLeft;
  int motorSpeed;
  bool enabled;

};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_motors">
    <states>
      <FORWARD index="0" on_enter="ENT_FORWARD">
        <EVT_BACKWARD>BACKWARD</EVT_BACKWARD>
        <EVT_STOP>IDLE</EVT_STOP>
        <EVT_LEFT>TURNING_LEFT</EVT_LEFT>
        <EVT_RIGHT>TURNING_RIGHT</EVT_RIGHT>
      </FORWARD>
      <TURNING_LEFT index="1" on_enter="ENT_TURNING_LEFT">
        <EVT_FORWARD>FORWARD</EVT_FORWARD>
        <EVT_BACKWARD>BACKWARD</EVT_BACKWARD>
        <EVT_STOP>IDLE</EVT_STOP>
        <EVT_RIGHT>TURNING_RIGHT</EVT_RIGHT>
      </TURNING_LEFT>
      <TURNING_RIGHT index="2" on_enter="ENT_TURNING_RIGHT">
        <EVT_FORWARD>FORWARD</EVT_FORWARD>
        <EVT_BACKWARD>BACKWARD</EVT_BACKWARD>
        <EVT_STOP>IDLE</EVT_STOP>
        <EVT_LEFT>TURNING_LEFT</EVT_LEFT>
      </TURNING_RIGHT>
      <BACKWARD index="3" on_enter="ENT_BACKWARD">
        <EVT_FORWARD>FORWARD</EVT_FORWARD>
        <EVT_STOP>IDLE</EVT_STOP>
        <EVT_LEFT>TURNING_LEFT</EVT_LEFT>
        <EVT_RIGHT>TURNING_RIGHT</EVT_RIGHT>
      </BACKWARD>
      <IDLE index="4" on_enter="ENT_IDLE">
        <EVT_FORWARD>FORWARD</EVT_FORWARD>
        <EVT_BACKWARD>BACKWARD</EVT_BACKWARD>
        <EVT_LEFT>TURNING_LEFT</EVT_LEFT>
        <EVT_RIGHT>TURNING_RIGHT</EVT_RIGHT>
      </IDLE>
    </states>
    <events>
      <EVT_FORWARD index="0" access="MIXED"/>
      <EVT_BACKWARD index="1" access="MIXED"/>
      <EVT_STOP index="2" access="MIXED"/>
      <EVT_LEFT index="3" access="MIXED"/>
      <EVT_RIGHT index="4" access="MIXED"/>
    </events>
    <connectors>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/
