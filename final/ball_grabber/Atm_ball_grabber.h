#pragma once

#include <Automaton.h>
#include <Servo.h>

class Atm_ball_grabber: public Machine {

 public:
  enum { IDLE, WAITING_TO_GRAB, HOLDING_ON, RELEASING }; // STATES
  enum { EVT_START, EVT_STOP, EVT_RELEASE, EVT_GRAB, ELSE }; // EVENTS
  Atm_ball_grabber( void ) : Machine() {};
  Atm_ball_grabber& begin( int sensorPin );
  Atm_ball_grabber& trace( Stream & stream );
  Atm_ball_grabber& trigger( int event );
  int state( void );
  Atm_ball_grabber& onBallgrabbed( Machine& machine, int event = 0 );
  Atm_ball_grabber& onBallgrabbed( atm_cb_push_t callback, int idx = 0 );
  Atm_ball_grabber& onBalllost( Machine& machine, int event = 0 );
  Atm_ball_grabber& onBalllost( atm_cb_push_t callback, int idx = 0 );
  Atm_ball_grabber& start( void );
  Atm_ball_grabber& stop( void );
  Atm_ball_grabber& release( void );
  Atm_ball_grabber& grab( void );

 private:
  enum { ENT_IDLE, ENT_WAITING_TO_GRAB, LP_WAITING_TO_GRAB, ENT_HOLDING_ON, LP_HOLDING_ON, ENT_RELEASING, LP_RELEASING }; // ACTIONS
  enum { ON_BALLGRABBED, ON_BALLLOST, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id );
  Servo claw;
  int sensorPin;
  bool ballDetected = false;
   

};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_ball_grabber">
    <states>
      <IDLE index="0" on_enter="ENT_IDLE">
        <EVT_START>WAITING_TO_GRAB</EVT_START>
      </IDLE>
      <WAITING_TO_GRAB index="1" on_enter="ENT_WAITING_TO_GRAB" on_loop="LP_WAITING_TO_GRAB">
        <EVT_STOP>IDLE</EVT_STOP>
        <EVT_RELEASE>RELEASING</EVT_RELEASE>
        <EVT_GRAB>HOLDING_ON</EVT_GRAB>
      </WAITING_TO_GRAB>
      <HOLDING_ON index="2" on_enter="ENT_HOLDING_ON" on_loop="LP_HOLDING_ON">
        <EVT_START>WAITING_TO_GRAB</EVT_START>
        <EVT_STOP>IDLE</EVT_STOP>
        <EVT_RELEASE>RELEASING</EVT_RELEASE>
      </HOLDING_ON>
      <RELEASING index="3" on_enter="ENT_RELEASING" on_loop="LP_RELEASING">
        <EVT_START>WAITING_TO_GRAB</EVT_START>
        <EVT_STOP>IDLE</EVT_STOP>
        <EVT_RELEASE>RELEASING</EVT_RELEASE>
        <EVT_GRAB>HOLDING_ON</EVT_GRAB>
      </RELEASING>
    </states>
    <events>
      <EVT_START index="0" access="MIXED"/>
      <EVT_STOP index="1" access="PUBLIC"/>
      <EVT_RELEASE index="2" access="PUBLIC"/>
      <EVT_GRAB index="3" access="MIXED"/>
    </events>
    <connectors>
      <BALLGRABBED autostore="0" broadcast="0" dir="PUSH" slots="1"/>
      <BALLLOST autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/
