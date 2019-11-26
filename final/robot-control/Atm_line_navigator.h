#pragma once

#include <Automaton.h>
#include "Atm_motors.h"
#include <QTRSensors.h>

class Atm_line_navigator: public Machine {

 public:
  enum { FOLLOW_LINE, IDENTIFY_INTERSECTION, INTERSECTION_TURN, INTERSECTION_ALIGN, IDLE }; // STATES
  enum { EVT_START, EVT_GAP_DETECTED, EVT_INTERSECTION_LEFT_DETECTED, EVT_INTERSECTION_RIGHT_DETECTED, EVT_STOP, ELSE }; // EVENTS
  enum { MOTOR_LEFT, MOTOR_RIGHT, MOTOR_FORWARD }; // Helpful enum for onMotorChange event processing
  Atm_line_navigator( ) : Machine() {};
  Atm_line_navigator& begin( const uint8_t qtr_pins[], int left_aux_pin, int right_aux_pin );
  Atm_line_navigator& trace( Stream & stream );
  Atm_line_navigator& trigger( int event );
  int state( void );
  Atm_line_navigator& onMotorChange( Machine& machine, int event = 0 );
  Atm_line_navigator& onMotorChange( atm_cb_push_t callback, int idx = 0 );
  Atm_line_navigator& start( void );
  Atm_line_navigator& stop( void );
  Atm_line_navigator& calibrate( void );

 private:
  enum { ENT_FOLLOW_LINE, LP_FOLLOW_LINE, ENT_IDENTIFY_INTERSECTION, LP_IDENTIFY_INTERSECTION, ENT_INTERSECTION_TURN, LP_INTERSECTION_TURN, ENT_INTERSECTION_ALIGN, LP_INTERSECTION_ALIGN }; // ACTIONS
  int event( int id ); 
  void action( int id );
  enum { ON_MOTOR_CHANGE, CONN_MAX }; // CONNECTORS
  
  atm_connector connectors[CONN_MAX];
  QTRSensors qtr;
  uint16_t linePosition;
  void pollLineSensors();
  bool lineUnderQtr;
  bool lineForwardDetected;
  bool lineLeftDetected;
  bool lineRightDetected;
  bool lineRightEnded;
  bool lineLeftEnded;
  
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_line_navigator">
    <states>
      <FOLLOW_LINE index="0" on_enter="ENT_FOLLOW_LINE" on_loop="LP_FOLLOW_LINE">
        <EVT_START>FOLLOW_LINE</EVT_START>
        <EVT_GAP_DETECTED>IDENTIFY_INTERSECTION</EVT_GAP_DETECTED>
        <EVT_INTERSECTION_LEFT_DETECTED>INTERSECTION_TURN</EVT_INTERSECTION_LEFT_DETECTED>
        <EVT_INTERSECTION_RIGHT_DETECTED>INTERSECTION_TURN</EVT_INTERSECTION_RIGHT_DETECTED>
        <EVT_STOP>IDLE</EVT_STOP>
      </FOLLOW_LINE>
      <IDENTIFY_INTERSECTION index="1" on_enter="ENT_IDENTIFY_INTERSECTION" on_loop="LP_IDENTIFY_INTERSECTION">
        <EVT_START>FOLLOW_LINE</EVT_START>
        <EVT_INTERSECTION_LEFT_DETECTED>INTERSECTION_TURN</EVT_INTERSECTION_LEFT_DETECTED>
        <EVT_INTERSECTION_RIGHT_DETECTED>INTERSECTION_TURN</EVT_INTERSECTION_RIGHT_DETECTED>
        <EVT_STOP>IDLE</EVT_STOP>
      </IDENTIFY_INTERSECTION>
      <INTERSECTION_TURN index="2" on_enter="ENT_INTERSECTION_TURN" on_loop="LP_INTERSECTION_TURN">
        <EVT_START>FOLLOW_LINE</EVT_START>
        <EVT_STOP>IDLE</EVT_STOP>
      </INTERSECTION_TURN>
      <INTERSECTION_ALIGN index="3" on_enter="ENT_INTERSECTION_ALIGN" on_loop="LP_INTERSECTION_ALIGN">
        <EVT_START>FOLLOW_LINE</EVT_START>
        <EVT_STOP>IDLE</EVT_STOP>
      </INTERSECTION_ALIGN>
      <IDLE index="4">
        <EVT_START>FOLLOW_LINE</EVT_START>
      </IDLE>
    </states>
    <events>
      <EVT_START index="0" access="PUBLIC"/>
      <EVT_GAP_DETECTED index="1" access="PRIVATE"/>
      <EVT_INTERSECTION_LEFT_DETECTED index="2" access="PRIVATE"/>
      <EVT_INTERSECTION_RIGHT_DETECTED index="3" access="PRIVATE"/>
      <EVT_STOP index="4" access="PUBLIC"/>
    </events>
    <connectors>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/
