#pragma once

#include <Automaton.h>
#include <Smoothed.h>

class Atm_flame_sensor: public Machine {

 public:
  enum { CALIBRATING, NO_FLAME, FLAME, DISABLED }; // STATES
  enum { EVT_ON, EVT_OFF, EVT_FLAME_DETECTED, EVT_NO_FLAME_DETECTED, EVT_DONE_CALIBRATING, ELSE }; // EVENTS
  Atm_flame_sensor( void ) : Machine() {};
  Atm_flame_sensor& begin( int analogPin );
  Atm_flame_sensor& trace( Stream & stream );
  Atm_flame_sensor& trigger( int event );
  int state( void );
  Atm_flame_sensor& onChange( Machine& machine, int event = 0 );
  Atm_flame_sensor& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_flame_sensor& on( void );
  Atm_flame_sensor& off( void );

 private:
  enum { ENT_CALIBRATING, LP_NO_FLAME, ENT_FLAME, LP_FLAME, ENT_DISABLED }; // ACTIONS
  enum { ON_CHANGE, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id ); 
  int analogPin;
  bool fire;
  bool calibratingDone;
  void calibrateSensor();
  float pollSensor();
  float ambientMin; float ambientMax;
  Smoothed <float> smoothed;
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_flame_sensor">
    <states>
      <CALIBRATING index="0">
        <EVT_OFF>DISABLED</EVT_OFF>
      </CALIBRATING>
      <NO_FLAME index="1">
        <EVT_OFF>DISABLED</EVT_OFF>
        <EVT_FLAME_DETECTED>FLAME</EVT_FLAME_DETECTED>
      </NO_FLAME>
      <FLAME index="2">
        <EVT_OFF>DISABLED</EVT_OFF>
        <EVT_NO_FLAME_DETECTED>NO_FLAME</EVT_NO_FLAME_DETECTED>
      </FLAME>
      <DISABLED index="3">
        <EVT_ON>CALIBRATING</EVT_ON>
      </DISABLED>
    </states>
    <events>
      <EVT_ON index="0" access="MIXED"/>
      <EVT_OFF index="1" access="MIXED"/>
      <EVT_FLAME_DETECTED index="2" access="PRIVATE"/>
      <EVT_NO_FLAME_DETECTED index="3" access="PRIVATE"/>
    </events>
    <connectors>
      <CHANGE autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/
