#pragma once

#include <Automaton.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

class Atm_imu: public Machine {

  public:
    enum { S_IDLE, TRACKING_TURN }; // STATES
    enum { EVT_TRACK_TURN, EVT_TURN_COMPLETE, ELSE }; // EVENTS
    Atm_imu( void ) : Machine(), bno(55, 0x28) {};
    Atm_imu& begin( void );
    Atm_imu& trace( Stream & stream );
    Atm_imu& trigger( int event );
    int state( void );
    Atm_imu& onTurnend( Machine& machine, int event = 0 );
    Atm_imu& onTurnend( atm_cb_push_t callback, int idx = 0 );
    Atm_imu& track_turn( float turnHeadingDifference);
    bool turnComplete; // TODO remove, used for non-automata pieces of code
    float angleAbsDiff(); // TODO make private

  private:
    enum { ENT_IDLE, LP_IDLE, ENT_TRACKING_TURN, LP_TRACKING_TURN, EXT_TRACKING_TURN }; // ACTIONS
    enum { ON_TURNEND, CONN_MAX }; // CONNECTORS
    atm_connector connectors[CONN_MAX];
    int event( int id );
    void action( int id );
    float turnStartHeading;
    float turnHeadingDifference;
    float currentTurnHeading;
    Adafruit_BNO055 bno;
    void pollIMU();
    


};

/*
  Automaton::ATML::begin - Automaton Markup Language

  <?xml version="1.0" encoding="UTF-8"?>
  <machines>
  <machine name="Atm_imu">
    <states>
      <IDLE index="0" on_enter="ENT_IDLE">
        <EVT_TRACK_TURN>TRACKING_TURN</EVT_TRACK_TURN>
      </IDLE>
      <TRACKING_TURN index="1" on_enter="ENT_TRACKING_TURN" on_exit="EXT_TRACKING_TURN">
      </TRACKING_TURN>
    </states>
    <events>
      <EVT_TRACK_TURN index="0" access="MIXED"/>
    </events>
    <connectors>
      <TURNSTART autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
  </machines>

  Automaton::ATML::end
*/
