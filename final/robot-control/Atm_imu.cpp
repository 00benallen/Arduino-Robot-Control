#include "Atm_imu.h"

/* Add optional parameters for the state machine to begin()
   Add extra initialization code
*/

Atm_imu& Atm_imu::begin() {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                           ON_ENTER                ON_LOOP            ON_EXIT  EVT_TRACK_TURN  EVT_TURN_COMPLETE  ELSE */
    /*        S_IDLE */          ENT_IDLE,      LP_IDLE,                         -1,  TRACKING_TURN,                -1,   -1,
    /* TRACKING_TURN */ ENT_TRACKING_TURN,      LP_TRACKING_TURN, EXT_TRACKING_TURN,             -1,            S_IDLE,   -1,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  Serial.println("Initializing BNO055 9-axis IMU");
  while (!bno.begin())
  {
    Serial.println("No BNO055 detected");
    delay(1000);
  }
  Serial.println("BNO055 9-axis IMU detected and initialized");
  return *this;
}

/* Add C++ code for each internally handled event (input)
   The code must return 1 to trigger the event
*/
// for now, no internal handling needed, later maybe some
int Atm_imu::event( int id ) {
  switch ( id ) {
    case EVT_TRACK_TURN:
      return 0;
    case EVT_TURN_COMPLETE:
      float diff = angleAbsDiff();
      Serial.print("Start heading: "); Serial.println(turnStartHeading);
      Serial.print("Angle diff: "); Serial.println(diff);
      if (angleAbsDiff() >= turnHeadingDifference) {
        push(connectors, ON_TURNEND, 0, 0, 0);
        return 1;
      } else {
        return 0;
      }
  }
  return 0;
}

float Atm_imu::angleAbsDiff() {
  return 180 - abs(abs(currentTurnHeading - turnStartHeading) - 180);
}

/* Add C++ code for each action
   This generates the 'output' for the state machine

   Available connectors:
     push( connectors, ON_TURNEND, 0, <v>, <up> );
*/

void Atm_imu::action( int id ) {
  switch ( id ) {
    case ENT_IDLE:
      return;
    case LP_IDLE:
      pollIMU();
      return;
    case ENT_TRACKING_TURN:
      turnStartHeading = currentTurnHeading;
      turnComplete = false;
      return;
    case LP_TRACKING_TURN:
      pollIMU();
      return;
    case EXT_TRACKING_TURN:
      Serial.println("Notifying connectors of turn end");
      push( connectors, ON_TURNEND, 0, 0, 0 );
      turnComplete = true;
      return;
  }
}


void Atm_imu::pollIMU() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  currentTurnHeading = orientationData.orientation.x;
  Serial.print("Current heading: "); Serial.println(currentTurnHeading);
}

/* Optionally override the default trigger() method
   Control how your machine processes triggers
*/

Atm_imu& Atm_imu::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
   Control what the machine returns when another process requests its state
*/

int Atm_imu::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line
 ************************************************************************************************
*/

/* Public event methods

*/

Atm_imu& Atm_imu::track_turn(float turnHeadingDifference) {
  this->turnHeadingDifference = turnHeadingDifference;
  trigger( EVT_TRACK_TURN );
  return *this;
}

/*
   onTurnend() push connector variants ( slots 1, autostore 0, broadcast 0 )
*/

Atm_imu& Atm_imu::onTurnend( Machine& machine, int event ) {
  onPush( connectors, ON_TURNEND, 0, 1, 1, machine, event );
  return *this;
}

Atm_imu& Atm_imu::onTurnend( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_TURNEND, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
   Sets the symbol table and the default logging method for serial monitoring
*/

Atm_imu& Atm_imu::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
                     "IMU\0EVT_TRACK_TURN\0EVT_TURN_COMPLETE\0ELSE\0S_IDLE\0TRACKING_TURN" );
  return *this;
}
