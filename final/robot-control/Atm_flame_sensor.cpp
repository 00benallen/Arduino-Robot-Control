#include "Atm_flame_sensor.h"

/* Add optional parameters for the state machine to begin()
   Add extra initialization code
*/

Atm_flame_sensor& Atm_flame_sensor::begin(int analogPin) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                         ON_ENTER       ON_LOOP  ON_EXIT        EVT_ON    EVT_OFF  EVT_FLAME_DETECTED  EVT_NO_FLAME_DETECTED  EVT_DONE_CALIBRATING  ELSE */
    /* CALIBRATING */  ENT_CALIBRATING,           -1,      -1,           -1,  DISABLED,                 -1,                    -1,             NO_FLAME,   -1,
    /*    NO_FLAME */               -1,  LP_NO_FLAME,      -1,           -1,  DISABLED,              FLAME,                    -1,                   -1,   -1,
    /*       FLAME */               -1,     LP_FLAME,      -1,           -1,  DISABLED,                 -1,              NO_FLAME,                   -1,   -1,
    /*    DISABLED */     ENT_DISABLED,           -1,      -1,  CALIBRATING,        -1,                 -1,                    -1,                   -1,   -1,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  this->analogPin = analogPin;
  pinMode(analogPin, INPUT);
  smoothed.begin(SMOOTHED_EXPONENTIAL, 5);
  return *this;
}

/* Add C++ code for each internally handled event (input)
   The code must return 1 to trigger the event
*/

int Atm_flame_sensor::event( int id ) {
  switch ( id ) {
    case EVT_ON:
      return 0;
    case EVT_OFF:
      return 0;
    case EVT_FLAME_DETECTED:
      if (state() == FLAME && !fire) {
        push (connectors, ON_CHANGE, 0, fire, 0);
      }

      return fire;
    case EVT_NO_FLAME_DETECTED:
      if (state() == NO_FLAME && fire) {
        push (connectors, ON_CHANGE, 0, fire, 0);
      }

      return fire;
    case EVT_DONE_CALIBRATING:
      if (state() == CALIBRATING) {
        return calibratingDone;
      }
      return 0;
  }
  return 0;
}

/* Add C++ code for each action
   This generates the 'output' for the state machine

   Available connectors:
     push( connectors, ON_CHANGE, 0, <v>, <up> );
*/

void Atm_flame_sensor::action( int id ) {
  switch ( id ) {
    case ENT_CALIBRATING:
      calibrateSensor();
      return;
    case LP_NO_FLAME:
      pollSensor();
      return;
    case LP_FLAME:
      return;
    case ENT_DISABLED:
      return;
  }
}

void Atm_flame_sensor::calibrateSensor() {
  // TODO see if we actually need to calibrate anything
  calibratingDone = true;
}

void Atm_flame_sensor::pollSensor() {
  float currentSensorValue = analogRead(analogPin);

  smoothed.add(currentSensorValue);
  float smoothedSensorValueExp = smoothed.get();

//  fire = smoothedSensorValueExp < 900;
//  Serial.print(currentSensorValue); Serial.print("\t"); Serial.print(smoothedSensorValueExp); Serial.print("\t"); Serial.println(1000 * fire);

  // Compute running slope, we want the running derivative of the temperature function
  float runningSlope = smoothedSensorValueExp - previousValue;

  float threshold = 10; // TODO refine
  if (runningSlope >= threshold) {
    fire = true;
  } else {
    fire = false;
  }

  previousValue = smoothedSensorValueExp;
}

/* Optionally override the default trigger() method
   Control how your machine processes triggers
*/

Atm_flame_sensor& Atm_flame_sensor::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
   Control what the machine returns when another process requests its state
*/

int Atm_flame_sensor::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line
 ************************************************************************************************
*/

/* Public event methods

*/

Atm_flame_sensor& Atm_flame_sensor::on() {
  trigger( EVT_ON );
  return *this;
}

Atm_flame_sensor& Atm_flame_sensor::off() {
  trigger( EVT_OFF );
  return *this;
}

/*
   onChange() push connector variants ( slots 1, autostore 0, broadcast 0 )
*/

Atm_flame_sensor& Atm_flame_sensor::onChange( Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, 0, 1, 1, machine, event );
  return *this;
}

Atm_flame_sensor& Atm_flame_sensor::onChange( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
   Sets the symbol table and the default logging method for serial monitoring
*/

Atm_flame_sensor& Atm_flame_sensor::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
                     "FLAME_SENSOR\0EVT_ON\0EVT_OFF\0EVT_FLAME_DETECTED\0EVT_NO_FLAME_DETECTED\0EVT_DONE_CALIBRATING\0ELSE\0CALIBRATING\0NO_FLAME\0FLAME\0DISABLED" );
  return *this;
}
