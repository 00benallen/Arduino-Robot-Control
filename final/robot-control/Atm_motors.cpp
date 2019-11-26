#include "Atm_motors.h"

/* Add optional parameters for the state machine to begin()
   Add extra initialization code
*/

Atm_motors& Atm_motors::begin(int motor_speed) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                           ON_ENTER  ON_LOOP  ON_EXIT    EVT_FORWARD    EVT_BACKWARD    EVT_STOP      EVT_LEFT      EVT_RIGHT  ELSE */
    /*       FORWARD */       ENT_FORWARD,      -1,      -1,            -1,     S_BACKWARD,     S_IDLE, TURNING_LEFT, TURNING_RIGHT,   -1,
    /*  TURNING_LEFT */  ENT_TURNING_LEFT,      -1,      -1,     S_FORWARD,     S_BACKWARD,     S_IDLE,           -1, TURNING_RIGHT,   -1,
    /* TURNING_RIGHT */ ENT_TURNING_RIGHT,      -1,      -1,     S_FORWARD,     S_BACKWARD,     S_IDLE, TURNING_LEFT,            -1,   -1,
    /*      BACKWARD */      ENT_BACKWARD,      -1,      -1,     S_FORWARD,             -1,     S_IDLE, TURNING_LEFT, TURNING_RIGHT,   -1,
    /*          IDLE */          ENT_IDLE,      -1,      -1,     S_FORWARD,     S_BACKWARD,         -1, TURNING_LEFT, TURNING_RIGHT,   -1,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  Serial.println("Initializing motors");
  motorFrontRight.setSpeed(motor_speed);
  motorFrontRight.run(RELEASE);
  motorBackRight.setSpeed(motor_speed);
  motorBackRight.run(RELEASE);
  motorFrontLeft.setSpeed(motor_speed);
  motorFrontLeft.run(RELEASE);
  motorBackLeft.setSpeed(motor_speed);
  motorBackLeft.run(RELEASE);
  return *this;
}

/* Add C++ code for each internally handled event (input)
   The code must return 1 to trigger the event
*/
// for now, no internal handling needed, later maybe some
int Atm_motors::event( int id ) {
  switch ( id ) {
    case EVT_FORWARD:
      return 0;
    case EVT_BACKWARD:
      return 0;
    case EVT_STOP:
      return 0;
    case EVT_LEFT:
      return 0;
    case EVT_RIGHT:
      return 0;
  }
  return 0;
}

/* Add C++ code for each action
   This generates the 'output' for the state machine
*/

void Atm_motors::action( int id ) {
  if (enabled) {
    switch ( id ) {
      case ENT_FORWARD:
        motorFrontRight.run(FORWARD);
        motorBackRight.run(FORWARD);
        motorFrontLeft.run(FORWARD);
        motorBackLeft.run(FORWARD);
        return;
      case ENT_TURNING_LEFT:
        motorFrontRight.run(FORWARD);
        motorBackRight.run(FORWARD);
        motorFrontLeft.run(BACKWARD);
        motorBackLeft.run(BACKWARD);
        return;
      case ENT_TURNING_RIGHT:
        motorFrontRight.run(BACKWARD);
        motorBackRight.run(BACKWARD);
        motorFrontLeft.run(FORWARD);
        motorBackLeft.run(FORWARD);
        return;
      case ENT_BACKWARD:
        motorFrontRight.run(BACKWARD);
        motorBackRight.run(BACKWARD);
        motorFrontLeft.run(BACKWARD);
        motorBackLeft.run(BACKWARD);
        return;
      case ENT_IDLE:
        motorFrontRight.run(RELEASE);
        motorBackRight.run(RELEASE);
        motorFrontLeft.run(RELEASE);
        motorBackLeft.run(RELEASE);
        return;
    }
  } else {
    motorFrontRight.run(RELEASE);
    motorBackRight.run(RELEASE);
    motorFrontLeft.run(RELEASE);
    motorBackLeft.run(RELEASE);
    return;
  }

}

Atm_motors& Atm_motors::disable( void ) {
  enabled = false;
}

Atm_motors& Atm_motors::enable( void ) {
  enabled = true;
}

/* Optionally override the default trigger() method
   Control how your machine processes triggers
*/

Atm_motors& Atm_motors::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
   Control what the machine returns when another process requests its state
*/

int Atm_motors::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line
 ************************************************************************************************
*/

/* Public event methods

*/

Atm_motors& Atm_motors::forward() {
  trigger( EVT_FORWARD );
  return *this;
}

Atm_motors& Atm_motors::backward() {
  trigger( EVT_BACKWARD );
  return *this;
}

Atm_motors& Atm_motors::stop() {
  trigger( EVT_STOP );
  return *this;
}

Atm_motors& Atm_motors::left() {
  trigger( EVT_LEFT );
  return *this;
}

Atm_motors& Atm_motors::right() {
  trigger( EVT_RIGHT );
  return *this;
}

/* State trace method
   Sets the symbol table and the default logging method for serial monitoring
*/

Atm_motors& Atm_motors::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
                     "MOTORS\0EVT_FORWARD\0EVT_BACKWARD\0EVT_STOP\0EVT_LEFT\0EVT_RIGHT\0ELSE\0FORWARD\0TURNING_LEFT\0TURNING_RIGHT\0BACKWARD\0IDLE" );
  return *this;
}
