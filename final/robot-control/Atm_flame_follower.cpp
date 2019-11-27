#include "Atm_flame_follower.h"

/* Add optional parameters for the state machine to begin()
   Add extra initialization code
*/

Atm_flame_follower& Atm_flame_follower::begin(int leftPin, int rightPin, int forwardPin) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                                                   ON_ENTER                   ON_LOOP  ON_EXIT  EVT_START  EVT_SIDE_FLAME_DETECTED   EVT_HALTING_COMPLETE  EVT_FORWARD_FLAME_DETECTED      EVT_BOOTH_AVOIDED  EVT_BOOTH_TOO_CLOSE_RIGHT  EVT_BOOTH_TOO_CLOSE_LEFT  ELSE */
    /*                      IDLE */                            -1,                  LP_IDLE,      -1,      IDLE,       HALTING_FOR_FLAME,                    -1,                         -1,                    -1,                        -1,                       -1,   -1,
    /*         HALTING_FOR_FLAME */         ENT_HALTING_FOR_FLAME,                       -1,      -1,      IDLE,                      -1, TURNING_TOWARDS_FLAME,                         -1,                    -1,                        -1,                       -1,   -1,
    /*     TURNING_TOWARDS_FLAME */                            -1, LP_TURNING_TOWARDS_FLAME,      -1,      IDLE,                      -1,                    -1,  STOPPED_IN_FRONT_OF_FLAME,                    -1,            AVOIDING_BOOTH,           AVOIDING_BOOTH,   -1,
    /* STOPPED_IN_FRONT_OF_FLAME */ ENT_STOPPED_IN_FRONT_OF_FLAME,                       -1,      -1,      IDLE,                      -1,                    -1,                         -1,                    -1,                        -1,                       -1,   -1,
    /*            AVOIDING_BOOTH */                            -1,        LP_AVOIDING_BOOTH,      -1,      IDLE,                      -1,                    -1,                         -1, TURNING_TOWARDS_FLAME,                        -1,                       -1,   -1,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  Serial.println("Initializing Flame Follower");
  this->leftPin = leftPin;
  this->rightPin = rightPin;
  this->forwardPin = forwardPin;
  pinMode(leftPin, INPUT);
  pinMode(rightPin, INPUT);
  pinMode(forwardPin, INPUT);
  halt_timer.set( ATM_TIMER_OFF );
  return *this;
}

/* Add C++ code for each internally handled event (input)
   The code must return 1 to trigger the event
*/

int Atm_flame_follower::event( int id ) {
  switch ( id ) {
    case EVT_SIDE_FLAME_DETECTED:

      if (state() == IDLE) {
        if (leftFlameDet) {
          flameSide = D_LEFT;
          return 1;
        } else if (rightFlameDet) {
          flameSide = D_RIGHT;
          return 1;
        } else {
          flameSide = D_NONE;
          return 0;
        }
      } 
      
      return 0;
    case EVT_HALTING_COMPLETE:
      return halt_timer.expired( this );
    case EVT_FORWARD_FLAME_DETECTED:

      if (state() == TURNING_TOWARDS_FLAME && forwardFlameDet) {
        return 1;
      } else {
        return 0;
      }
      
    case EVT_BOOTH_AVOIDED:
      // TODO when booth sensors are working
      return 0;
    case EVT_BOOTH_TOO_CLOSE_RIGHT:
      // TODO when booth sensors are working
      return 0;
    case EVT_BOOTH_TOO_CLOSE_LEFT:
      // TODO when booth sensors are working
      return 0;
  }
  return 0;
}

void Atm_flame_follower::pollFlameSensors() {
  if (digitalRead(leftPin) == LOW) {
    Serial.println("Left fire detected");
    leftFlameDet = true;
  } else {
    leftFlameDet = false;
  }

  if (digitalRead(rightPin) == LOW) {
    Serial.println("Right fire detected");
    rightFlameDet = true;
  } else {
    rightFlameDet = false;
  }

  if (digitalRead(forwardPin) == LOW) {
    Serial.println("Right fire detected");
    forwardFlameDet = true;
  } else {
    forwardFlameDet = false;
  }
}

/* Add C++ code for each action
   This generates the 'output' for the state machine

   Available connectors:
     push( connectors, ON_FLAMEDETECTED, 0, <v>, <up> );
     push( connectors, ON_FLAMEHANDLED, 0, <v>, <up> );
     push( connectors, ON_MOTOR_CHANGE, 0, <v>, <up> );
*/

void Atm_flame_follower::action( int id ) {
  switch ( id ) {
    case LP_IDLE:
      pollFlameSensors();
      return;
    case ENT_HALTING_FOR_FLAME:
      halt_timer.set(500);
      push( connectors, ON_FLAMEDETECTED, 0, 0, 0 );
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
      return;
    case LP_TURNING_TOWARDS_FLAME:
      pollFlameSensors();
      if (flameSide == D_LEFT) {
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_LEFT, 0 );
      } else if (flameSide == D_RIGHT) {
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_RIGHT, 0 );
      } else {
        Serial.println("ERROR: TURNING_TOWARDS_FLAME state doesn't know what side the flame is on");
      }
    
      return;
    case ENT_STOPPED_IN_FRONT_OF_FLAME:
//      push( connectors, ON_FLAMEHANDLED, 0, 0, 0 ); TODO when booth resolution protocol resolved
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
      return;
    case LP_AVOIDING_BOOTH:
      // TODO implement when booth sensors work
      return;
  }
}

/* Optionally override the default trigger() method
   Control how your machine processes triggers
*/

Atm_flame_follower& Atm_flame_follower::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
   Control what the machine returns when another process requests its state
*/

int Atm_flame_follower::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line
 ************************************************************************************************
*/

/* Public event methods

*/

Atm_flame_follower& Atm_flame_follower::side_flame_detected() {
  trigger( EVT_SIDE_FLAME_DETECTED );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::halting_complete() {
  trigger( EVT_HALTING_COMPLETE );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::forward_flame_detected() {
  trigger( EVT_FORWARD_FLAME_DETECTED );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::booth_avoided() {
  trigger( EVT_BOOTH_AVOIDED );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::booth_too_close_right() {
  trigger( EVT_BOOTH_TOO_CLOSE_RIGHT );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::booth_too_close_left() {
  trigger( EVT_BOOTH_TOO_CLOSE_LEFT );
  return *this;
}

/*
   onFlamedetected() push connector variants ( slots 1, autostore 0, broadcast 0 )
*/

Atm_flame_follower& Atm_flame_follower::onFlamedetected( Machine& machine, int event ) {
  onPush( connectors, ON_FLAMEDETECTED, 0, 1, 1, machine, event );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::onFlamedetected( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_FLAMEDETECTED, 0, 1, 1, callback, idx );
  return *this;
}

/*
   onFlamehandled() push connector variants ( slots 1, autostore 0, broadcast 0 )
*/

Atm_flame_follower& Atm_flame_follower::onFlamehandled( Machine& machine, int event ) {
  onPush( connectors, ON_FLAMEHANDLED, 0, 1, 1, machine, event );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::onFlamehandled( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_FLAMEHANDLED, 0, 1, 1, callback, idx );
  return *this;
}

/*
   onMotorChange() push connector variants ( slots 1, autostore 0, broadcast 0 )
*/

Atm_flame_follower& Atm_flame_follower::onMotorChange( Machine& machine, int event ) {
  onPush( connectors, ON_MOTOR_CHANGE, 0, 1, 1, machine, event );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::onMotorChange( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_MOTOR_CHANGE, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
   Sets the symbol table and the default logging method for serial monitoring
*/

Atm_flame_follower& Atm_flame_follower::trace( Stream & stream ) {
   Machine::setTrace( &stream, atm_serial_debug::trace,
    "FLAME_FOLLOWER\0EVT_START\0EVT_SIDE_FLAME_DETECTED\0EVT_HALTING_COMPLETE\0EVT_FORWARD_FLAME_DETECTED\0EVT_BOOTH_AVOIDED\0EVT_BOOTH_TOO_CLOSE_RIGHT\0EVT_BOOTH_TOO_CLOSE_LEFT\0ELSE\0IDLE\0HALTING_FOR_FLAME\0TURNING_TOWARDS_FLAME\0STOPPED_IN_FRONT_OF_FLAME\0AVOIDING_BOOTH" );
  return *this;
}
