#include "Atm_ball_grabber.h"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

Atm_ball_grabber& Atm_ball_grabber::begin( void ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                               ON_ENTER             ON_LOOP  ON_EXIT        EVT_START  EVT_STOP  EVT_RELEASE    EVT_GRAB  ELSE */
    /*            IDLE */            ENT_IDLE,                 -1,      -1, WAITING_TO_GRAB,       -1,          -1,         -1,   -1,
    /* WAITING_TO_GRAB */ ENT_WAITING_TO_GRAB, LP_WAITING_TO_GRAB,      -1,              -1,     IDLE,   RELEASING, HOLDING_ON,   -1,
    /*      HOLDING_ON */      ENT_HOLDING_ON,      LP_HOLDING_ON,      -1,              -1,     IDLE,   RELEASING,         -1,   -1,
    /*       RELEASING */       ENT_RELEASING,       LP_RELEASING,      -1, WAITING_TO_GRAB,     IDLE,   RELEASING, HOLDING_ON,   -1,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  Serial.println("Initializing Ball Grabber TM");
//  pinMode(sensorPinLeft, INPUT);
//  pinMode(sensorPinRight, INPUT);
  claw.attach(9);

//  while(true) {
//    Serial.println(sonic.read());
//    claw.write(80);
//  }
  
  
  return *this;          
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_ball_grabber::event( int id ) {
  switch ( id ) {
    case EVT_START:
      return 0;
    case EVT_GRAB:
      return ballDetected;
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_BALLGRABBED, 0, <v>, <up> );
 *   push( connectors, ON_BALLLOST, 0, <v>, <up> );
 */

void Atm_ball_grabber::action( int id ) {
  switch ( id ) {
    case ENT_IDLE:
      ballDetected = false;
      ballGrabbed = false;
      Serial.println("Claw relaxed Idle");
      claw.write(0);
      return;
    case ENT_WAITING_TO_GRAB:
      ballDetected = false;
      ballGrabbed = false;
      Serial.println("Claw relaxed Waiting");
      claw.write(0);
      return;
    case LP_WAITING_TO_GRAB:
//      ballDetected = digitalRead(sensorPinLeft) == LOW || digitalRead(sensorPinRight) == LOW;
//      ballDetected = digitalRead(sensorPinLeft) == LOW;
//      ballDetected = digitalRead(sensorPinRight) == LOW;

      ballDetected = sonic.read() > 5;

      if (ballDetected) {
        Serial.println("BALL DETECTED");
      }
      return;
    case ENT_HOLDING_ON:
      Serial.println("Holding the ball");
      
      if (!ballGrabbed) {
        Serial.println("Ball grabbed");
        ballGrabbed = true;
        claw.write(80);
        push( connectors, ON_BALLGRABBED, 0, 0, 0 );
      } else {
        claw.write(80);
      }
      
      return;
    case LP_HOLDING_ON:

//      ballDetected = digitalRead(sensorPinLeft) == LOW || digitalRead(sensorPinRight) == LOW;

      if (!ballDetected) {
        push( connectors, ON_BALLLOST, 0, 0, 0 );
      }
      else {
        Serial.println("Closing claw");
        claw.write(80);
      }
    
      return;
    case ENT_RELEASING:
      claw.write(0);
      return;
    case LP_RELEASING:
      return;
  }
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_ball_grabber& Atm_ball_grabber::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_ball_grabber::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

Atm_ball_grabber& Atm_ball_grabber::start() {
  trigger( EVT_START );
  return *this;
}

Atm_ball_grabber& Atm_ball_grabber::stop() {
  trigger( EVT_STOP );
  return *this;
}

Atm_ball_grabber& Atm_ball_grabber::release() {
  trigger( EVT_RELEASE );
  return *this;
}

Atm_ball_grabber& Atm_ball_grabber::grab() {
  trigger( EVT_GRAB );
  return *this;
}

/*
 * onBallgrabbed() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

Atm_ball_grabber& Atm_ball_grabber::onBallgrabbed( Machine& machine, int event ) {
  onPush( connectors, ON_BALLGRABBED, 0, 1, 1, machine, event );
  return *this;
}

Atm_ball_grabber& Atm_ball_grabber::onBallgrabbed( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_BALLGRABBED, 0, 1, 1, callback, idx );
  return *this;
}

/*
 * onBalllost() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

Atm_ball_grabber& Atm_ball_grabber::onBalllost( Machine& machine, int event ) {
  onPush( connectors, ON_BALLLOST, 0, 1, 1, machine, event );
  return *this;
}

Atm_ball_grabber& Atm_ball_grabber::onBalllost( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_BALLLOST, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_ball_grabber& Atm_ball_grabber::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "BALL_GRABBER\0EVT_START\0EVT_STOP\0EVT_RELEASE\0EVT_GRAB\0ELSE\0IDLE\0WAITING_TO_GRAB\0HOLDING_ON\0RELEASING" );
  return *this;
}
