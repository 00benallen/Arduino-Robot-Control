#include <Automaton.h>
#include "Atm_ball_grabber.h"

// Basic Arduino sketch - instantiates the state machine and nothing else

Atm_ball_grabber ball_grabber;

void setup() {

  Serial.begin( 9600 );
  // ball_grabber.trace( Serial );

  ball_grabber.begin(32).trace( Serial ).start();

}

void loop() {
  automaton.run();
}
