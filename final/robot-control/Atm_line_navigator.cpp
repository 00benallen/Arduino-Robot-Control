#include "Atm_line_navigator.h"

/* Add optional parameters for the state machine to begin()
   Add extra initialization code
*/

Atm_line_navigator& Atm_line_navigator::begin(const uint8_t qtrPins[], int leftAuxPin, int rightAuxPin) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                                           ON_ENTER                   ON_LOOP  ON_EXIT    EVT_START       EVT_GAP_DETECTED  EVT_INTERSECTION_LEFT_DETECTED  EVT_INTERSECTION_RIGHT_DETECTED  EVT_STOP  ELSE */
    /*           FOLLOW_LINE */           ENT_FOLLOW_LINE,           LP_FOLLOW_LINE,      -1, FOLLOW_LINE, IDENTIFY_INTERSECTION,              INTERSECTION_TURN,               INTERSECTION_TURN,     IDLE,   -1,
    /* IDENTIFY_INTERSECTION */ ENT_IDENTIFY_INTERSECTION, LP_IDENTIFY_INTERSECTION,      -1, FOLLOW_LINE,                    -1,              INTERSECTION_TURN,               INTERSECTION_TURN,     IDLE,   -1,
    /*     INTERSECTION_TURN */     ENT_INTERSECTION_TURN,     LP_INTERSECTION_TURN,      -1, FOLLOW_LINE,                    -1,                             -1,                              -1,     IDLE,   -1,
    /*    INTERSECTION_ALIGN */    ENT_INTERSECTION_ALIGN,    LP_INTERSECTION_ALIGN,      -1, FOLLOW_LINE,                    -1,                             -1,                              -1,     IDLE,   -1,
    /*                  IDLE */                        -1,                       -1,      -1, FOLLOW_LINE,                    -1,                             -1,                              -1,       -1,   -1,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );

  Serial.println("Initializing Auxillary IR Sensors");
  pinMode(leftAuxPin, INPUT);
  pinMode(rightAuxPin, INPUT);

  Serial.println("Initializing QTC Reflectance Array");
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, 8);

  return *this;
}

Atm_line_navigator& Atm_line_navigator::calibrate() {
  int sensorCount = 8;
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  for (int i = 0; i < 100; i++)
  {
    if (i <= 25) {
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_LEFT, 0 );
    } else if (i <= 75) {
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_RIGHT, 0 );
    } else {
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_LEFT, 0 );
    }

    qtr.calibrate();
  }

  // print the calibration minimum values measured when emitters were on
  Serial.println("Minimum reflectance values");
  for (int i = 0; i < sensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  Serial.println("Maximum reflectance values");
  for (int i = 0; i < sensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}

/* Add C++ code for each internally handled event (input)
   The code must return 1 to trigger the event
*/

int Atm_line_navigator::event( int id ) {
  switch ( id ) {
    case EVT_GAP_DETECTED:
      return 0;
    case EVT_INTERSECTION_LEFT_DETECTED:
      return 0;
    case EVT_INTERSECTION_RIGHT_DETECTED:
      return 0;
  }
  return 0;
}

/* Add C++ code for each action
   This generates the 'output' for the state machine
*/

void Atm_line_navigator::action( int id ) {
  switch ( id ) {
    case ENT_FOLLOW_LINE:
      return;
    case LP_FOLLOW_LINE:
      pollLineSensors();
      //
      if (!lineUnderQtr) {
        Serial.println("Motor go forward plz");
        delay(500);
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_FORWARD, 0); // by default go forward during this state, if we're here without a line something is wrong
      } else {
        if (linePosition < 1000) {
          push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_RIGHT, 0);
        } else if (linePosition > 6000) {
          push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_LEFT, 0);
        } else {
          push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_FORWARD, 0);
        }
      }

      return;
    case ENT_IDENTIFY_INTERSECTION:
      return;
    case LP_IDENTIFY_INTERSECTION:
      return;
    case ENT_INTERSECTION_TURN:
      return;
    case LP_INTERSECTION_TURN:
      return;
    case ENT_INTERSECTION_ALIGN:
      return;
    case LP_INTERSECTION_ALIGN:
      return;
  }
}

void Atm_line_navigator::pollLineSensors() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  int sensorCount = 8;
  unsigned int sensorValues[sensorCount];
  linePosition = qtr.readLineBlack(sensorValues);
  Serial.print("Position: "); Serial.print(linePosition); Serial.print(" Raw values: ");

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  lineUnderQtr = false;
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    unsigned int curReading = sensorValues[i];
    Serial.print(curReading); Serial.print(" ");
    if (curReading > 700) {
      lineUnderQtr = true;

      if (i >= 2 && i <= 5) {
        lineForwardDetected = true;
      }
    }
  }
  Serial.println();
}

/* Optionally override the default trigger() method
   Control how your machine processes triggers
*/

Atm_line_navigator& Atm_line_navigator::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
   Control what the machine returns when another process requests its state
*/

int Atm_line_navigator::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line
 ************************************************************************************************
*/

/* Public event methods

*/

Atm_line_navigator& Atm_line_navigator::start() {
  trigger( EVT_START );
  return *this;
}

Atm_line_navigator& Atm_line_navigator::stop() {
  trigger( EVT_STOP );
  return *this;
}

/*
   onMotorChange() push connector variants ( slots 1, autostore 0, broadcast 0 )
*/

Atm_line_navigator& Atm_line_navigator::onMotorChange( Machine& machine, int event ) {
  onPush( connectors, ON_MOTOR_CHANGE, 0, 1, 1, machine, event );
  return *this;
}

Atm_line_navigator& Atm_line_navigator::onMotorChange( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_MOTOR_CHANGE, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
   Sets the symbol table and the default logging method for serial monitoring
*/

Atm_line_navigator& Atm_line_navigator::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
                     "LINE_NAVIGATOR\0EVT_START\0EVT_GAP_DETECTED\0EVT_INTERSECTION_LEFT_DETECTED\0EVT_INTERSECTION_RIGHT_DETECTED\0EVT_STOP\0ELSE\0FOLLOW_LINE\0IDENTIFY_INTERSECTION\0INTERSECTION_TURN\0INTERSECTION_ALIGN\0IDLE" );
  return *this;
}
