#include "Atm_flame_follower.h"

/* Add optional parameters for the state machine to begin()
   Add extra initialization code
*/

Atm_flame_follower& Atm_flame_follower::begin(int leftPin, int rightPin, int forwardPin) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                                                   ON_ENTER                   ON_LOOP                  ON_EXIT      EVT_START  EVT_STOP  EVT_SIDE_FLAME_DETECTED   EVT_HALTING_COMPLETE  EVT_FORWARD_FLAME_DETECTED      EVT_BOOTH_AVOIDED  EVT_BOOTH_TOO_CLOSE_RIGHT  EVT_BOOTH_TOO_CLOSE_LEFT  EVT_DONE_CALIBRATING  ELSE */
    /*               CALIBRATING */               ENT_CALIBRATING,                      -1,                      -1,         IDLE,  DISABLED,                      -1,                    -1,                         -1,                    -1,                        -1,                       -1,                 IDLE,    -1,
    /*                      IDLE */                            -1,                  LP_IDLE,                     -1,           -1,  DISABLED,       HALTING_FOR_FLAME,                    -1,                         -1,                    -1,                        -1,                       -1,                   -1,    -1,
    /*         HALTING_FOR_FLAME */         ENT_HALTING_FOR_FLAME,                       -1,                     -1,         IDLE,  DISABLED,                      -1, TURNING_TOWARDS_FLAME,                         -1,                    -1,                        -1,                       -1,                   -1,    -1,
    /*     TURNING_TOWARDS_FLAME */                            -1, LP_TURNING_TOWARDS_FLAME,                     -1,         IDLE,  DISABLED,                      -1,                    -1,          APPROACHING_FLAME,                    -1,            AVOIDING_BOOTH,           AVOIDING_BOOTH,                   -1,    -1,
    /*         APPROACHING_FLAME */         ENT_APPROACHING_FLAME,     LP_APPROACHING_FLAME,  EXT_APPROACHING_FLAME,         IDLE,  DISABLED,                      -1,                    -1,                         -1,                    -1,                        -1,                       -1,                   -1,    -1,
    /*            AVOIDING_BOOTH */                            -1,        LP_AVOIDING_BOOTH,                     -1,         IDLE,  DISABLED,                      -1,                    -1,                         -1, TURNING_TOWARDS_FLAME,                        -1,                       -1,                   -1,    -1,
    /*                  DISABLED */                  ENT_DISABLED,                       -1,                     -1,         IDLE,  DISABLED,                      -1,                    -1,                         -1,                    -1,                        -1,                       -1,                   -1,    -1,
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
  smoothedLeft.begin(SMOOTHED_EXPONENTIAL, 50);
  smoothedRight.begin(SMOOTHED_EXPONENTIAL, 50);
  smoothedForward.begin(SMOOTHED_EXPONENTIAL, 300);
  smoothedDistance.begin(SMOOTHED_EXPONENTIAL, 50);
  halt_timer.set( ATM_TIMER_OFF );

  //  while (!lox.begin()) {
  //    Serial.println(F("Failed to boot VL53L0X"));
  //    delay(1000);
  //  }
  //  Serial.println("VL53L0X laser TOF sensor detected");
//
//    while (true) {
//      Serial.println(sonic.read());
//    }
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
    case EVT_DONE_CALIBRATING:
      Serial.println("Checking if calibration done");
      if (state() == CALIBRATING) {
        Serial.println("done");
        return doneCalibrating;
      } else {
        return 0;
      }

  }
  return 0;
}


void Atm_flame_follower::calibrateSensors() {

  push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
  // TODO see if we actually need to calibrate anything

  for (int i = 0; i < 400; i++) {
    float cur = analogRead(leftPin);
    if (cur < ambientMinLeft) {
      ambientMinLeft = cur;
    } else if (cur > ambientMaxLeft) {
      ambientMaxLeft = cur;
    }
    delay(1);
  }

  Serial.println("Left sensor done");

  for (int i = 0; i < 400; i++) {
    float cur = analogRead(rightPin);
    if (cur < ambientMinRight) {
      ambientMinRight = cur;
    } else if (cur > ambientMaxRight) {
      ambientMaxRight = cur;
    }
    delay(1);
  }

  Serial.println("Right sensor done");

  for (int i = 0; i < 400; i++) {
    float cur = analogRead(forwardPin);
    if (cur < ambientMinForward) {
      ambientMinForward = cur;
    } else if (cur > ambientMaxForward) {
      ambientMaxForward = cur;
    }
    delay(1);
  }

  Serial.print("Ambient min/max for sensor on pin "); Serial.println(leftPin);
  Serial.print(ambientMinLeft); Serial.print(" / "); Serial.println(ambientMaxLeft);
  Serial.print("Ambient min/max for sensor on pin "); Serial.println(rightPin);
  Serial.print(ambientMinRight); Serial.print(" / "); Serial.println(ambientMaxRight);
  Serial.print("Ambient min/max for sensor on pin "); Serial.println(forwardPin);
  Serial.print(ambientMinForward); Serial.print(" / "); Serial.println(ambientMaxForward);

  doneCalibrating = true;
}

void Atm_flame_follower::pollFlameSensors() {

  // poll left
  float currentSensorValue = analogRead(leftPin);

  smoothedLeft.add(currentSensorValue);
  float smoothedSensorValue = smoothedLeft.get();
  // Compute running slope, we want the running derivative of the temperature function

  float sensorDiff = ambientMinLeft - smoothedSensorValue;
  Serial.print("Left: "); Serial.print(smoothedSensorValue); Serial.print(" / "); Serial.println(sensorDiff);
  float sideThreshold = 12; // TODO refine
  if (sensorDiff >= sideThreshold) {
    leftFlameDet = true;
  } else if (smoothedSensorValue >= ambientMinLeft) {
    leftFlameDet = false;
  }

  // poll right
  currentSensorValue = analogRead(rightPin);

  smoothedRight.add(currentSensorValue);
  smoothedSensorValue = smoothedRight.get();
  // Compute running slope, we want the running derivative of the temperature function

  sensorDiff = ambientMinRight - smoothedSensorValue;
  Serial.print("Right: "); Serial.print(smoothedSensorValue); Serial.print(" / "); Serial.println(sensorDiff);
  if (sensorDiff >= sideThreshold) {
    rightFlameDet = true;
  } else if (smoothedSensorValue >= ambientMinRight) {
    rightFlameDet = false;
  }

  // poll forward
  currentSensorValue = analogRead(forwardPin);

  smoothedForward.add(currentSensorValue);
  smoothedSensorValue = smoothedForward.get();
  // Compute running slope, we want the running derivative of the temperature function

  sensorDiff = ambientMinForward - smoothedSensorValue;
  int forwardThreshold = 8;
  Serial.print("Forward: "); Serial.print(smoothedSensorValue); Serial.print(" / "); Serial.println(sensorDiff);
  if (sensorDiff >= forwardThreshold) {
    forwardFlameDet = true;
  } else if (smoothedSensorValue >= ambientMinForward) {
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
    case ENT_CALIBRATING:
      calibrateSensors();
    case LP_IDLE:
      pollFlameSensors();

      if (forwardFlameDet) {
        push ( connectors, ON_FLAMEDETECTED, 0, D_FORWARD, 0 );
        forwardFlameDet = false;
      }
      return;
    case ENT_HALTING_FOR_FLAME:
      Serial.println("Halting");
      halt_timer.set(500);
      push( connectors, ON_FLAMEDETECTED, 0, flameSide, 0 );
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
      return;
    case LP_TURNING_TOWARDS_FLAME:
      pollFlameSensors();
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0);
      delay(50);
      if (flameSide == D_LEFT) {
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_LEFT, 0 );
//        delay(500);
      } else if (flameSide == D_RIGHT) {
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_RIGHT, 0 );
//        delay(500);
      } else {
        Serial.println("ERROR: TURNING_TOWARDS_FLAME state doesn't know what side the flame is on");
      }

      return;
    case ENT_APPROACHING_FLAME:
      {
        push( connectors, ON_APPROACHING_BALL, 0, 0, 0 );
        closeEnoughToBall = false;
        loopsBeforeUltrasonic = 0;
        pendingStop = false;
        return;
      }
      break;
    case LP_APPROACHING_FLAME:
      if (pendingStop) { // to reduce delay on stopping, may cause crashes?
        return;
      }
      
      loopsBeforeUltrasonic++;

      Serial.println("LOOOOOOP");
      unsigned int forwardDelay = 200;
      unsigned int stopDelay = 400;

      unsigned int candleDist = sonic.read();
      if (candleDist <= 8) {
        push ( connectors, ON_NO_BALL, 0, 0, 0);
      }

      if (closeEnoughToBall) {
        Serial.println("Close enough to ball, rushing in");
        //        VL53L0X_RangingMeasurementData_t measure;
        //        lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
        //        if (measure.RangeStatus == 2) {
        //          Serial.print(measure.RangeMilliMeter); Serial.print("\t"); Serial.println(smoothedDistance.get());
        //        }
        //
        //        if (measure.RangeMilliMeter < 150) {
        //          push ( connectors, ON_NO_BALL, 0, 0, 0);
        //        }


        int fastForwardDelay = 100 ;
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_FORWARD, 0 );
        delay(fastForwardDelay);
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
        delay(fastForwardDelay);
        return;
      }

      Serial.println("Not close enough to ball, searching");

      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
      delay(stopDelay);

      // turn right
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_RIGHT, 0 );
      delay(turnDelay);
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
      delay(stopDelay);

      // read avg value
      unsigned int rightSum = 0;
      for (int i = 0; i < 10; i++) {
        rightSum += analogRead(forwardPin);
        delayMicroseconds(1);
      }
      unsigned int rightRead = rightSum / 10;

      // turn to middle
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_LEFT, 0 );
      delay(turnDelay);
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
      delay(stopDelay);

      // read avg value
      unsigned int middleSum = 0;
      for (int i = 0; i < 10; i++) {
        middleSum += analogRead(forwardPin);
        delayMicroseconds(1);
      }
      unsigned int middleRead = middleSum / 10;

      // turn to left
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_LEFT, 0 );
      delay(turnDelay);
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
      delay(stopDelay);

      // read avg value
      unsigned int leftSum = 0;
      for (int i = 0; i < 10; i++) {
        leftSum += analogRead(forwardPin);
        delayMicroseconds(1);
      }
      unsigned int leftRead = leftSum / 10;

      Serial.print("Right"); Serial.println(rightRead);
      Serial.print("Left"); Serial.println(leftRead);
      Serial.print("Middle"); Serial.println(middleRead);

      // choose hottest of 3, turn to that direction and go forward a bit
      if (rightRead <= middleRead && rightRead <= leftRead) {
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_RIGHT, 0 );
        delay(turnDelay);
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
        delay(stopDelay);
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_RIGHT, 0 );
        delay(turnDelay);
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
        delay(stopDelay);
      } else if (middleRead < leftRead) {
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_RIGHT, 0 );
        delay(turnDelay);
        push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
        delay(stopDelay);
      } // else do nothing bc we're already stopped at left

      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
      delay(stopDelay);

      //      VL53L0X_RangingMeasurementData_t measure;
      //      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      //      if (measure.RangeStatus == 2) {
      //        Serial.print(measure.RangeMilliMeter); Serial.print("\t"); Serial.println(smoothedDistance.get());
      //      }

      //      if (measure.RangeMilliMeter < 250) {
      //        closeEnoughToBall = true;
      //      }

      if (loopsBeforeUltrasonic >= 3) {
        unsigned int candleDist = sonic.read();
        turnDelay = 100;
        if (candleDist <= 21) {
          closeEnoughToBall = true;
        }
      }

      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_FORWARD, 0 );
      delay(forwardDelay);

      return;
    case EXT_APPROACHING_FLAME:
      push( connectors, ON_MOTOR_CHANGE, 0, MOTOR_STOP, 0 );
      closeEnoughToBall = false;
      return;
    case LP_AVOIDING_BOOTH:
      // TODO implement when booth sensors work
      return;
    case ENT_DISABLED:
      Serial.println("Disabling flame follower");
      forwardFlameDet = false;
      rightFlameDet = false;
      leftFlameDet = false;
      flameSide = D_NONE;
      smoothedRight.clear();
      smoothedLeft.clear();
      smoothedForward.clear();
      smoothedDistance.clear();
      closeEnoughToBall = false;
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
Atm_flame_follower& Atm_flame_follower::start() {
  trigger( EVT_START );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::stop() {
  trigger( EVT_STOP );
  pendingStop = true;
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

/*
   onApproachingBall() push connector variants ( slots 1, autostore 0, broadcast 0 )
*/

Atm_flame_follower& Atm_flame_follower::onApproachingBall( Machine& machine, int event ) {
  onPush( connectors, ON_APPROACHING_BALL, 0, 1, 1, machine, event );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::onApproachingBall( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_APPROACHING_BALL, 0, 1, 1, callback, idx );
  return *this;
}

/*
   onNoBall() push connector variants ( slots 1, autostore 0, broadcast 0 )
*/

Atm_flame_follower& Atm_flame_follower::onNoBall( Machine& machine, int event ) {
  onPush( connectors, ON_NO_BALL, 0, 1, 1, machine, event );
  return *this;
}

Atm_flame_follower& Atm_flame_follower::onNoBall( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_NO_BALL, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
   Sets the symbol table and the default logging method for serial monitoring
*/

Atm_flame_follower& Atm_flame_follower::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
                     "FLAME_FOLLOWER\0EVT_START\0EVT_STOP\0EVT_SIDE_FLAME_DETECTED\0EVT_HALTING_COMPLETE\0EVT_FORWARD_FLAME_DETECTED\0EVT_BOOTH_AVOIDED\0EVT_BOOTH_TOO_CLOSE_RIGHT\0EVT_BOOTH_TOO_CLOSE_LEFT\0EVT_DONE_CALIBRATING\0ELSE\0CALIBRATING\0IDLE\0HALTING_FOR_FLAME\0TURNING_TOWARDS_FLAME\0APPROACHING_FLAME\0AVOIDING_BOOTH\0DISABLED" );
  return *this;
}
