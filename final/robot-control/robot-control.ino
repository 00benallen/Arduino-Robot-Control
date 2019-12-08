/** Motors **/
#include "Atm_motors.h"
Atm_motors motors(3, 4, 2, 1);

/** Motor Constants **/
const int MOTOR_SPEED_LOW = 120;
const int MOTOR_SPEED_HIGH = 255;

/** Sensors **/

// 9-Axis IMU
#include "Atm_imu.h"
Atm_imu IMU;

// IR TOF Sensor
//#include "Adafruit_VL53L0X.h"
//Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#include "Atm_line_navigator.h"
Atm_line_navigator lineNav;

#include "Atm_flame_follower.h"
Atm_flame_follower flameFollower;

/** Grabber **/
#include "Atm_ball_grabber.h"
Atm_ball_grabber ball_grabber(31, 29);

/** Miscellaneous **/
int randomSeedPin = 15;

void setup()
{
  Serial.begin(9800);
  Serial.println("[CPS603-Robot Control Program] starting up");

  ball_grabber.begin().trace ( Serial ).stop().onBallgrabbed( [] (int idx, int v, int up) {

    Serial.println("Stopping flame follower");
    flameFollower.stop();
    Serial.println("Stoppping motors");
    motors.stop();
    Serial.println("Restarting line nav");
    lineNav.backup();

  });

  motors.begin(MOTOR_SPEED_LOW).trace( Serial );
  motors.enable(); // comment out to disable motors


  //
  IMU.begin()
  .onTurnend(lineNav, Atm_line_navigator::EVT_INTERSECTION_TURN_COMPLETE)
  .trace( Serial );

  //  initializeTOF();

  flameFollower.begin(A8, A9, A10)
  .trace ( Serial )
  .onMotorChange( [] (int idx, int v, int up) {
    switch (v) {
      case Atm_flame_follower::MOTOR_LEFT:
        motors.left();
        return;
      case Atm_flame_follower::MOTOR_RIGHT:
        motors.right();
        return;
      case Atm_flame_follower::MOTOR_FORWARD:
        motors.forward();
        return;
      case Atm_flame_follower::MOTOR_STOP:
        motors.stop();
        return;
    }
    return;
  })
  //  .onFlamedetected( lineNav, Atm_line_navigator::EVT_STOP )
  .onFlamedetected( [] (int idx, int v, int up) {

    switch (v) {
      case Atm_flame_follower::D_LEFT:
        lineNav.stop();
      case Atm_flame_follower::D_RIGHT:
        lineNav.stop();
      case Atm_flame_follower::D_FORWARD:
        lineNav.dropoff_detected();
        return;
    }
  })
  .onFlamehandled( lineNav, Atm_line_navigator::EVT_START )
  .onApproachingBall ( ball_grabber, Atm_ball_grabber::EVT_START )
  .stop();

  lineNav.begin((const uint8_t[]) {
    53, 51, 49, 47, 45, 43, 41, 39
  }, 52, 22)
  .onMotorChange( [] (int idx, int v, int up) {
    switch (v) {
      case Atm_line_navigator::MOTOR_BACKWARD:
        motors.backward();
        return;
      case Atm_line_navigator::MOTOR_LEFT:
        motors.left();
        return;
      case Atm_line_navigator::MOTOR_RIGHT:
        motors.right();
        return;
      case Atm_line_navigator::MOTOR_FORWARD:
        motors.forward();
        return;
      case Atm_line_navigator::MOTOR_STOP:
        motors.stop();
        return;
    }
    return;
  })
  .onTurnStart([] (int idx, int v, int up) {
    IMU.track_turn(v);
    flameFollower.stop();
  })
  .onTurnEnd([] (int idx, int v, int up) {
    flameFollower.start();
  })
  .trace( Serial )
  .start();
}

//void initializeTOF() {
//  Serial.println("Initializing VL53L0X laser TOF sensor");
//  Serial.println("Adafruit VL53L0X test");
//  while (!lox.begin()) {
//    Serial.println(F("Failed to boot VL53L0X"));
//    delay(1000);
//  }
//  Serial.println("VL53L0X laser TOF sensor detected");
//}

void loop()
{
  //  motors.disable(); // comment out if you want to move during loops
  automaton.run();
}
