// This demo shows how the Romi can use its gyroscope to detect
// when it is being rotated, and use the motors to resist that
// rotation.  If you place the robot on a rotating platter and
// spin it, it should keep pointing in approximately the same
// direction.
//
// This code was tested on a Romi with a 120:1 Mini Plastic
// Gearmotor HP and 6 NiMH batteries.  If you have different
// motors or batteries, you might need to adjust the PID
// constants.
//
// Be careful to not move the robot for a few seconds after
// starting it while the gyro is being calibrated.  During the
// gyro calibration, the yellow LED is on and the words "Gyro
// cal" are displayed on the LCD.
//
// After the gyro calibration is done, press button A to start
// the demo.  If you try to turn the Romi, or put it on a surface
// that is turning, it will drive its motors to counteract the
// turning.
//
// This sketch only uses the Z axis of the gyro, so it is
// possible to pick up the Romi, rotate it about its X and Y
// axes, and then put it down facing in a new position.
//
// To run this sketch, you will need to install the LSM6 library:
//
// https://github.com/pololu/lsm6-arduino


#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>
#include "TurnSensor.h"

const int16_t maxSpeed = 100;

Romi32U4LCD lcd;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4Motors motors;
LSM6 imu;
int deg = 90;

void setup()
{
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  
}

void turn(int degrees)
{
  int delay = 0;
  int32_t turnSpeed = 1;
  while(turnSpeed != 0 && delay < 200) {
    turnSensorUpdate();
    // Read the gyro to update turnAngle, the estimation of how far
    // the robot has turned, and turnRate, the estimation of how
    // fast it is turning.
    int multiplier = degrees/45;
    uint32_t turn= 0x20000000*(multiplier);
    Serial.println((((int32_t)turnAngle >> 16) * 360) >> 16);
    
    //Serial.println((((int32_t)turnAngle >> 16) * 360) >> 16);
    // if(abs((int)((((int32_t)turnAngle >> 16) * 360) >> 16)) < 5) {
    //    turnAngle = turn;
    // }
    
    // Calculate the motor turn speed using proportional and
    // derivative PID terms.  Here we are a using a proportional
    // constant of 15 and a derivative constant of 1/73.
    turnSpeed = -(int32_t)(turnAngle-turn)/ (turnAngle1 / 15) - turnRate / 73;
    // Serial.println((int32_t)turnAngle1);
    // Serial.println(turnSpeed);

    // Constrain our motor speeds to be between
    // -maxSpeed and maxSpeed.
    turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);

    motors.setSpeeds(-turnSpeed, turnSpeed);
    if(turnSpeed == 0) {
      delay++;
    }
  }
}
void loop(){
  buttonA.waitForButton();
  turn(180);
  turn(0);
  turn(-90);
  turn(90);
}