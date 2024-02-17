#include <Romi32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include <PID_v1.h>
#include <SimpleKalmanFilter.h>
#include "Kalman.h"

Romi32U4Motors motors;
Romi32U4Encoders encoders;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
LSM6 imu;

int offset;
double currentDegree = 0.0;
double lastDegree = 0.0;
double lastX = 0.0;
double lastY = 0.0;
bool isFirstPoint = true;

int trigPin = 11;    // Trigger
int echoPin = 12;    // Echo
long duration;
double cm, inches;

float e_mea = 1;  // Measurement Uncertainty - How much do we expect to our measurement vary
float e_est = 1;  // Estimation Uncertainty - Can be initialized with the same value as e_mea since the kalman filter will adjust this over time
float q = 0.001;   // Process Noise - A smaller process noise indicates that you trust the model's predictions more

SimpleKalmanFilter kalman0(e_mea, e_est, 0.005);
SimpleKalmanFilter kalman1(e_mea, e_est, 0.01);
SimpleKalmanFilter kalman2(e_mea, e_est, 0.1);

Kalman kalmanZ;
unsigned long timer;

// Define the PID variables
double Setpoint, Input, Output;

// Define the PID tuning parameters (Kp, Ki, Kd)
double Kp = 0.4, Ki = 0.0, Kd = 0.0;

// Initialize the PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (!imu.init()) {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01010000);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  //calibrateGyro(10000);
}

void calculateArea(double x, double y) {
  if (!isFirstPoint) {
    double h = x - lastX;
    double area = (lastY + y) * h / 2.0;
    currentDegree += area;
  } else {
    isFirstPoint = false;
  }

  lastX = x;
  lastY = y;

  //currentDegree = fmod(currentDegree, 360.0);
}

void updateAngle() {
  imu.read();
  double angVelocity = (imu.g.z - offset) / 114.285714286;
  
  // double kalman_angVelocity0 = kalman0.updateEstimate(angVelocity);
  // double kalman_angVelocity1 = kalman1.updateEstimate(angVelocity);
  //double kalman_angVelocity2 = currentDegree*0.98 + 0.02*accAngleZ;
  double kalman_angVelocity2 = kalman2.updateEstimate(angVelocity);

  timer = micros();
  //double kalman_angVelocity = angVelocity;

  // if (abs(kalman_angVelocity2) < 1) {
  //   kalman_angVelocity2 = 0;
  // }

  calculateArea(micros() / 1000000.0, kalman_angVelocity2);
  // Serial.print(currentDegree);
  // Serial.print(",");
  // Serial.print(kalman_angVelocity0);
  // Serial.print(",");
  // Serial.print(kalman_angVelocity1);
  // Serial.print(",");
  // Serial.println(kalman_angVelocity2);
  // Serial.print(",");
  // Serial.println(currentDegree);
  
}

void turn(double degree) {
  double diff = currentDegree - degree;

  if(fabs(currentDegree - degree) > fabs(currentDegree + 360 - degree)) {
    currentDegree += 360;
  } else if(fabs(currentDegree - degree) > fabs(currentDegree - 360 - degree)) {
    currentDegree -= 360;
  }
  
  //Serial.println(currentDegree);

  int startSpeed = 70;
  int endSpeed = 20;
  double originalDegree = currentDegree;
  double slope = (endSpeed - startSpeed) / (degree - currentDegree);
  while (fabs(currentDegree - degree) >= 0.02) {
    updateAngle();
    int speed = slope * (currentDegree - originalDegree) + startSpeed;

    // Serial.print(slope);
    // Serial.print(" : ");
    // Serial.println(speed);

    if (currentDegree - degree < 0) {
      motors.setSpeeds(-speed, speed);
    } else {
      motors.setSpeeds(speed, -speed);
    }
  }

  isFirstPoint = true;
  motors.setSpeeds(0, 0);
  delay(50);
  //Serial.println(currentDegree);
  //Serial.println("DONEDONEDONE");
}

void calibrateGyro(int time) {
  // find gyro error
  long startTime = millis();
  long endTime = millis();
  long gyroTotal = 0;
  long count = 0;

  while (endTime - startTime < time) {
    imu.read();
    gyroTotal += imu.g.z;
    count++;
    endTime = millis();
  }

  offset = gyroTotal / count;

  timer = micros();
  
}


long cmToCounts(float cm) {
  double wheelCircumferenceCm = 7.0 * PI;
  double encoderCountsPerRevolution = 1440;

  return (long)(cm / wheelCircumferenceCm * encoderCountsPerRevolution);
}

void move(double distance) {
  long encoderCount = cmToCounts(distance);

  // Reset encoders
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  // PID setup
  Setpoint = 0; // We want no difference between left and right encoders
  myPID.SetMode(AUTOMATIC); // Set PID to automatic mode
  myPID.SetOutputLimits(-230, 230); // Set maximum and minimum output limits
  double speed = 40;
  if(distance > 0) {
    while(encoders.getCountsLeft() < encoderCount) {
      Input = encoders.getCountsLeft() - encoders.getCountsRight();
      myPID.Compute();
      if(speed < 200 && encoders.getCountsLeft() < 0.7*encoderCount) {
        speed += 0.25;
      } else if(encoders.getCountsLeft() > 0.75*encoderCount){
        if(speed > 50) {
          speed -= 0.4;
        }
      }
      motors.setSpeeds(speed + Output, speed - Output);
      updateAngle();
    }
  } else {
    while(encoders.getCountsLeft() > encoderCount) {
      Input = encoders.getCountsRight() - encoders.getCountsLeft();
      myPID.Compute();

      if(speed < 200 && encoders.getCountsLeft() > 0.7*encoderCount) {
        speed += 0.25;
      } else if(encoders.getCountsLeft() < 0.75*encoderCount) { 
        if(speed > 50) {
          speed -= 0.4;
        }
      }
      motors.setSpeeds(-(speed + Output), -(speed - Output));
      updateAngle();
    }
  }
  //Serial.println(currentDegree);
  motors.setSpeeds(0, 0);
  updateAngle();
  delay(250);
  
}

double ultrasoundDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  double cm_dist = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  return cm_dist;
}

void endPosition(){

  double offsetX = -8.5;
  double offsetY = -21;

  // double dist = ultrasoundDistance();

  // while(fabs(dist - 25 - offsetX) > 0.5) {
  //   delay(100);
  //   move(-(dist - 25 - offset));
  //   dist = ultrasoundDistance(); 
  // }

  // turn(0);
  // delay(100);

  // dist = ultrasoundDistance();

  // while(fabs(dist - 25 - offsetY) > 0.5) {
  //   delay(100);
  //   move(-(dist - 25 - offset));
  //   dist = ultrasoundDistance(); 
  // }
  // delay(100);
  double dist = 0;
  
  dist = ultrasoundDistance();
  move(-(dist-23-offsetX)); // 23cm off since 25cm - 0.75in (2cm) = 23cm
  delay(100);

  dist = ultrasoundDistance();
  move(-(dist-23-offsetX));
  delay(100);

  turn(0);
  delay(100);

  dist = ultrasoundDistance();
  move(-(dist-25-offsetY));
  delay(100);

  dist = ultrasoundDistance();
  move(-(dist-25-offsetY));
  delay(100);

}

void loop() {
  //Serial.println(currentDegree);
  buttonA.waitForButton();  // Wait until Button A is pressed
  delay(250);
  //currentDegree = 0;
  //updateAngle();
  calibrateGyro(10000);
  int N = 0;
  int E = -90;
  int S = 180;
  int W = 90;
  
  move(36);
  turn(E);
  move(50);
  turn(N);
  move(50);
  turn(W);
  move(30);
  move(-30);
  turn(N);
  move(80);
  move(-30);
  turn(W);
  move(75);
  turn(135);
  move(3*25*1.42);
  turn(N);
  move(110);
  move(-110);
  turn(-45);
  move(2*25*1.42);
  turn(N);
  move(80);
  turn(W);
  endPosition();
}