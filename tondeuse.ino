#include <DualVNH5019MotorShield.h>
#include "Ultrasonic.h"

#include "Battery.h"

// Constants ------------------------------------
const bool DEBUG = false;        // activate overall logs (could be overwhelming)
const bool DEBUG_LOGS = false;   // activate printed logs for states (could be overwhelming)
const bool DEBUG_BUMPER = false; // activate bumper logs
const bool DEBUG_SONAR = false;  // activate sonar logs
// const bool DEBUG_BATTERY = false;     // activate battery logs
const bool DEBUG_MOTOR_SPEED = false; // activate motor speeds logs ; these logs are really verbose and thus not included in normal DEBUG
const float MOTOR_MAX_SPEED = 400;    // motor max speed, given by DualVNH5019MotorShield library
const float STOP_TIME = 1000;
const float REVERSE_TIME = 2500;             // Time to have motors in reverse mode
const float TURN_TIME = 1000;                // Time to have motor inversed to turn
const float SONAR_TIMEOUT = 10000UL;         // 10ms to get approx 1.7m of range
const float SONAR_SIDE_DISTANCE_SLOW = 50;   // sonar range, motor will slow down
const float SONAR_CENTER_DISTANCE_SLOW = 60; // sonar range, motor will slow down
const float SONAR_SIDE_DISTANCE_STOP = 10;   // sonar range, the mower will reverse and then turn
const float SONAR_CENTER_DISTANCE_STOP = 21; // sonar range, the mower will reverse and then turn
const float REVERSE_AND_TURN_SPEED = 400;    // motor speed when reversing and turning
const float OBSTACLE_AVOIDANCE_SPEED = 300;  // motor speed when avoiding obstacle
const float HIGH_TURN_DIVISION = 10.0;       // Divise the motor speed of the other wheel by this value
const float LOW_TURN_DIVISION = 2.0;         // Divise the motor speed of the other wheel by this value
const float MOTOR_ACCELERATION = 800;        // +=-/1000 is hypothetic value given by ardumower project
const float MOTOR_RATIO = 1;                 // Ratio between left and right motor speed, 1 = same speed, 0.5 = right motor speed is half of left motor speed, 2 = right motor speed is twice left motor speed
// ----------------------------------------------

// Pins -----------------------------------------
// JSN-SR04 Central
const int SONAR_CENTER_TRIG_PIN = 50;
const int SONAR_CENTER_ECHO_PIN = 48;

// HC-SR04 Left
const int SONAR_LEFT_TRIG_PIN = 32;
const int SONAR_LEFT_ECHO_PIN = 30;
Ultrasonic sonarLeft(SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN, SONAR_TIMEOUT);

// HC-SR04 Right
const int SONAR_RIGHT_TRIG_PIN = 28;
const int SONAR_RIGHT_ECHO_PIN = 26;
Ultrasonic sonarRight(SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN, SONAR_TIMEOUT);

// Bumper
const int BUMPER_PIN = 46;

// Led
const int LED_PIN = 42;

// Motors
DualVNH5019MotorShield md; // Use default pins

// Motor Mow
const int MOW_MOTOR_PIN = 44;

// Battery
Battery battery(A13);
// ----------------------------------------------

// Variables ------------------------------------
// Motors
bool emergencyBrake = false;
// Wanted motor speeds
float leftSpeed = 0;
float rightSpeed = 0;
// Smoothed/real motor speeds
float motorRightSpeed = 0;
float motorLeftSpeed = 0;
unsigned char motorRightFault = 0;
unsigned char motorLeftFault = 0;

// Bumper
bool bumperState = false;

// Sonar
long sonarCenterDist, sonarLeftDist, sonarRightDist;
bool stopDistSonar = false;
bool slowDistSonar = false;
// ----------------------------------------------

// Next Time ------------------------------------
unsigned long nextTimeSonar = 0;         // Next time we check sonars
unsigned long nextTimeBattery = 0;       // Next time we check battery level
unsigned long nextTimeBumper = 0;        // Next time we check bumper
unsigned long nextTimeMotorFault = 0;    // Next time we check motor faults using DualVNH5019MotorShield lib
unsigned long nextTimeFlashLed = 0;      // Next time we flash the led
unsigned long nextPrintDebug = 0;        // Next time we print logs
unsigned long lastSetMotorSpeedTime = 0; // The last time we updated motors speeds (used in new speed equation)
unsigned long stopStoppingTime = 0;      // Time given so the mower can stop during a reverseAndTurn
unsigned long stopReverseTime = 0;       // Time after which the mower should stop reverse
unsigned long stopTurnTime = 0;          // Time after which the mower should stop turn
int senSonarTurn = 0;                    // Next sonar we check (circle between center, left and right)
int senMotorFaultTurn = 0;               // Next motor we check for fault (circle between m1 and m2)
// ----------------------------------------------

// Behavior--------------------------------------
bool reverseAndTurn = false; // If true, the mower will engage a reverse and turn.
bool shoudTurnLeft = false;
// ----------------------------------------------

// Helpers --------------------------------------
void printDebug(String msg)
{
  if (DEBUG_LOGS && millis() > nextPrintDebug)
  {
    nextPrintDebug = millis() + 500;
    Serial.println(msg);
  }
}

void motorRatio()
{
  if (MOTOR_RATIO > 1)
    leftSpeed /= MOTOR_RATIO;
  else if (MOTOR_RATIO < 1)
    rightSpeed *= MOTOR_RATIO;
}
// ----------------------------------------------

// Functions ------------------------------------
/*
flashLED will flash the led at the interval freq in milliseconds.
*/
void flashLED(int pin, int freq)
{
  if (millis() >= nextTimeFlashLed)
  {
    nextTimeFlashLed = millis() + freq;

    digitalWrite(pin, HIGH);
    delay(50);
    digitalWrite(pin, LOW);
  }
}

/*
getDistance. Get the distance from a Sonar sensor (without Ultrasonic lib)
*/
float getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(SONAR_TIMEOUT);
  digitalWrite(trigPin, LOW);

  float distance = (pulseIn(echoPin, HIGH) * .0343) / 2;
  return distance;
}

/*
checkSonar will update the current ultrasonic sensor to check
*/
void checkSonar()
{
  if (millis() >= nextTimeSonar)
  {
    switch (senSonarTurn)
    {
    case 0:
      sonarCenterDist = getDistance(SONAR_CENTER_TRIG_PIN, SONAR_CENTER_ECHO_PIN);
      senSonarTurn++;
      if (DEBUG || DEBUG_SONAR)
      {
        Serial.print("Sonar Center Distance: ");
        Serial.println(sonarCenterDist);
      }
      break;
    case 1:
      sonarLeftDist = sonarLeft.read();
      senSonarTurn++;
      if (DEBUG || DEBUG_SONAR)
      {
        Serial.print("Sonar Left Distance: ");
        Serial.println(sonarLeftDist);
      }
      break;
    case 2:
      sonarRightDist = sonarRight.read();
      senSonarTurn = 0;
      if (DEBUG || DEBUG_SONAR)
      {
        Serial.print("Sonar Right Distance: ");
        Serial.println(sonarRightDist);
      }
      break;
    default:
      Serial.println("Wrong sensor…");
      senSonarTurn = 0;
    }

    stopDistSonar = inRange(SONAR_SIDE_DISTANCE_STOP, SONAR_CENTER_DISTANCE_STOP);
    slowDistSonar = inRange(SONAR_SIDE_DISTANCE_SLOW, SONAR_CENTER_DISTANCE_SLOW);
  }
}

void checkMotorFault()
{
  if (millis() >= nextTimeMotorFault)
  {
    nextTimeMotorFault = millis() + 300000; // every 5mn

    switch (senMotorFaultTurn)
    {
    case 0:
      motorRightFault = md.getM1Fault();
      senMotorFaultTurn = 1;
      break;
    case 1:
      motorLeftFault = md.getM2Fault();
      senMotorFaultTurn = 0;
      break;
    default:
      Serial.println("Wrong motor…");
      senMotorFaultTurn = 0;
      break;
    }
  }
}

void checkBumper()
{
  if (millis() >= nextTimeBumper)
  {
    nextTimeBumper = millis() + 100;

    bumperState = digitalRead(BUMPER_PIN) == 1;

    if (DEBUG || DEBUG_BUMPER)
    {
      Serial.print("Bumper State: ");
      Serial.println(bumperState);
    }
  }
}

void checkBattery()
{
  if (millis() >= nextTimeBattery)
  {
    nextTimeBattery = millis() + 60000;
    battery.update();
  }
}

/*
setMotorsSpeed set wanted speeds to a certain value.
*/
void setMotorsSpeed(int left, int right)
{
  leftSpeed = left;
  rightSpeed = right;
}

/*
adjustMotorsSpeed smoothly adjusts the motors speeds to the wanted speeds.
If emergencyBrake is true, abruptly set all speeds to 0.
*/
void adjustMotorsSpeed()
{
  if (emergencyBrake)
  {
    motorLeftSpeed = 0;
    motorRightSpeed = 0;
    leftSpeed = 0;
    rightSpeed = 0;
    emergencyBrake = false;
  }
  else
  {
    unsigned long TaC = millis() - lastSetMotorSpeedTime; // sampling time in millis
    lastSetMotorSpeedTime = millis();
    if (TaC > 1000)
      TaC = 1;

    // Assert that speeds are not above limit
    leftSpeed = constrain(leftSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

    if (leftSpeed == rightSpeed)
      motorRatio();

    motorLeftSpeed += TaC * (leftSpeed - motorLeftSpeed) / MOTOR_ACCELERATION;
    motorRightSpeed += TaC * (rightSpeed - motorRightSpeed) / MOTOR_ACCELERATION;
  }

  md.setSpeeds(motorLeftSpeed, motorRightSpeed);

  if (DEBUG_MOTOR_SPEED) // really verbose logs so not enable in global DEBUG
  {
    Serial.print("Motor Left Speed: ");
    Serial.println(motorLeftSpeed);
    Serial.print("Motor Right Speed: ");
    Serial.println(motorRightSpeed);
  }
}

/*
inRange returns true if an obstacle is in the range of one of the sonars.
It returns false systematically if one of the distance is 0.
Indeed, before a sonar is queried its distance is 0 so it'll be a false positive.
*/
bool inRange(int sideRange, int centerRange)
{
  if (!sonarCenterDist || !sonarRightDist || !sonarLeftDist)
    return false;

  return sonarCenterDist < centerRange || sonarLeftDist < sideRange || sonarRightDist < sideRange;
}
// ----------------------------------------------

void setup()
{
  Serial.flush();
  Serial.begin(9600);

  pinMode(SONAR_CENTER_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_CENTER_TRIG_PIN, LOW);
  pinMode(SONAR_CENTER_ECHO_PIN, INPUT);
  printDebug("Initialized Sonar Center.");

  pinMode(SONAR_LEFT_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_LEFT_TRIG_PIN, LOW);
  pinMode(SONAR_LEFT_ECHO_PIN, INPUT);
  printDebug("Initialized Sonar Left.");

  pinMode(SONAR_RIGHT_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_RIGHT_TRIG_PIN, LOW);
  pinMode(SONAR_RIGHT_ECHO_PIN, INPUT);
  printDebug("Initialized Sonar Right.");

  pinMode(BUMPER_PIN, INPUT_PULLUP);
  printDebug("Initialized Bumper.");

  md.init();
  printDebug("Initialized motors.");

  pinMode(LED_PIN, OUTPUT);
  printDebug("Initialized LED");

  pinMode(MOW_MOTOR_PIN, OUTPUT);
  digitalWrite(MOW_MOTOR_PIN, HIGH);
  printDebug("Initialized mow motor.");

  printDebug("Skip starting mow motor in debug mode.");
  if (!DEBUG)
    digitalWrite(MOW_MOTOR_PIN, LOW);
  else
    digitalWrite(MOW_MOTOR_PIN, HIGH);
}

void loop()
{
  checkMotorFault();
  checkBattery();
  checkBumper();
  checkSonar();
  adjustMotorsSpeed();

  if (DEBUG)
  {
    flashLED(LED_PIN, 10000);
  }

  if (motorLeftFault || motorRightFault)
  {
    setMotorsSpeed(0, 0);
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    flashLED(LED_PIN, 1000);
    printDebug("One of the motor has a fault!!!");
  }
  else if (battery.getLevel() < 20)
  {
    setMotorsSpeed(0, 0);
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    flashLED(LED_PIN, 3000);
    printDebug("Battery below 20%, stopping all motors and activate LED.");
  }
  else if (reverseAndTurn)
  {
    if (stopStoppingTime > millis())
    {
      printDebug("stopping…");
      emergencyBrake = true;
    }
    else if (stopReverseTime > millis())
    {
      printDebug("reversing…");
      setMotorsSpeed(-REVERSE_AND_TURN_SPEED, -REVERSE_AND_TURN_SPEED);
    }
    else if (stopTurnTime > millis())
    {
      printDebug("turning…");
      if (shoudTurnLeft)
        setMotorsSpeed(REVERSE_AND_TURN_SPEED, -REVERSE_AND_TURN_SPEED);
      else
        setMotorsSpeed(-REVERSE_AND_TURN_SPEED, REVERSE_AND_TURN_SPEED);
    }
    else
    {
      printDebug("stop reversing and turning");
      reverseAndTurn = false;
    }
  }
  else if (bumperState || stopDistSonar)
  {
    printDebug("Bumper has touched, stop all motors and engaging reverse and turn");
    reverseAndTurn = true;
    if (sonarRightDist < SONAR_SIDE_DISTANCE_STOP)
      shoudTurnLeft = false;
    else
      shoudTurnLeft = true;
    stopStoppingTime = millis() + STOP_TIME;
    stopReverseTime = stopStoppingTime + REVERSE_TIME;
    stopTurnTime = stopReverseTime + TURN_TIME;
  }
  // else if (stopDistSonar)
  // {
  //   printDebug("Obstacle in critical zone, engaging reverse and turn");
  //   reverseAndTurn = true;
  //   stopStoppingTime = millis() + STOP_TIME;
  //   stopReverseTime = stopStoppingTime + REVERSE_TIME;
  //   stopTurnTime = stopReverseTime + TURN_TIME;
  // }
  // Obstacle near anywhere
  else if (slowDistSonar)
  {
    float leftSpeed = 0;
    float rightSpeed = 0;
    if (sonarLeftDist < SONAR_SIDE_DISTANCE_SLOW)
    {
      if (sonarCenterDist < SONAR_CENTER_DISTANCE_SLOW) // Turn right
      {
        leftSpeed = OBSTACLE_AVOIDANCE_SPEED;
        rightSpeed = OBSTACLE_AVOIDANCE_SPEED / HIGH_TURN_DIVISION;
      }
      else // Turn slightly less
      {
        leftSpeed = OBSTACLE_AVOIDANCE_SPEED;
        rightSpeed = OBSTACLE_AVOIDANCE_SPEED / LOW_TURN_DIVISION;
      }
    }
    else if (sonarRightDist < SONAR_SIDE_DISTANCE_SLOW)
    {
      if (sonarCenterDist < SONAR_CENTER_DISTANCE_SLOW) // Turn left
      {
        leftSpeed = OBSTACLE_AVOIDANCE_SPEED / HIGH_TURN_DIVISION;
        rightSpeed = OBSTACLE_AVOIDANCE_SPEED;
      }
      else // Turn slightly less
      {
        leftSpeed = OBSTACLE_AVOIDANCE_SPEED / LOW_TURN_DIVISION;
        rightSpeed = OBSTACLE_AVOIDANCE_SPEED;
      }
    }
    else // Only on the center
    {
      leftSpeed = OBSTACLE_AVOIDANCE_SPEED;
      rightSpeed = OBSTACLE_AVOIDANCE_SPEED;
    }
    setMotorsSpeed(leftSpeed, rightSpeed);
  }
  else
  {
    printDebug("Straight Forward Captain!");
    setMotorsSpeed(MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  }
}
