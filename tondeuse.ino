#include <DualVNH5019MotorShield.h>
#include "Ultrasonic.h"

#include "Battery.h"
#include "LineSensor.h"

// Constants ------------------------------------
const bool DEBUG = false;            // activate overall logs (could be overwhelming)
const bool DEBUG_LOGS = false;       // activate printed logs for states (could be overwhelming)
const bool DEBUG_BUMPER = false;     // activate bumper logs
const bool DEBUG_SONAR = false;      // activate sonar logs
const bool DEBUG_LINESENSOR = false; // activate line sensor logs (decorrelated from overall logs)
const bool DEBUG_BATTERY = false;     // activate battery logs
const bool DEBUG_MOTOR_SPEED = false; // activate motor speeds logs ; these logs are really verbose and thus  not included in normal DEBUG
const bool DEBUG_LINE = false;         // activate line following logs
const float MOTOR_MAX_SPEED = 400;    // motor max speed, given by DualVNH5019MotorShield library
const float STOP_TIME = 200;
const float REVERSE_TIME = 2000;                 // Time to have motors in reverse mode
const float TURN_TIME = 1000;                    // Time to have motor inversed to turn
const float SONAR_TIMEOUT = 10000UL;             // 10ms to get approx 1.7m of range
const float SONAR_SIDE_DISTANCE_SLOW = 30;       // sonar range, motor will slow down
const float SONAR_CENTER_DISTANCE_SLOW = 40;     // sonar range, motor will slow down
const float SONAR_SIDE_DISTANCE_STOP = 10;       // sonar range, the mower will reverse and then turn
const float SONAR_CENTER_DISTANCE_STOP = 21;     // sonar range, the mower will reverse and then turn
const float REVERSE_AND_TURN_SPEED = 400;        // motor speed when reversing and turning
const float OBSTACLE_AVOIDANCE_SPEED = 300;      // motor speed when avoiding obstacle
const float HIGH_TURN_DIVISION = 10.0;           // Divise the motor speed of the other wheel by this value
const float LOW_TURN_DIVISION = 2.0;             // Divise the motor speed of the other wheel by this value
const float MOTOR_ACCELERATION = 800;            // +=-/1000 is hypothetic value given by ardumower project
const float MOTOR_RATIO = 1.3;                   // Ratio between left and right motor speed, 1 = same speed, 0.5 = right motor speed is half of left motor speed, 2 = right motor speed is twice left motor speed
const bool IS_BASE_STATION_CLOCKWISE = false;    // Should the mower go clockwise or counter clockwise to go back to the base station
const float LINE_SENSOR_THRESHOLD = 150;         // Level below which there is no line
const float LINE_SENSOR_FOLLOW_LINE_VALUE = 250; // Level the mower will try to keep when following the line using the main sensor (depending on IS_BASE_STATION_CLOCKWISE, see getLineSensors function)
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

// Line Sensors
LineSensor rightLineSensor(A11, LINE_SENSOR_THRESHOLD, DEBUG_LINESENSOR);
LineSensor leftLineSensor(A12, LINE_SENSOR_THRESHOLD, DEBUG_LINESENSOR);
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

// Line
bool lineState = false;
float lastLevelFollowingLine = 0;
int lastFollowingLine = true; // 1 or -1
// ----------------------------------------------

// Next Time ------------------------------------
unsigned long nextTimeSonar = 0;         // Next time we check sonars
unsigned long nextTimeBattery = 0;       // Next time we check battery level
unsigned long nextTimeLineSensor = 0;    // Next time we check line sensors
unsigned long nextTimeBumper = 0;        // Next time we check bumper
unsigned long nextTimeMotorFault = 0;    // Next time we check motor faults using DualVNH5019MotorShield lib
unsigned long nextTimeFlashLed = 0;      // Next time we flash the led
unsigned long nextPrintDebug = 0;        // Next time we print logs
unsigned long lastSetMotorSpeedTime = 0; // The last time we updated motors speeds (used in new speed equation)
unsigned long stopStoppingTime = 0;      // Time given so the mower can stop during a doReverseAndTurn
unsigned long stopReverseTime = 0;       // Time after which the mower should stop reverse
unsigned long stopTurnTime = 0;          // Time after which the mower should stop turn
int senSonarTurn = 0;                    // Next sonar we check (circle between center, left and right)
int senMotorFaultTurn = 0;               // Next motor we check for fault (circle between m1 and m2)
// ----------------------------------------------

// Behavior--------------------------------------
bool doReverseAndTurn = false; // If true, the mower will engage a reverse and turn.
bool shoudTurnLeft = false;
bool doFindLine = false;
bool doFollowLine = false;
bool doNothing = false;
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

    bumperState = digitalRead(BUMPER_PIN) == LOW;

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

void checkLineSensor()
{
  if (millis() >= nextTimeLineSensor)
  {
    nextTimeLineSensor = millis() + 100;
    rightLineSensor.update();
    leftLineSensor.update();
  }
  if (rightLineSensor.isAboveLine() || leftLineSensor.isAboveLine())
    lineState = true;
  else
    lineState = false;
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

/*
adaptSpeed will adapt the speed of the motors if an obstacle is near.
*/
void adaptSpeed()
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

/*
prepareReverseAndTurn will prepare the mower to reverse and turn by setting the right flags and times.
*/
void prepareReverseAndTurn()
{
  doReverseAndTurn = true;
  if (sonarRightDist < SONAR_SIDE_DISTANCE_STOP)
    shoudTurnLeft = false;
  else
    shoudTurnLeft = true;
  stopStoppingTime = millis() + STOP_TIME;
  stopReverseTime = stopStoppingTime + REVERSE_TIME;
  stopTurnTime = stopReverseTime + TURN_TIME;
}

/*
reverseAndTurn will reverse the mower and turn it.
*/
void reverseAndTurn()
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
    doReverseAndTurn = false;
  }
}

/*
getLineSensors will return the line sensors in the right order depending on the base station direction.
*/
void getLineSensors(LineSensor &main, LineSensor &secondary)
{
  IS_BASE_STATION_CLOCKWISE ? (main = leftLineSensor, secondary = rightLineSensor) : (main = rightLineSensor, secondary = leftLineSensor);
}

/*
findLine will make the mower go back to the perimeter.
*/
void findLine()
{
  setMotorsSpeed(MOTOR_MAX_SPEED * 0.75, MOTOR_MAX_SPEED * 0.75);
  flashLED(LED_PIN, 500);
  if (stopDistSonar || bumperState)
  {
    prepareReverseAndTurn();
    return;
  }

  // LineSensor goodLineSensor, badLineSensor;
  // getLineSensors(goodLineSensor, badLineSensor);

  if (lineState)
  {
    emergencyBrake = true;
    doFindLine = false;
    doFollowLine = true;
  }
}

/*
followLine will make the mower follow the perimeter until it reaches the base station.
*/
void followLine()
{
  int maxSpeed = MOTOR_MAX_SPEED * 0.75;
  flashLED(LED_PIN, 100);
  // Stop everything, we have probably reached the base station
  if (bumperState)
    doNothing = true;

  // No line sensor is above the line, we have lost the line. Let's find it again.
  bool lostLine = !leftLineSensor.isAboveLine() && !rightLineSensor.isAboveLine();
  if (lostLine)
  {
    doFollowLine = false;
    doFindLine = true;
  }

  // LineSensor goodLineSensor, badLineSensor;
  // getLineSensors(goodLineSensor, badLineSensor);

  if (rightLineSensor.isAboveLine())
  {
    if (rightLineSensor.getLevel() > LINE_SENSOR_FOLLOW_LINE_VALUE)
    {
      setMotorsSpeed(maxSpeed, maxSpeed);
    }
    // It is worst than the last time, we are going away from the line
    else if (rightLineSensor.getLevel() < lastLevelFollowingLine)
    {
      lastFollowingLine = !lastFollowingLine;
      if (lastFollowingLine)
        setMotorsSpeed(maxSpeed, 0);
      else
        setMotorsSpeed(0, maxSpeed);
    }
  }
  else if (leftLineSensor.isAboveLine())
  {
    if (IS_BASE_STATION_CLOCKWISE)
      setMotorsSpeed(maxSpeed, 0);
    else
      setMotorsSpeed(0, maxSpeed);
  }
  lastLevelFollowingLine = rightLineSensor.getLevel();
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
  checkLineSensor();
  adjustMotorsSpeed();

  bool motorFault = motorRightFault || motorLeftFault;
  bool batteryLow = battery.getLevel() < 20;
  bool shouldReverseAndTurn = bumperState || stopDistSonar || lineState;

  if (doNothing || DEBUG)
  {
    setMotorsSpeed(0, 0);
    flashLED(LED_PIN, 10000);
  }

  if (motorFault)
  {
    setMotorsSpeed(0, 0);
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    flashLED(LED_PIN, 1000);
    printDebug("One of the motor has a fault!!!");
  }
  else if (doFollowLine)
  {
    followLine();
  }
  else if (!doFindLine && (batteryLow || DEBUG_LINE))
  {
    printDebug("Battery below 20%, stopping mowing motors and going to find the perimeter.");
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    doFindLine = true;
  }
  else if (doReverseAndTurn)
  {
    reverseAndTurn();
  }
  else if (doFindLine)
  {
    findLine();
  }
  else if (shouldReverseAndTurn)
  {
    printDebug("Either the bumper has touched, the sonar is in critical zone or we are about to cross perimeter; stop all motors and preparing a reverse and turn");
    prepareReverseAndTurn();
  }
  // Obstacle near anywhere
  else if (slowDistSonar)
  {
    adaptSpeed();
  }
  else
  {
    printDebug("Straight Forward Captain!");
    setMotorsSpeed(MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  }
}
