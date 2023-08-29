#include <DualVNH5019MotorShield.h>
#include "Ultrasonic.h"

// Constants ------------------------------------
const bool DEBUG = false;
const float MOTOR_MAX_SPEED = 400;
const float SONAR_TIMEOUT = 10000UL;      // 20ms to get approx 3.4m of range
const float SONAR_MIN_DISTANCE = 50;      // 50cm
const float SONAR_CRITICAL_DISTANCE = 10; // 10cm
const float MOTOR_ACCELERATION = 500;
// ----------------------------------------------

// Pins -----------------------------------------
// JSN-SR04 Central
const int SONAR_CENTER_TRIG_PIN = 50;
const int SONAR_CENTER_ECHO_PIN = 48;
Ultrasonic sonarCenter(SONAR_CENTER_TRIG_PIN, SONAR_CENTER_ECHO_PIN, SONAR_TIMEOUT);

// HC-SR04 Left
const int SONAR_LEFT_TRIG_PIN = 32;
const int SONAR_LEFT_ECHO_PIN = 30;
Ultrasonic sonarLeft(SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN, SONAR_TIMEOUT);

// HC-SR04 Right
const int SONAR_RIGHT_TRIG_PIN = 28;
const int SONAR_RIGHT_ECHO_PIN = 26;
Ultrasonic sonarRight(SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN, SONAR_TIMEOUT);

// Bumper
const int BUMPER_PIN = 41;

// Motors
DualVNH5019MotorShield md; // Use default pins

// Motor Mow
const int MOW_MOTOR_PIN = 40;

// Battery
const int BATTERY_PIN = A13;
const float BATTERY_MIN_VOLTAGE = 11.2;
const float BATTERY_MIN_VALUE = (1023 * BATTERY_MIN_VOLTAGE) / 5;
const float BATTERY_MAX_VOLTAGE = 14.4;
const float BATTERY_MAX_VALUE = (1023 * BATTERY_MAX_VOLTAGE) / 5;
// ----------------------------------------------

// Variables ------------------------------------
// Motor speeds
float motorRightSpeed = 0;
float motorLeftSpeed = 0;
unsigned char motorRightFault = 0;
unsigned char motorLeftFault = 0;

// Bumper
bool bumperState = false;

// Sonar
long sonarCenterDist, sonarLeftDist, sonarRightDist;

// Battery
float batteryLevel = 100;
// ----------------------------------------------

// Next Time ------------------------------------
unsigned int nextTimeSonar = 0;
unsigned int nextTimeBattery = 0;
unsigned int nextTimeBumper = 0;
unsigned int nextTimeMotorFault = 0;
unsigned int lastSetMotorSpeedTime = 0;

int senSonarTurn = 0;
int senMotorFaultTurn = 0;

// Helpers --------------------------------------
void printDebug(String msg)
{
  if (DEBUG)
  {
    Serial.println(msg);
  }
}
// ----------------------------------------------

// Functions ------------------------------------
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
    if (DEBUG)
    {
      nextTimeSonar = millis() + 250;
    }
    else
    {
      nextTimeSonar = millis() + 250;
    }

    switch (senSonarTurn)
    {
    case 0:
      sonarCenterDist = getDistance(SONAR_CENTER_TRIG_PIN, SONAR_CENTER_ECHO_PIN);
      senSonarTurn++;
      if (DEBUG)
      {
        Serial.print("Sonar Center Distance: ");
        Serial.println(sonarCenterDist);
      }
      break;
    case 1:
      sonarLeftDist = sonarLeft.read();
      senSonarTurn++;
      if (DEBUG)
      {
        Serial.print("Sonar Left Distance: ");
        Serial.println(sonarLeftDist);
      }
      break;
    case 2:
      sonarRightDist = sonarRight.read();
      senSonarTurn = 0;
      if (DEBUG)
      {
        Serial.print("Sonar Right Distance: ");
        Serial.println(sonarRightDist);
      }
      break;
    default:
      Serial.println("Wrong sensor…");
      senSonarTurn = 0;
    }
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
      Serial.println("Wrong motor…")
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

    bumperState = digitalRead(BUMPER_PIN) == 0;

    if (DEBUG)
    {
      Serial.print("Bumper State: ");
      Serial.println(bumperState);
      nextTimeBumper = millis() + 1000;
    }
  }
}

void checkBattery()
{
  if (millis() >= nextTimeBattery)
  {
    nextTimeBattery = millis() + 60000;

    float result = ((analogRead(BATTERY_PIN) - BATTERY_MIN_VALUE) / (BATTERY_MAX_VALUE - BATTERY_MIN_VALUE)) * 100;

    if (result > 100)
      result = 100;
    else if (result < 0)
      result = 0;

    batteryLevel = result;
    if (DEBUG)
    {
      Serial.print("Battery Level: ");
      Serial.println(batteryLevel);
    }
  }
}

void motorSpeed(int speedLeft, int speedRight)
{
  unsigned long TaC = millis() - lastSetMotorSpeedTime; // sampling time in millis
  lastSetMotorSpeedTime = millis();
  if (TaC > 1000)
    TaC = 1;

  // Assert that speeds are not above limit
  speedLeft = constrain(speedLeft, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  speedRight = constrain(speedRight, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

  motorLeftSpeed += TaC * (speedLeft - motorLeftSpeed) / MOTOR_ACCELERATION;
  motorRightSpeed += TaC * (speedRight - motorRightSpeed) / MOTOR_ACCELERATION;

  md.setSpeeds(motorRightSpeed, motorLeftSpeed);
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

  pinMode(BUMPER_PIN, INPUT);
  digitalWrite(BUMPER_PIN, INPUT_PULLUP);
  printDebug("Initialized Bumper.");

  md.init();
  printDebug("Initialized motors.");

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
  // checkBattery();
  checkBumper();
  checkSonar();

  if (motorLeftFault || motorRightFault)
  {
    // TODO: Clignoter la LED
    printDebug("One of the motor has a fault!!!");
  }
  
  if (batteryLevel < 20)
  {
    // printDebug("Battery below 20%, stopping all motors and activate LED.");
    motorSpeed(0, 0);
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    // TODO: Clignoter la LED
  }
  else if (sonarCenterDist < SONAR_CRITICAL_DISTANCE || sonarLeftDist < SONAR_CRITICAL_DISTANCE || sonarRightDist < SONAR_CRITICAL_DISTANCE || bumperState)
  {
    // printDebug("Obstacle in critical zone");
    motorSpeed(0, 0);
    // TODO: Reculer et tourner dans le sens inverse de l'obstacle.
  }
  // Obstacle near anywhere
  else if (sonarCenterDist < SONAR_MIN_DISTANCE || sonarLeftDist < SONAR_MIN_DISTANCE || sonarRightDist < SONAR_MIN_DISTANCE)
  {
    // printDebug("Obstacle detected, slowing down");
    motorSpeed(25, 25);
  }
  else
  {
    // printDebug("Straight Forward Captain!");
    motorSpeed(400, 400);
  }
}
