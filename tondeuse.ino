#include <DualVNH5019MotorShield.h>
#include "Ultrasonic.h"

// Constants ------------------------------------
const bool DEBUG = false;                 // activate overall logs (could be overwhelming)
const bool DEBUG_BUMPER = false;          // activate bumper logs
const bool DEBUG_SONAR = false;           // activate sonar logs
const bool DEBUG_BATTERY = false;         // activate battery logs
const bool DEBUG_MOTOR_SPEED = false;     // activate motor speeds logs ; these logs are really verbose and thus not included in normal DEBUG
const float MOTOR_MAX_SPEED = 400;        // motor max speed, given by DualVNH5019MotorShield library
const float REVERSE_TIME = 3000;          // Time to have motors in reverse mode
const float TURN_TIME = 3000;             // Time to have motor inversed to turn
const float SONAR_TIMEOUT = 10000UL;      // 10ms to get approx 1.7m of range
const float SONAR_MIN_DISTANCE = 80;      // 80cm at that sonar range, motor will slow down
const float SONAR_CRITICAL_DISTANCE = 21; // 21cm at that sonar range, the mower will reverse and then turn
const float MOTOR_ACCELERATION = 1000;    // hypothetic value given by ardumower project
const float BATTERY_MIN_VOLTAGE = 0.5;    // Battery minimum voltage
const float BATTERY_MAX_VOLTAGE = 5;      // Battery maximum voltage
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
const int BUMPER_PIN = 41;

// Led
const int LED_PIN = 3;

// Motors
DualVNH5019MotorShield md; // Use default pins

// Motor Mow
const int MOW_MOTOR_PIN = 40;

// Battery
const int BATTERY_PIN = A13;
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
bool criticalDistSonar = false;
bool minDistSonar = false;

// Battery
float batteryLevel = 100;
// ----------------------------------------------

// Next Time ------------------------------------
unsigned int nextTimeSonar = 0;         // Next time we check sonars
unsigned int nextTimeBattery = 0;       // Next time we check battery level
unsigned int nextTimeBumper = 0;        // Next time we check bumper
unsigned int nextTimeMotorFault = 0;    // Next time we check motor faults using DualVNH5019MotorShield lib
unsigned int nextTimeFlashLed = 0;      // Next time we flash the led
unsigned int lastSetMotorSpeedTime = 0; // The last time we updated motors speeds (used in new speed equation)
unsigned int stopReverseTime = 0;       // Time after which the mower should stop reverse
unsigned int stopTurnTime = 0;          // Time after which the mower should stop turn
int senSonarTurn = 0;                   // Next sonar we check (circle between center, left and right)
int senMotorFaultTurn = 0;              // Next motor we check for fault (circle between m1 and m2)
// ----------------------------------------------

// Behavior--------------------------------------
bool reverseAndTurn = false; // If true, the mower will engage a reverse and turn.
// ----------------------------------------------

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

    bumperState = digitalRead(BUMPER_PIN) == 0;

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

    float measure = analogRead(BATTERY_PIN);
    float voltage = measure * 5.0 / 1023;
    batteryLevel = map(voltage, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0, 100);
    batteryLevel = constrain(batteryLevel, 0, 100);

    if (DEBUG || DEBUG_BATTERY)
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

  md.setSpeeds(motorLeftSpeed, motorRightSpeed);

  if (DEBUG_MOTOR_SPEED) // really verbose logs so not enable in
  {
    Serial.print("Motor Left Speed: ");
    Serial.println(motorLeftSpeed);
    Serial.print("Motor Right Speed: ");
    Serial.println(motorRightSpeed);
  }
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

  criticalDistSonar = sonarCenterDist < SONAR_CRITICAL_DISTANCE || sonarLeftDist < SONAR_CRITICAL_DISTANCE || sonarRightDist < SONAR_CRITICAL_DISTANCE;
  minDistSonar = sonarCenterDist < SONAR_MIN_DISTANCE || sonarLeftDist < SONAR_MIN_DISTANCE || sonarRightDist < SONAR_MIN_DISTANCE;

  if (DEBUG)
  {
    flashLED(LED_PIN, 10000);
  }

  if (motorLeftFault || motorRightFault)
  {
    motorSpeed(0, 0);
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    flashLED(LED_PIN, 1000);
    printDebug("One of the motor has a fault!!!");
  }
  else if (batteryLevel < 20)
  {
    motorSpeed(0, 0);
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    flashLED(LED_PIN, 3000);
    printDebug("Battery below 20%, stopping all motors and activate LED.");
  }
  else if (reverseAndTurn)
  {
    if (stopReverseTime > millis())
      motorSpeed(-50, -50);
    else if (stopTurnTime > millis())
    {
      if (sonarLeftDist < SONAR_MIN_DISTANCE)
        motorSpeed(50, -50);
      else
        motorSpeed(-50, 50);
    }
    else
      reverseAndTurn = false;
  }
  else if (bumperState)
  {
    printDebug("Bumper has touched, stop all motors and engaging reverse and turn");
    md.setSpeeds(0, 0);
    reverseAndTurn = true;
    stopReverseTime = millis() + REVERSE_TIME;
    stopTurnTime = stopReverseTime + TURN_TIME;
  }
  else if (criticalDistSonar)
  {
    printDebug("Obstacle in critical zone, engaging reverse and turn");
    reverseAndTurn = true;
    stopReverseTime = millis() + REVERSE_TIME;
    stopTurnTime = stopReverseTime + TURN_TIME;
  }
  // Obstacle near anywhere
  else if (minDistSonar)
  {
    if (sonarLeftDist < SONAR_MIN_DISTANCE)
    {
      if (sonarCenterDist < SONAR_MIN_DISTANCE)
        motorSpeed(50, 10); // Turn Right
      else
        motorSpeed(35, 10); // Turn slightly
    }
    else if (sonarRightDist < SONAR_MIN_DISTANCE)
    {
      if (sonarCenterDist < SONAR_MIN_DISTANCE)
        motorSpeed(10, 50); // Turn Left
      else
        motorSpeed(10, 35); // Turn slighlty
    }
    else // Only on the center
    {
      motorSpeed(50, 50);
    }
  }
  else
  {
    // printDebug("Straight Forward Captain!");
    motorSpeed(400, 400);
  }
}
