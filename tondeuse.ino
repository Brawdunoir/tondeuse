#include <DualVNH5019MotorShield.h>
#include "Ultrasonic.h"

// Constants ------------------------------------
const float MAX_SPEED = 400;
const float SONAR_TIMEOUT = 20000UL; // 20ms to get approx 3.4m of range
const bool DEBUG = true;
// ----------------------------------------------

// Pins -----------------------------------------
// JSN-SR04 Central
const int SONAR_CENTER_TRIG_PIN = 50;
const int SONAR_CENTER_ECHO_PIN = 48;
Ultrasonic sonarCenter(SONAR_CENTER_TRIG_PIN, SONAR_CENTER_ECHO_PIN, SONAR_TIMEOUT);

// HC-SR04 Left
const int SONAR_LEFT_TRIG_PIN = 30;
const int SONAR_LEFT_ECHO_PIN = 32;
Ultrasonic sonarLeft(SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN, SONAR_TIMEOUT);

// HC-SR04 Right
const int SONAR_RIGHT_TRIG_PIN = 26;
const int SONAR_RIGHT_ECHO_PIN = 28;
Ultrasonic sonarRight(SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN, SONAR_TIMEOUT);

// Bumper
const int BUMPER_PIN = 34;

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

// TODO: Donner des meilleurs noms et puis pas sûr qu'on en ait besoin avec la lib DualVNH…
int E1 = 6; // M1 Speed Control
int E2 = 5; // M2 Speed Control
int M1 = 8; // M1 Direction Control
int M2 = 7; // M2 Direction Control

///////////////////////////////////////////////////////capteur de fil
int rsensor = 2; // Left Sensor on Analog Pin 2
int lsensor = 1; // Right Sensor on Analog Pin 1
int msensor = 0; // Middle Sensor on Analog Pin 0

int val = analogRead, (x); // TODO Qu'est ce que c'est ?

// TODO Keske C ?
int ledpinCapteur = 13;
const int whitelevl = 600; // reading level is white if <600
const int blacklevl = 850; // reading level is black if >850
////////////////////////////////////////////////////////////Batterie
int ledpinBatterie = 3; // TODO Kesque Se ?
// ----------------------------------------------

// Variables ------------------------------------
// Motor speed
float speed = 0;
float PasAccel = 10; // TODO: Voir pour l'enlever avec meilleure méthode accélération (ardumower)

// Bumper
int bumperState = 0;

// Sonar
long sonarCenterDist, sonarLeftDist, sonarRightDist;

// Battery
float batteryLevel = 100;
// ----------------------------------------------

// Next Time ------------------------------------
unsigned int nextTimeSonar = 0;
unsigned int nextTimeBattery = 0;
unsigned int nextTimeBumper = 0;

// Functions --------
/*
checkSonar will update the current ultrasonic sensor to check
*/
void checkSonar()
{
  if (millis() >= nextTimeSonar)
  {
    static short senSonarTurn = 0;
    nextTimeSonar = millis() + 250;

    switch (senSonarTurn)
    {
    case 0:
      sonarCenterDist = sonarCenter.read();
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
  if (DEBUG)
  {
    nextTimeSonar = millis() + 2500;
  }
}

void checkBumper()
{
  if (millis() >= nextTimeBumper)
  {
    nextTimeBumper = millis() + 100;

    bumperState = digitalRead(BUMPER_PIN);

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

void setup()
{
  Serial.begin(9600);

  pinMode(SONAR_CENTER_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_CENTER_TRIG_PIN, LOW);
  pinMode(SONAR_CENTER_ECHO_PIN, INPUT);
  if (DEBUG)
    Serial.println("Initialized Sonar Center.");

  pinMode(SONAR_LEFT_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_LEFT_TRIG_PIN, LOW);
  pinMode(SONAR_LEFT_ECHO_PIN, INPUT);
  if (DEBUG)
    Serial.println("Initialized Sonar Left.");

  pinMode(SONAR_RIGHT_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_RIGHT_TRIG_PIN, LOW);
  pinMode(SONAR_RIGHT_ECHO_PIN, INPUT);
  if (DEBUG)
    Serial.println("Initialized Sonar Right.");

  pinMode(BUMPER_PIN, INPUT);
  digitalWrite(BUMPER_PIN, INPUT_PULLUP);
  if (DEBUG)
    Serial.println("Initialized Bumper.");

  md.init();
  if (DEBUG)
    Serial.println("Initialized motors.");

  pinMode(MOW_MOTOR_PIN, OUTPUT);
  digitalWrite(MOW_MOTOR_PIN, HIGH);
  if (DEBUG)
    Serial.println("Initialized mow motor.");

  // déclenchement du relai moteur tonte progressif
  if (DEBUG)
  {
    Serial.println("Skip launching mow motor in debug mode.")
  }
  else
  {
    // TODO: Pourquoi?
    digitalWrite(MOW_MOTOR_PIN, LOW);
    delay(100);
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    delay(1000);
    digitalWrite(MOW_MOTOR_PIN, LOW);
    delay(100);
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    delay(1000);
    digitalWrite(MOW_MOTOR_PIN, LOW);
  }
}

void loop()
{
  checkBattery();
  checkBumper();
  checkSonar();
  
}
