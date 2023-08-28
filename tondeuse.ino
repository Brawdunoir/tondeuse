#include <DualVNH5019MotorShield.h>
#include "Ultrasonic.h"

// Constants --------
const float MAX_SPEED = 400;
const float SONAR_TIMEOUT = 20000UL; // 20ms to get approx 3.4m of range
const bool DEBUG = true;
// ------------------

// Pins -------------
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
// TODO: Pas compris
// int getBATTERIE = (INPUT);

// TODO: Donner des meilleurs noms
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
// ------------------

// Variables
float speed = 0;
float PasAccel = 10; // TODO: Voir pour l'enlever avec meilleure méthode accélération (ardumower)
// Sonar
long sonarCenterDist, sonarLeftDist, sonarRightDist;

// Next Time ------------------------------------
unsigned int nextTimeSonar = 0;

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
}

/*
getDistance. Get the distance from a Sonar sensor
*/
void getBattery()
{
  float result = ((analogRead(BATTERY_PIN) - BATTERY_MIN_VALUE) / (BATTERY_MAX_VALUE - BATTERY_MIN_VALUE)) * 100;

  if (result > 100)
    result = 100;
  else if (result < 0)
    result = 0;

  return result;
}

void setup()
{
  Serial.begin(9600);

  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  pinMode(SONAR_CENTER_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_CENTER_TRIG_PIN, LOW);
  pinMode(SONAR_CENTER_ECHO_PIN, INPUT);

  pinMode(SONAR_LEFT_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_LEFT_TRIG_PIN, LOW);
  pinMode(SONAR_LEFT_ECHO_PIN, INPUT);

  pinMode(SONAR_RIGHT_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_RIGHT_TRIG_PIN, LOW);
  pinMode(SONAR_RIGHT_ECHO_PIN, INPUT);

  pinMode(BUMPER_PIN, INPUT);
  digitalWrite(BUMPER_PIN, INPUT_PULLUP);

  // Initialize la sortie du relai
  pinMode(MOW_MOTOR_PIN, OUTPUT);
  // relai par défaut éteint
  digitalWrite(MOW_MOTOR_PIN, HIGH);

  // démarrage du programme tonte autonome
  delay(5000);

  // déclenchement du relai moteur tonte progressif
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

///////////// Go Forward Routine ///////////////////
void goforward()
{
  analogWrite(E1, 210);
  digitalWrite(M1, LOW);
  analogWrite(E2, 210);
  digitalWrite(M2, LOW);
  digitalWrite(ledpinCapteur, HIGH);
  delay(50);
  digitalWrite(ledpinCapteur, LOW);
  delay(1000);
}
///////////// Turn Right Routine ///////////////////
void turnright()
{
  analogWrite(E1, 210);
  digitalWrite(M1, LOW);
  analogWrite(E2, 210);
  digitalWrite(M2, HIGH);
}
///////////// Turn Left Routine ///////////////////
void turnleft()
{
  analogWrite(E1, 210);
  digitalWrite(M1, HIGH);
  analogWrite(E2, 210);
  digitalWrite(M2, LOW);
}

void getBATTERIE()
{
  float b = analogRead(BATTERY_PIN);

  int minValue = (1023 * TensionMin) / 5;
  int maxValue = (1023 * TensionMax) / 5;

  b = ((b - minValue) / (maxValue - minValue)) * 100;

  if (b > 100)
    b = 100;
  else if (b = 0)
    b = 0;
  if (b = 20)
  {
    goforward();
    digitalWrite(MOW_MOTOR_PIN, HIGH);
    digitalWrite(ledpinBatterie, HIGH);
    delay(50);
    digitalWrite(ledpinBatterie, LOW);
    delay(2000);
  }
  int valeur = b;
  return b;
}

void loop()
{
  // Case 1: Left sensor and right sensors are reading white and middle sensor is reading black. Drive forward!

  if (readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl)
  {
    goforward();
  }

  // Case 2 : Left sensor and middle sensor are reading white and right sensor is reading black. Turn right!

  else if (readQD(lsensor) < whitelevl && readQD(msensor) < whitelevl && readQD(rsensor) > blacklevl)
  {
    while (true)
    {
      turnright();
      if ((readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) || (readQD(lsensor) > blacklevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl))
      {
        break;
      } // Break if Left sensor and right sensor are reading white and middle sensor is reading black
    }
  }

  // Case 3 : Left sensor is reading white, middle sensor and right sensor are reading black. Turn right!

  else if (readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) > blacklevl)
  {
    while (true)
    {
      turnright();
      if ((readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) || (readQD(lsensor) > blacklevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl))
      {
        break;
      } // Break if Left sensor and right sensor are reading white and middle sensor is reading black
    }
  }

  // Case 4 :  Left sensor is reading black, middle sensor and right sensor are reading white. Turn left!
  else if (readQD(lsensor) > blacklevl && readQD(msensor) < whitelevl && readQD(rsensor) < whitelevl)
  {
    while (true)
    {
      turnleft();
      if ((readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) || (readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) > blacklevl))
      {
        break;
      } // Break if Left sensor and right sensor are reading white and middle sensor is reading black
    }
  }

  // Case 5 : Left sensor and middle sensor are reading black and right sensor is reading white. Turn left!
  else if (readQD(lsensor) > blacklevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl)
  {
    while (true)
    {
      turnleft();
      if ((readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) || (readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) > blacklevl))
      {
        break;
      } // Break if Left sensor and right sensor are reading white and middle sensor is reading black
    }
  }
  else
  {
    goforward(); // If there is no line, the rover will go forward
  }
  //////////////////// Read Sensor Routine //////////
  int readQD(int x);
  {
    int val = analogRead(x);
    return val;
  }
}
