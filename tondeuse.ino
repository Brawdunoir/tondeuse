#include <DualVNH5019MotorShield.h>

// Constants --------
const float MAX_SPEED = 400;
const bool DEBUG = true;
// ------------------

// Pins -------------
// JSN-SR04 Central
const int SONAR_CENTER_TRIG_PIN = 50;
const int SONAR_CENTER_ECHO_PIN = 48;

// HC-SR04 Left
const int SONAR_LEFT_TRIG_PIN = 30;
const int SONAR_LEFT_ECHO_PIN = 32;

// HC-SR04 Right
const int SONAR_RIGHT_TRIG_PIN = 26;
const int SONAR_RIGHT_ECHO_PIN = 28;

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
int etatInter1 = 1;  // bumper
// création de l´objet pour contrôler le calcul de distance
long lecture_echo1, lecture_echo2, lecture_echo3;
long cm1, cm2, cm3;

// Functions --------

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
