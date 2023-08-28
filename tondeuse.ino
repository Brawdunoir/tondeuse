#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield md;  //Use default pins

// broches utilisaées par le contrôleur moteur DualVNH5019 par défaut :
// Digital 2   M1INA   Motor 1 direction input A
// Digital 4   M1INB   Motor 1 direction input B
// Digital 6   M1EN/DIAG   Motor 1 enable input/fault output
// Digital 7   M2INA   Motor 2 direction input A
// Digital 8   M2INB   Motor 2 direction input B
// Digital 9   M1PWM   Motor 1 speed input
// Digital 10  M2PWM   Motor 2 speed input
// Digital 12  M2EN/DIAG   Motor 2 enable input/fault output
// Analog 0    M1CS  Motor 1 current sense output
// Analog 1    M2CS  Motor 2 current sense output


float VitesseMax = 400;
float Vitesse = 0;
float PasAccel = 10;
float VitesseMin = 0;

float Temps = 0;
float PasTemps = 1;
float TempsMax = 5000;


// broches utilisées par le capteur JSN-SR04 central
int trig1 = 50;
int echo1 = 48;

// broches utilisées par le capteur HC-SR04 face gauche
int trig2 = 30;
int echo2 = 32;

// broches utilisées par le capteur HC-SR04 face droite
int trig3 = 26;
int echo3 = 28;

// broches inter colision 1 et 2
const int inter1 = 34;

// variable utilisée pour stocker l'état des interrupteurs
int etatInter1 = 1;

// création de l´objet pour contrôler le calcul de distance
long lecture_echo1, lecture_echo2, lecture_echo3;
long cm1, cm2, cm3;

// déclaration variable relai de la lame de tonte
const int relai40 = 40;
int getBATTERIE = (INPUT);
///////////////////////////////////////////////////////capteur de fil
int rsensor = 2;  // Left Sensor on Analog Pin 2
int lsensor = 1;  // Right Sensor on Analog Pin 1
int msensor = 0;  // Middle Sensor on Analog Pin 0

int E1 = 6;  //M1 Speed Control
int E2 = 5;  //M2 Speed Control
int M1 = 8;  //M1 Direction Control
int M2 = 7;  //M2 Direction Control
int val = analogRead,(x);

int ledpinCapteur = 13;
const int whitelevl = 600;  // reading level is white if <600
const int blacklevl = 850;  // reading level is black if >850
////////////////////////////////////////////////////////////Batterie
const int BATTERIEPIN = A13;
const float TensionMin = 11.2;
const float TensionMax = 14.4;
int ledpinBatterie = 3;


void setup() {
  Serial.begin(9600);

  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  pinMode(trig1, OUTPUT);
  digitalWrite(trig1, LOW);
  pinMode(echo1, INPUT);

  pinMode(trig2, OUTPUT);
  digitalWrite(trig2, LOW);
  pinMode(echo2, INPUT);

  pinMode(trig3, OUTPUT);
  digitalWrite(trig3, LOW);
  pinMode(echo3, INPUT);

  pinMode(inter1, INPUT);
  digitalWrite(inter1, INPUT_PULLUP);

  // Initialize la sortie du relai
  pinMode(relai40, OUTPUT);
  // relai par défaut éteint
  digitalWrite(relai40, HIGH);

  // démarrage du programme tonte autonome
  delay(5000);

  // déclenchement du relai moteur tonte progressif
  digitalWrite(relai40, LOW);
  delay(100);
  digitalWrite(relai40, HIGH);
  delay(1000);
  digitalWrite(relai40, LOW);
  delay(100);
  digitalWrite(relai40, HIGH);
  delay(1000);
  digitalWrite(relai40, LOW);

}
void calcul_distance()
  {
    digitalWrite(trig1, LOW);
    delayMicroseconds(2);
    digitalWrite(trig1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig1, LOW);
    lecture_echo1 = pulseIn(echo1, HIGH);
    cm1 = lecture_echo1 / 58;
    Serial.print("distance mesurée au centre : ");
    Serial.println(lecture_echo1 / 58);

    digitalWrite(trig2, LOW);
    delayMicroseconds(2);
    digitalWrite(trig2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig2, LOW);
    lecture_echo2 = pulseIn(echo2, HIGH);
    cm2 = lecture_echo2 / 58;
    Serial.print("distance mesurée à gauche : ");
    Serial.println(lecture_echo2 / 58);

    digitalWrite(trig3, LOW);
    delayMicroseconds(2);
    digitalWrite(trig3, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig3, LOW);
    lecture_echo3 = pulseIn(echo3, HIGH);
    cm3 = lecture_echo3 / 58;
    Serial.print("distance mesurée à droite : ");
    Serial.println(lecture_echo3 / 58);
  }
  void calcul_distance_frontale()
  {
    digitalWrite(trig1, LOW);
    delayMicroseconds(2);
    digitalWrite(trig1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig1, LOW);
    lecture_echo1 = pulseIn(echo1, HIGH);
    cm1 = lecture_echo1 / 58;
    Serial.print("distance mesurée au centre : ");
    Serial.println(lecture_echo1 / 58);
  }
  void recul_et_avance_vers_la_plus_longue_distance()  // la tondeuse s´arrête et repart sur la distance la plus longue
  {
    md.setM1Speed(0);
    md.setM2Speed(0);
    delay(200);
    md.setM1Speed(-50);
    md.setM2Speed(-50);
    delay(300);
    md.setM1Speed(-100);
    md.setM2Speed(-100);
    delay(300);
    md.setM1Speed(-200);
    md.setM2Speed(-200);
    delay(300);
    md.setM1Speed(-200);
    md.setM2Speed(-200);
    delay(1500);
    md.setM1Speed(0);
    md.setM2Speed(0);
    delay(200);
  }
  void calcul_direction()
  {
    delay(100);
    if (cm3 < cm2);  // la tondeuse tourne à gauche
    {
      md.setM1Speed(50);
      md.setM2Speed(-50);
      delay(250);
      md.setM1Speed(100);
      md.setM2Speed(-100);
      delay(250);
      md.setM1Speed(150);
      md.setM2Speed(-150);
      delay(250);
      md.setM1Speed(200);
      md.setM2Speed(-200);
      delay(700);
      md.setM1Speed(0);
      md.setM2Speed(0);
      delay(400);
      Vitesse = 0;
    }
    if (cm3 > cm2);// La tondeuse tourne à droite
    {
      md.setM1Speed(-50);
      md.setM2Speed(50);
      delay(250);
      md.setM1Speed(-100);
      md.setM2Speed(100);
      delay(250);
      md.setM1Speed(-150);
      md.setM2Speed(150);
      delay(250);
      md.setM1Speed(-200);
      md.setM2Speed(200);
      delay(500);
      md.setM1Speed(0);
      md.setM2Speed(0);
      delay(400);
      Vitesse = 0;
    }
    if (cm3 == cm2) {
      // La tondeuse fait demi-tour
      md.setM1Speed(50);
      md.setM2Speed(-50);
      delay(250);
      md.setM1Speed(100);
      md.setM2Speed(-100);
      delay(250);
      md.setM1Speed(150);
      md.setM2Speed(-150);
      delay(250);
      md.setM1Speed(200);
      md.setM2Speed(-200);
      delay(1000);
      md.setM1Speed(200);
      md.setM2Speed(-200);
      delay(1000);
      Vitesse = 0;
    }
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


  void calcul_direction_frontale()
  {
    etatInter1 = digitalRead(inter1);
    if (etatInter1 == 0 && cm1 >= 20)  //si le capteur frontal ne détecte pas d´obstacle à 20 cm      cm1 >= 20 &&
      if (Vitesse <= VitesseMax) {
        Vitesse = Vitesse + PasAccel;
        md.setM1Speed(Vitesse);
        md.setM2Speed(Vitesse);
      } else {
        md.setM1Speed(VitesseMax);
        md.setM2Speed(VitesseMax);
      }
    else {
      // on arrête la tondeuse
      recul_et_avance_vers_la_plus_longue_distance();
    }
  }


  void getBATTERIE();
  }
    float b = analogRead(BATTERIEPIN);

    int minValue = (1023 * TensionMin) / 5;
    int maxValue = (1023 * TensionMax) / 5;

    b = ((b - minValue) / (maxValue - minValue)) * 100;

    if (b > 100)
      b = 100;
    else if (b = 0)
      b = 0;
    if (b = 20) {
      goforward();
      digitalWrite(relai40, HIGH);
      digitalWrite(ledpinBatterie, HIGH);
      delay(50);
      digitalWrite(ledpinBatterie, LOW);
      delay(2000);
    }
    int valeur = b;
    return b;
  }

}

void loop() {
  // Case 1: Left sensor and right sensors are reading white and middle sensor is reading black. Drive forward!

  if (readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) {
    goforward();
  }

  // Case 2 : Left sensor and middle sensor are reading white and right sensor is reading black. Turn right!

  else if (readQD(lsensor) < whitelevl && readQD(msensor) < whitelevl && readQD(rsensor) > blacklevl) {
    while (true) {
      turnright();
      if ((readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) || (readQD(lsensor) > blacklevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl)) { break; }  // Break if Left sensor and right sensor are reading white and middle sensor is reading black
    }
  }

  // Case 3 : Left sensor is reading white, middle sensor and right sensor are reading black. Turn right!

  else if (readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) > blacklevl) {
    while (true) {
      turnright();
      if ((readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) || (readQD(lsensor) > blacklevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl))
      { break; }  // Break if Left sensor and right sensor are reading white and middle sensor is reading black
    }
  }

  // Case 4 :  Left sensor is reading black, middle sensor and right sensor are reading white. Turn left!
  else if (readQD(lsensor) > blacklevl && readQD(msensor) < whitelevl && readQD(rsensor) < whitelevl) {
    while (true) {
      turnleft();
      if ((readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) || (readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) > blacklevl))
      { break; }  // Break if Left sensor and right sensor are reading white and middle sensor is reading black
    }
  }

  // Case 5 : Left sensor and middle sensor are reading black and right sensor is reading white. Turn left!
  else if (readQD(lsensor) > blacklevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) {
    while (true) {
      turnleft();
      if ((readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) < whitelevl) || (readQD(lsensor) < whitelevl && readQD(msensor) > blacklevl && readQD(rsensor) > blacklevl))
      { break; }  // Break if Left sensor and right sensor are reading white and middle sensor is reading black
    }
  }
  else {
    goforward();  // If there is no line, the rover will go forward
  }
  //////////////////// Read Sensor Routine //////////
  int readQD(int x);
  {
    int val = analogRead(x);
    return val;
  }
}

/*Serial.flush();  // vide le cache

  if (Temps <= TempsMax)
    {
      Temps = Temps + PasTemps;
      Serial.println("");
      Serial.println("Temps de fonctionnement : ");
      Serial.print(Temps);
    }
  else
    {
      digitalWrite(relai40, HIGH);
      md.setM1Speed(0);
      md.setM2Speed(0);
      delay(100000000);
    }*/
