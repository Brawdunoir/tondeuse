#include "LineSensor.h"
#include <Arduino.h>

LineSensor::LineSensor(int pin, float whiteLevel, float blackLevel, bool debug)
    : pin(pin), level(0), aboveLine(false), whiteLevel(whiteLevel), blackLevel(blackLevel), debug(debug)
{
  pinMode(pin, INPUT);
}

bool LineSensor::isAboveLine()
{
  return aboveLine;
}

void LineSensor::update()
{
  level = analogRead(pin);
  if (debug)
    Serial.println(level);
  
  if (level < whiteLevel)
    aboveLine = false;
  else if (level > blackLevel)
    aboveLine = true;
}

float LineSensor::getLevel()
{
  return level;
}