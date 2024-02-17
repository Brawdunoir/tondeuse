#include "LineSensor.h"
#include <Arduino.h>

LineSensor::LineSensor(int pin, float whiteLevel, float blackLevel)
    : pin(pin), level(0), isAboveLine(false), whiteLevel(whiteLevel), blackLevel(blackLevel)
{
  pinMode(pin, INPUT);
}

bool LineSensor::isAboveLine()
{
  return isAboveLine;
}

void LineSensor::update()
{
  level = analogRead(pin);
  if (level < whiteLevel)
    isAboveLine = false;
  else if (level > blackLevel)
    isAboveLine = true;
}

float LineSensor::getLevel()
{
  return level
}