#include "LineSensor.h"
#include <Arduino.h>

LineSensor::LineSensor(int pin, float threshold, bool debug)
    : pin(pin), level(0), aboveLine(false), threshold(threshold), debug(debug)
{
  pinMode(pin, INPUT);
}

LineSensor::LineSensor() {}

bool LineSensor::isAboveLine()
{
  return aboveLine;
}

void LineSensor::update()
{
  level = analogRead(pin);
  if (debug)
    Serial.println(level);

  if (level > threshold)
    aboveLine = true;
  else
    aboveLine = false;
}

float LineSensor::getLevel()
{
  return level;
}
