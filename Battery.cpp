#include "Battery.h"

Battery::Battery(int pin, float minV, float maxV)
    : pin(pin), level(100), minVoltage(minV), maxVoltage(maxV)
{
  pinMode(pin, INPUT);
}

int Battery::getLevel()
{
  return batteryLevel;
}

void Battery::update()
{
  float voltage = readVoltage();
  level = map(voltage, minVoltage, maxVoltage, 0, 100);
  level = constrain(level, 0, 100);
}

float Battery::readVoltage()
{
  float voltage = analogRead(pin);
  voltage *= 5.0;
  voltage /= 1023.0;
  return voltage;
}
