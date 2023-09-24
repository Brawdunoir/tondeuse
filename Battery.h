#ifndef BATTERY_H
#define BATTERY_H

class Battery
{
public:
  Battery(int pin, float minVoltage = 1, float maxVoltage = 5);
  float getLevel();
  void update();

private:
  int pin;
  float level;
  float minVoltage; // Min battery voltage
  float maxVoltage; // Max battery voltage

  float readVoltage();
};

#endif
