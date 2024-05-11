#ifndef LINESENSOR_H
#define LINESENSOR_H

class LineSensor
{
public:
  LineSensor(int pin, float threshold, bool debug);
  LineSensor();
  bool isAboveLine();
  void update();
  float getLevel();

private:
  int pin;
  float threshold; // Level below which there is no line
  float level;     // Actual level read
  bool aboveLine;  // Is the sensor above a line
  bool debug;
};

#endif
