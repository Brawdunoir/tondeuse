#ifndef LINESENSOR_H
#define LINESENSOR_H

class LineSensor
{
public:
  LineSensor(int pin, float whiteLevel, float blackLevel, bool debug);
  bool isAboveLine();
  void update();
  float getLevel();

private:
  int pin;
  float whiteLevel;   // Level below which there is no line
  float blackLevel;   // Level above which there is a line
  float level;        // Actual level read
  bool aboveLine;   // Is the sensor above a line
  bool debug;
};

#endif
