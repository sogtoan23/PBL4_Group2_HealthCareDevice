#ifndef FALL_MEMORY_H
#define FALL_MEMORY_H

#include <Arduino.h>

#define FALL_MEM_MAX 100
#define FALL_SIM_THRESHOLD 0.8f

struct FallPattern {
  float ax, ay, az;
  float gx, gy, gz;
};

class FallMemory {
public:
  void begin();

  bool hasData();
  uint16_t count();

  bool isFalseFall(const FallPattern& cur);
  void addFalseFall(const FallPattern& cur);

  void clear();
  void debugPrint();

private:
  float similarity(const FallPattern& a, const FallPattern& b);

  FallPattern patterns[FALL_MEM_MAX];
  uint16_t size = 0;
};

#endif
