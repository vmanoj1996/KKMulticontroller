#ifndef PID_H
#define PID_H

#include "config.h"

enum PID_ArrayIndex { PID_ROLL = 0, PID_PITCH, PID_YAW };

extern struct PID_PidObject {
  float measured;

  float setPoint;
  float kP;
  float kI;
  float kD;
  float epsilon;

  uint64_t lastTime;
  float lastError;
  float integral;

  float pidOutput;
} PID_Array[3];

void PID_Calculate(struct PID_pidObject *pidObject);
void PID_Calculate(void);

#endif
