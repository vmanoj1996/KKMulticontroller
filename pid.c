#include "pid.h"

#include "time.h"

struct PID_pidObject PID_Array[3];

void PID_calculate(struct PID_pidObject *pidObject)
{
  uint64_t time = millis();
  uint64_t dt = time - pidObject->lastTime;

  float error = pidObejct->measured - pidObject->setPoint;

  // Calculate product
  float output = pidObject->kP * error;

  // Calculate integral
  float integral = pidObject->integral;
  if(abs(error) > pidObject->epsilon) {
    integral += error * dt;
  }
  output += pidObject->kI * integral;

  // Calculate differential
  output += pidObject->kD * ((pidObject->lastError - error) / dt);
  
  pidObject->integral = integral;
  pidObject->lastTime = time;
  pidObject->lastError = error;
}
