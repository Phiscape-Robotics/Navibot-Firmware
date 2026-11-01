#ifndef NAVIBOT_MOTOR_STATE_H
#define NAVIBOT_MOTOR_STATE_H

#include <Arduino.h>

enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_LEFT_ENCODER_STALL,
  FAULT_RIGHT_ENCODER_STALL
};

struct MotorState {
  uint8_t enablePin;
  uint8_t in1Pin;
  uint8_t in2Pin;
  float countsPerRev;
  float targetRPM;
  float measuredRPM;
  float integral;
  float previousError;
  int8_t appliedDirection;
  int8_t reportDirection;
  unsigned long lastPulseMs;
  int pwm;
};

#endif
