#pragma once

#ifdef FLIPSKY

#include <Arduino.h>
#include <VescUart.h>

extern HardwareSerial flipskySerial;
extern VescUart flipskyUart;

class FlipskyMotor {
  uint8_t canId;
  bool inverted;

public:
  struct Status1 {
    unsigned long timestamp = 0;
    float motorTemperature = 0;
    float busVoltage = 0;
    float outputCurrent = 0;
    float sensorVelocity = 0;
  } status1;

  struct Status2 {
    unsigned long timestamp = 0;
    float sensorPosition = 0;
  } status2;

  FlipskyMotor(uint8_t setCanId, bool setInverted);

  inline int getID() const { return canId; }

  void sendSpeed(float val);
  inline void sendDuty(float /*val*/) {}
  void stop();

  inline void setBrake(bool) {}
  inline void identify() {}
  inline void accelerate() {}
  inline void setSlowStatusPeriods() {}

  void pollStatus();
};

#endif
