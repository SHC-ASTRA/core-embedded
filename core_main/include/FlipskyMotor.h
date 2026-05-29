#pragma once

#ifdef FLIPSKY

#    include <Arduino.h>
#    include <VescUart.h>

extern HardwareSerial flipskySerial;
extern VescUart flipskyUart;

class FlipskyMotor {
    uint8_t revId;
    uint8_t vescId;
    bool inverted;
    int gearBox;

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

    FlipskyMotor(uint8_t setRevId, uint8_t setVescId, bool setInverted, int setGearBox = 1);

    inline int getID() const {
        return revId;
    }

    inline int getGearBox() const {
        return gearBox;
    }

    void sendSpeed(float val);
    inline void sendDuty(float /*val*/) {
    }
    void stop();

    inline void setBrake(bool) {
    }
    inline void identify() {
    }
    inline void accelerate() {
    }
    inline void setSlowStatusPeriods() {
    }

    void pollStatus();
};

#endif
