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
    enum class CmdMode {
        NONE,
        RPM,
        DUTY,
        CURRENT,
        BRAKE
    };
    CmdMode lastCmdMode = CmdMode::NONE;
    float lastSentValue = 0.0f;
    unsigned long lastSendTime = 0;
    static constexpr unsigned long RESEND_INTERVAL_MS = 500;
    static constexpr float BRAKE_CURRENT_A = 5.0f;

    // Duty cycle ramping
    float targetDuty = 0.0f;
    float currentDuty = 0.0f;
    // 0.05 reaches full duty in about a second
    static constexpr float DUTY_ACCEL = 0.05f;

    void dispatch(CmdMode mode, float value);
    void transmit();

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
    void sendDuty(float val);
    void stop();

    void setBrake(bool brake);
    inline void identify() {
    }
    void accelerate();
    inline void setSlowStatusPeriods() {
    }

    void pollStatus();
};

#endif
