#ifdef FLIPSKY

#    include "FlipskyMotor.h"

#    include "CoreMainMCU.h"

VescUart flipskyUart;

FlipskyMotor::FlipskyMotor(uint8_t setCanId, bool setInverted) : canId(setCanId), inverted(setInverted) {
}

void FlipskyMotor::sendSpeed(float val) {
    float rpm = inverted ? -val : val;
    status1.sensorVelocity = rpm;
    flipskyUart.setRPM(rpm, canId);
}

void FlipskyMotor::stop() {
    status1.sensorVelocity = 0;
    flipskyUart.setRPM(0, canId);
}

void FlipskyMotor::pollStatus() {
    if (!flipskyUart.getVescValues(canId))
        return;

    unsigned long now = millis();
    status1.timestamp = now;
    status1.motorTemperature = flipskyUart.data.tempMotor;
    status1.busVoltage = flipskyUart.data.inpVoltage;
    status1.outputCurrent = flipskyUart.data.avgMotorCurrent;
    status1.sensorVelocity = flipskyUart.data.rpm;

    status2.timestamp = now;
    status2.sensorPosition = static_cast<float>(flipskyUart.data.tachometer);
}

#endif
