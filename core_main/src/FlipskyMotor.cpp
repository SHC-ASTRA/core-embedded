#ifdef FLIPSKY

#    include "FlipskyMotor.h"

#    include "CoreMainMCU.h"

VescUart flipskyUart;

FlipskyMotor::FlipskyMotor(uint8_t setRevId, uint8_t setVescId, bool setInverted, int setGearBox)
    : revId(setRevId), vescId(setVescId), inverted(setInverted), gearBox(setGearBox <= 0 ? 1 : setGearBox) {
}

void FlipskyMotor::sendSpeed(float val) {
    // val is at the output shaft; VESC setRPM expects motor-side RPM, so scale by gearbox.
    float rpm = inverted ? -val : val;
    status1.sensorVelocity = rpm;
    flipskyUart.setRPM(rpm * gearBox, vescId);
}

void FlipskyMotor::stop() {
    status1.sensorVelocity = 0;
    flipskyUart.setRPM(0, vescId);
}

void FlipskyMotor::pollStatus() {
    if (!flipskyUart.getVescValues(vescId))
        return;

    unsigned long now = millis();
    status1.timestamp = now;
    status1.motorTemperature = flipskyUart.data.tempMotor;
    status1.busVoltage = flipskyUart.data.inpVoltage;
    status1.outputCurrent = flipskyUart.data.avgMotorCurrent;
    // VESC reports motor-side RPM and tachometer; rescale to output shaft for parity with REV.
    status1.sensorVelocity = static_cast<float>(flipskyUart.data.rpm) / gearBox;

    status2.timestamp = now;
    status2.sensorPosition = static_cast<float>(flipskyUart.data.tachometer) / gearBox;
}

#endif
