#ifdef FLIPSKY

#    include "FlipskyMotor.h"

#    include "CoreMainMCU.h"

VescUart flipskyUart;

FlipskyMotor::FlipskyMotor(uint8_t setRevId, uint8_t setVescId, bool setInverted, int setGearBox)
    : revId(setRevId), vescId(setVescId), inverted(setInverted), gearBox(setGearBox <= 0 ? 1 : setGearBox) {
}

void FlipskyMotor::dispatch(CmdMode mode, float value) {
    if (lastCmdMode == mode && value == lastSentValue)
        return;
    lastCmdMode = mode;
    lastSentValue = value;
    lastSendTime = millis();
    transmit();
}

void FlipskyMotor::transmit() {
    switch (lastCmdMode) {
        case CmdMode::RPM:
            flipskyUart.setRPM(lastSentValue, vescId);
            break;
        case CmdMode::DUTY:
            flipskyUart.setDuty(lastSentValue, vescId);
            break;
        case CmdMode::CURRENT:
            flipskyUart.setCurrent(lastSentValue, vescId);
            break;
        case CmdMode::BRAKE:
            flipskyUart.setBrakeCurrent(lastSentValue, vescId);
            break;
        case CmdMode::NONE:
            break;
    }
}

void FlipskyMotor::sendSpeed(float val) {
    // val is at the output shaft; VESC setRPM expects motor-side RPM, so scale by gearbox.
    float rpm = inverted ? -val : val;
    status1.sensorVelocity = rpm;
    targetDuty = 0.0f;
    currentDuty = 0.0f;
    // Zero RPM via the speed PID would actively brake; instead coast by zeroing motor current.
    if (rpm == 0.0f) {
        dispatch(CmdMode::CURRENT, 0.0f);
    } else {
        dispatch(CmdMode::RPM, rpm * gearBox);
    }
}

void FlipskyMotor::sendDuty(float val) {
    // Sets the target only & accelerate() ramps currentDuty toward it
    targetDuty = inverted ? -val : val;
    // instantly coast
    if (targetDuty == 0.0f) {
        currentDuty = 0.0f;
        dispatch(CmdMode::CURRENT, 0.0f);
    }
}

void FlipskyMotor::stop() {
    status1.sensorVelocity = 0;
    targetDuty = 0.0f;
    currentDuty = 0.0f;
    dispatch(CmdMode::CURRENT, 0.0f);
}

void FlipskyMotor::setBrake(bool brake) {
    targetDuty = 0.0f;
    currentDuty = 0.0f;
    dispatch(brake ? CmdMode::BRAKE : CmdMode::CURRENT, brake ? BRAKE_CURRENT_A : 0.0f);
}

void FlipskyMotor::accelerate() {
    // Ramp duty cycle &then also use heartbeat
    if (targetDuty != currentDuty) {
        float diff = targetDuty - currentDuty;
        if (fabs(diff) <= DUTY_ACCEL) {
            currentDuty = targetDuty;
        } else {
            currentDuty += (diff > 0 ? DUTY_ACCEL : -DUTY_ACCEL);
        }
        dispatch(CmdMode::DUTY, currentDuty);
        return;
    }

    // Heartbeat
    if (lastCmdMode == CmdMode::NONE)
        return;
    if (millis() - lastSendTime < RESEND_INTERVAL_MS)
        return;
    lastSendTime = millis();
    transmit();
}

void FlipskyMotor::pollStatus() {
    if (!flipskyUart.getVescValues(vescId))
        return;

    unsigned long now = millis();
    status1.timestamp = now;
    status1.motorTemperature = flipskyUart.data.tempMotor;
    status1.busVoltage = flipskyUart.data.inpVoltage;
    status1.outputCurrent = flipskyUart.data.avgMotorCurrent;
    status1.sensorVelocity = static_cast<float>(flipskyUart.data.rpm) / gearBox;

    status2.timestamp = now;
    status2.sensorPosition = static_cast<float>(flipskyUart.data.tachometer) / gearBox;
}

#endif
