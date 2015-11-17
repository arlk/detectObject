/**
    @author Arun Lakshmanan
    @date 11/14/15
**/

#include "customPID.h"

PID::PID(float Pgain, float Igain, float Dgain, float Isat, float Csat, float CmdBias, float DeadZone)
{
    kp = Pgain;
    ki = Igain;
    kd = Dgain;
    intSat = Isat;
    cmdSat = Csat;
    bias = CmdBias;
    deadzone = DeadZone;
    intError = 0;
    prevError = 0;
    cmd = CmdBias;
}

void PID::PIDUpdate(float currValue, float reference)
{
    float error;
    
    error = reference - currValue;
    intError += error;
    intError = saturate(intError, intSat);
    dError = error - prevError;

    if (limiting(error, deadzone)) {
        cmd = kp*error + ki*intError + kd*dError;
        cmd = saturate(cmd, cmdSat);
    }
    else {
        cmd = 0;
    }

    cmd += bias;
    prevError = error; 
}

float PID::saturate(float value, float boundary) {
    if (value < -boundary) {
        return -boundary;
    }
    else if (value > boundary) {
        return boundary;
    }
    else {
        return value;
    }
}

float PID::limiting(float value, float boundary) {
    if ((value < -boundary)||(value > boundary)) {
        return value;
    }
    else {
        return 0;
    }
}

float PID::getCmdValue() {
    return cmd;
}

float PID::getBiasValue() {
    return bias;
}

float PID::geterr() {
    return prevError;
}

void PID::setGains(float Pgain, float Igain, float Isat) {
    kp = Pgain;
    ki = Igain;
    intSat = Isat;
}
