/**
    @author Arun Lakshmanan
    @date 11/14/15
**/

#include "robot.h"

Robot::Robot(float l, float r, PinName sbtx, int addr, int baud, int encdrRes, float kp, float ki, float kd, float Isat, float CSat, float CBias, float DeadZone, float deltaT) : Sb(sbtx, addr, baud), WheelL(kp, ki, kd, Isat, CSat, CBias, DeadZone) , WheelR(kp, ki, kd, Isat, CSat, CBias, DeadZone) {
    V = 0;
    W = 0;
    L = l;
    R = r;
    encoderRes = encdrRes;
    prevAngL = 0;
    prevAngR = 0;
    dt = deltaT;
    Sb.InitializeCom();
} 

void Robot::checkVelocityDesired(float v, float w) {
    wDesiredR = ((2*v-w*L)/(4*R*M_PI));
    wDesiredL = ((2*v+w*L)/(4*R*M_PI));
}

void Robot::checkEncoder(int angL, int angR) {
    wL = (float) (angL - prevAngL)/(encoderRes*dt);
    wR = (float) (angR - prevAngR)/(encoderRes*dt);
    prevAngL = angL;
    prevAngR = angR;
}

void Robot::Update() {
    WheelL.PIDUpdate(wL, wDesiredL);
    WheelR.PIDUpdate(wR, wDesiredR);
}

void Robot::CommandVel() {
    Sb.SetSpeedMotorA(WheelL.getCmdValue());
    Sb.SetSpeedMotorB(WheelR.getCmdValue());
}

void Robot::setGains(float kp, float ki, float intSat) {
    WheelL.setGains(kp, ki, intSat);
    WheelR.setGains(kp, ki, intSat);
}
float Robot::getcmd() {
    return WheelL.getCmdValue();
}
