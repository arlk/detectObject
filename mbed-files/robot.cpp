/**
    @author Arun Lakshmanan
    @date 11/14/15
**/

#include "robot.h"

Robot::Robot(float l, float r, PinName sbtx, int addr, int baud, PinName Ltx, PinName Lrx, PinName Rtx, PinName Rrx, int encdrRes, float kp, float ki, float kd, float Isat, float CSat, float CBias, float DeadZone, float deltaT) : Sb(sbtx, addr, baud), WheelL(kp, ki, kd, Isat, CSat, CBias, DeadZone) , WheelR(kp, ki, kd, Isat, CSat, CBias, DeadZone), EncL(Ltx, Lrx, NC, encdrRes), EncR(Rtx, Rrx, NC, encdrRes) {
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
    wDesiredR = ((2*V-W*L)/(4*R*M_PI));
    wDesiredL = ((2*V-W*L)/(4*R*M_PI));
}

void Robot::checkEncoder() {
    int angL, angR;
    angL = EncL.getPulses();
    angR = EncR.getPulses();
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
    Sb.SetSpeedMotorA((int)WheelR.getCmdValue());
    Sb.SetSpeedMotorB((int)WheelL.getCmdValue());
}
