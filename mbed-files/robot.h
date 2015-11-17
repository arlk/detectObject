/**
    @author Arun Lakshmanan
    @date 11/14/15
**/

#ifndef ROBOT_H
#define ROBOT_H

#include "mbed.h"
#include "customPID.h"
#include "Sabertooth.h"
#include "QEI.h"
#include "math.h"

class Robot {
    public:
        Robot(float l, float r, PinName sbtx, int addr, int baud, int encdrRes, float kp, float ki, float kd, float Isat, float CSat, float CBias, float DeadZone, float deltaT);
        
        void checkVelocityDesired(float v, float w);
        void checkEncoder(int angL, int angR);
        void Update();
        void CommandVel();
        void setGains(float kp, float ki, float intSat);
        float getcmd();

    private:
        Sabertooth Sb;
        PID WheelL, WheelR;
        float V,W,L,R;
        float wDesiredR, wDesiredL;
        float wR, wL;
        int prevAngL, prevAngR;
        float encoderRes;
        float dt;


};

#endif
