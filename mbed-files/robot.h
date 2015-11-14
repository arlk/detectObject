/**
    @author Arun Lakshmanan
    @date 11/14/15
**/

#ifndef ROBOT_H
#define ROBOT_H

#include "mbed.h"
#include "customPID.h"
#include "Sabertooth.h"
#include "math.h"
#include "QEI.h"

class Robot {
    public:
        Robot(float l, float r, PinName sbtx, int addr, int baud, PinName Ltx, PinName Lrx, PinName Rtx, PinName Rrx, int encdrRes, float kp, float ki, float kd, float Isat, float CSat, float CBias, float DeadZone, float deltaT);
        
        void checkVelocityDesired(float v, float w);
        void checkEncoder();
        void Update();
        void CommandVel();

    private:
        Sabertooth Sb;
        PID WheelL, WheelR;
        QEI EncL, EncR;
        float V,W,L,R;
        float wDesiredR, wDesiredL;
        float wR, wL;
        float prevAngL, prevAngR;
        float encoderRes;
        float dt;


};

#endif
