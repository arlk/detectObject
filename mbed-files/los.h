/**
    @author Arun Lakshmanan
    @date 11/14/15
**/

#ifndef LOS_H
#define LOS_H

#include "mbed.h"
#include "customPID.h"
#include "Sabertooth.h"

#define NOT_DETECTED 9999 

class LOS {
    public:
        float angle, angleDot;

        LOS(float center, float fieldOfView, float deltaT, PinName pin, float kp, float ki, float kd, float ISat, float CSat, float CBias, float DZone, PinName sbtx, int addr, int baud);

        void addCamAngle(int pixel);
        void potAngle();
        float getAngle();
        void calcAngleDot();
        void Update(int pixel);
        float getAngleDot();
        void CommandServo();

    private:
        AnalogIn pot;
        PID Servo;
        Sabertooth Sb;
        float servoCmd, servoAngle;
        float dt;
        float prevAngle;
        float xCenter;
        float FOV;
        float kpixel;

};

#endif
