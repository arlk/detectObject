/**
    @author Arun Lakshmanan
    @date 11/14/15
**/

#include "los.h"

LOS::LOS(float center, float fieldOfView, float deltaT, PinName pin, float kp, float ki, float kd, float ISat, float CSat, float CBias, float DZone, PinName sbtx, int addr, int baud) : pot(pin), Servo(kp, ki, kd, ISat, CSat, CBias, DZone), Sb(sbtx, addr, baud) {
            xCenter = center;
            FOV = fieldOfView;
            kpixel = tan(FOV/2*M_PI/180)/xCenter;
            dt = deltaT;
            angle = 0;
            prevAngle = angle;
            angleDot = 0;
            servoCmd = 0;
	    Sb.InitializeCom();
        }

void LOS::addCamAngle(int pixel) {
    angle = servoAngle + (atan(kpixel*(pixel - xCenter)))*180/M_PI;
}        

void LOS::potAngle() {
    servoAngle = pot.read()*216.517 - 106.897;
}

float LOS::getAngle() {
    return angle;
} 

void LOS::calcAngleDot() {
    angleDot = (angle - prevAngle)/dt;
    prevAngle = angle;
}
 
void LOS::Update(int pixel) {
    potAngle();
    if (pixel==NOT_DETECTED) {
        servoCmd = Servo.getBiasValue();
    }
    else {
        addCamAngle(pixel);
        Servo.PIDUpdate(pixel, xCenter);
        servoCmd = Servo.getCmdValue();
        calcAngleDot();
    }

}

float LOS::getAngleDot() {
    return angleDot;
}

void LOS::CommandServo() {
    if ((servoAngle > 90) || (servoAngle < -90)) {
        Sb.SetSpeedMotorA(Servo.getBiasValue());
    }
    else {
        Sb.SetSpeedMotorA(servoCmd); 
    }
}

