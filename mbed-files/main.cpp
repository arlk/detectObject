/**
    @author Arun Lakshmanan
    @date 11/14/15
**/

#include "robot.h"
#include "los.h"
#include <cstdlib>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#define NOT_DETECTED -1

float dt = 0.01;
int pix = 640;
float V=0;
float W=0;

Ticker loop;

void servoCmd(const std_msgs::Int32 &cmd_msg) {
     pix = cmd_msg.data;
}

void motorCmd(const geometry_msgs::Vector3 &cmd_msg) {
     V = (float) cmd_msg.x;
     W = (float) cmd_msg.y;  
}

ros::NodeHandle nh(USBTX,USBRX);

ros::Subscriber<std_msgs::Int32> cameraSub("camera_x", servoCmd);
ros::Subscriber<geometry_msgs::Vector3> PathFollowSub("cmd_vel_w", motorCmd);

geometry_msgs::Vector3 los;
ros::Publisher losPub("losAngle", &los);

//PID Params: kp, ki, kd, Integral Saturation, Command Saturation, Command Bias, Deadzone

//length, radius, pwm tx, addr, baud, left(tx, rx), right(tx, rx), encoder res, PID params, interrupt rate
Robot Jim(0.28, 0.03, p13, 129, 9600, p27, p28, p29, p30, 20000, 3.0, 0.0001, 0, 2000, 100, 0, 0.01, dt); 

//xCenter, FOV, interrupt rate, pinName, PID Params, pwm tx, addr, baud
LOS Logitech(640, 70.42, dt, p20, 0.05, 0.0001, 0.00001, 2000, 5, 64, 0, p9, 129, 9600);


void PIDLoop() {

    Jim.checkVelocityDesired(V, W);
    Jim.checkEncoder();
    Jim.Update();

    Logitech.Update(pix);

   }

void losUpdate() { 
    los.x = Logitech.getAngle();
    los.y = Logitech.getAngleDot();
    losPub.publish(&los);
}


int main() {
    nh.initNode();
    nh.advertise(losPub);
    nh.subscribe(cameraSub);
    nh.subscribe(PathFollowSub);

    loop.attach(&PIDLoop, dt);
    
    while(1) {

        losUpdate();   
        nh.spinOnce();
        wait(0.001);
        Logitech.CommandServo();
        Jim.CommandVel();
    }
}





