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

float dt = 0.01;
int pix = 640;

float V=0;
float W=0;

float Kp = 20;
float Ki = 0.1;
float Kd = 0;
float IntSat = 5;

Ticker loop;

void servoCmd(const std_msgs::Int32 &cmd_msg) {
     pix = cmd_msg.data;
}

void motorCmd(const geometry_msgs::Vector3 &cmd_msg) {
     V = (float) cmd_msg.x;
     W = (float) cmd_msg.y; 
}

void getGains(const geometry_msgs::Vector3 &cmd_msg) {
     Kp = (float) cmd_msg.x;
     Ki = (float) cmd_msg.y;
     IntSat = (float) cmd_msg.z;
}

//ros::NodeHandle nh(p28,p27);
ros::NodeHandle nh(USBTX,USBRX);

ros::Subscriber<std_msgs::Int32> cameraSub("camera_x", servoCmd);
ros::Subscriber<geometry_msgs::Vector3> PathFollowSub("cmd_vel_w", motorCmd);
ros::Subscriber<geometry_msgs::Vector3> RobotGains("gains", getGains);

geometry_msgs::Vector3 los;
ros::Publisher losPub("losAngle", &los);

//PID Params: kp, ki, kd, Integral Saturation, Command Saturation, Command Bias, Deadzone

//length, radius, pwm tx, addr, baud, encoder res, PID params, interrupt rate
QEI EncdrL(p29, p30, NC, 624);
QEI EncdrR(p23, p24, NC, 624);
Robot Jim(0.28, 0.03, p13, 129, 9600, 20000, 30.0, 0.003, 0, 5000, 120, 0, 0.01, dt); 

//xCenter, FOV, interrupt rate, pinName, PID Params, pwm tx, addr, baud
LOS Logitech(640, 70.42, dt, p20, 0.05, 0.0001, 0.00001, 2000, 5, 64, 0, p9, 129, 9600);


void PIDLoop() {
 
    nh.spinOnce();
    wait(0.05);

    Jim.checkVelocityDesired(V, W);
    
    int pulseL, pulseR;
    pulseL = EncdrL.getPulses();
    pulseR = EncdrR.getPulses();

    Jim.checkEncoder(pulseL, pulseR);
    Jim.setGains(Kp, Ki, IntSat);
    Jim.Update();

    Logitech.Update(pix);
    
    nh.spinOnce();
    wait(0.05);

   }

void losUpdate() { 
    los.x = Logitech.getAngle();
    los.y = Logitech.getAngleDot();
    los.z = Jim.getcmd();
    losPub.publish(&los);
}


int main() {
    
    nh.initNode();
    nh.advertise(losPub);
    nh.subscribe(cameraSub);
    nh.subscribe(PathFollowSub);
    //nh.subscribe(RobotGains);

    //loop.attach(&PIDLoop, dt);
    while(1) {
        PIDLoop();
        losUpdate();   
        Logitech.CommandServo();
        Jim.CommandVel();
    }
}





