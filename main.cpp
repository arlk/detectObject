#include "mbed.h"
#include "Sabertooth.h"
#include <cstdlib>
#include <ros.h>
#include <std_msgs/Int32.h>

//#define X_CENTER 1280
//#define Y_CENTER 720

float kp = 0.2;
float ki = 0.001;
float kd = 0.01;
float leftAngle = 0.026;
float rightAngle = 0.745;
float xCenter = 1280/2;
int L = 640;

float camAngle;  // camera gimbal angle
float desAngle = 100;  // Center angle
float indx = 0.5;  // angle step
int dir = 1; // direction of travel

float prevError;  // for angle error derivative
float angleError;  // angle error
float ddtAngleError;  // derivative of angle error
float intAngleError = 0;  // integral of angle error
float w;  // motor speed
float camAngle_dot = 0;
float prev_cam_angle = 0;

Ticker loop;
AnalogIn pot(p20);
//DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);
Sabertooth saber(p9,129,9600);
//Pixy pixy(Pixy::SPI, D11, D12, D13);

void PID() {
    
    //uint16_t blocks;
    //blocks = pixy.getBlocks();
    if (L!=-1) {
        prevError = angleError;  // for derivative term
        //angleError =  (float)(pixy.blocks[0].x - X_CENTER);
        angleError = L - xCenter;
        ddtAngleError = angleError - prevError;
        /*if( (angleError < 2) && (angleError > -2) )  // dead zone
            {
                angleError = 0;
            }*/
        intAngleError=intAngleError+angleError;
        w = (kp*angleError + ki*intAngleError + kd*ddtAngleError)+ 64 ;
        if (w<10) {
            w=10;
        }
        if (w>117) {
            w=117;
        }
        
        prev_cam_angle = camAngle;
        camAngle = pot.read()*200;
        camAngle_dot = (camAngle-prev_cam_angle)/0.01;
    }
    else{
        w=64;
        }
    
    
  
    /*if (dir>=0) {
        desAngle=desAngle + indx;
    }
    else {
        desAngle=desAngle - indx;
    }
     if ((desAngle>180) || (desAngle<20)) {
        dir=(-1)*dir;
    }
        
    camAngle = pot.read()*200;
    angleError = desAngle- camAngle;
    */
    
        
    //prevAngle=camAngle;
        
    if (intAngleError > 1) {
        intAngleError = 1;
        }   
}


void cmd(const std_msgs::Int32 &cmd_msg) {
     L = cmd_msg.data;
}


ros::NodeHandle nh(USBTX,USBRX);
ros::Subscriber<std_msgs::Int32> sub("camera", cmd);

int main() {
    //wait(2);
    nh.initNode();
    nh.subscribe(sub);
    loop.attach(&PID, 0.01);
    while(1) {
        nh.spinOnce();
        pc.printf("%f\n",camAngle);
        //wait(0.5);
        saber.SetSpeedMotorA(w);  // Between 0 and 127
    }
}





