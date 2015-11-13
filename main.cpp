#include "mbed.h"
#include "Sabertooth.h"
#include "mbed.h"
#include <cstdlib>
#include <ros.h>
#include <std_msgs/Int32.h>
//#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
//#define X_CENTER 1280
//#define Y_CENTER 720

float kp = 0.05;
float ki = 0.0001;
float kd = 0.00001;
float leftAngle = 0.026;
float rightAngle = 0.745;
int xCenter = 640;
int L = 640;

float camAngle;  // camera gimbal angle
float desAngle = 100;  // Center angle
float indx = 0.5;  // angle step
int dir = 1; // direction of travel

float prevError;  // for angle error derivative
float angleError=0;  // angle error
float ddtAngleError;  // derivative of angle error
float intAngleError = 0;  // integral of angle error
float w=64;  // motor speed
float camAngle_dot = 0;
float prev_cam_angle = 0;
float sat=2000;
float FOV = 70.42;
float kpixel = tan(FOV/2*M_PI/180)/xCenter;

Ticker loop;
AnalogIn pot(p20);
//DigitalOut myled(LED1);
//Serial pc(USBTX, USBRX);
Sabertooth saber(p9,129,9600);
//Pixy pixy(Pixy::SPI, D11, D12, D13);
void cmd(const std_msgs::Int32 &cmd_msg) {
     L = cmd_msg.data;
}


ros::NodeHandle nh(USBTX,USBRX);
ros::Subscriber<std_msgs::Int32> sub("camera_x", cmd);
//geometry_msgs::Vector3 err;
//ros::Publisher pub("error", &err);
std_msgs::Float64 los;
ros::Publisher pub("losAngle", &los);


float findAngle(int pixel) {
    float ang = 0;
    ang = atan(kpixel*(pixel - xCenter));
    ang *= 180/M_PI;
    return ang;
}


void PID() {
    
    float angle = 0;
    float angle_dot=0;
    angle = pot.read()*202 - 101;

    //uint16_t blocks;
    //blocks = pixy.getBlocks();
    if (L!=-1) {
        angle += findAngle(L);
        
        
        if (fabs(angle)<90) {
            prevError = angleError;  // for derivative term
            //angleError =  (float)(pixy.blocks[0].x - X_CENTER);
            angleError = (float) (L - xCenter);
            ddtAngleError = angleError - prevError;
            /*if( (angleError < 2) && (angleError > -2) )  // dead zone
            {
                angleError = 0;
            }*/
            intAngleError=intAngleError+angleError;
            w = (kp*angleError + ki*intAngleError + kd*ddtAngleError)+ 64 ;
            if (w<59) {
                w=59;
            }
            if (w>69) {
                w=69;
            }

            if (intAngleError<-sat) {
                intAngleError=-sat;
            }
            if (intAngleError>sat) {
                intAngleError=sat;
            }
        }
        else {
            w = 64;
        }
        
        angle_dot = (angle-prev_cam_angle)/0.01;
        prev_cam_angle = angle;

        
    }
    else{
        w=64;
        }
    
   /* err.x = w;*/
    //err.y = angle;
    //err.z = angle_dot;
    /*pub.publish(&err);*/
    los.data = angle;
    pub.publish(&los);   
  
    /*if (dir>=0) {
        desAngle=desAngle + indx;
    }
    else {
        desAngle=desAngle - indx;
    }
     if ((desAngle>180) || (desAngle<20)) {
        dir=(-1)*dir;
    }*/
        

    //angleError = desAngle- camAngle;
    
    
        
    //prevAngle=camAngle;
        
}



int main() {
    //wait(2);
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);
    loop.attach(&PID, 0.01);
    while(1) {
        nh.spinOnce();
        //pc.printf("%f\n",camAngle);
        wait(0.001);
        saber.SetSpeedMotorA(w);  // Between 0 and 127
    }
}





