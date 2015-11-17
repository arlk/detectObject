/**
    @author Arun Lakshmanan
    @date 11/14/15
**/

#ifndef CUSTOM_PID_H
#define CUSTOM_PID_H

class PID {
    public:
        float cmd;
        void PIDUpdate(float currValue, float reference);
        void PIDUpdate2(float currValue, float reference);
        float saturate(float value, float boundary);
        float limiting(float value, float boundary);
        float getCmdValue();
	float getBiasValue();        
    float geterr();
    void setGains(float Pgain, float Igain, float Isat);
	PID(float Pgain, float Igain, float Dgain, float Isat, float Csat, float CmdBias, float DeadZone);

    private:
        float kp;
        float ki;
        float kd;
        float intSat;
        float cmdSat;
        float bias; 
        float intError;
        float prevError;
        float dError;    
        float deadzone;
};

#endif
