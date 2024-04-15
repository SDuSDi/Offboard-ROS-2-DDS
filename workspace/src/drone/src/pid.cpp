#include <iostream>

class PID
{
public:
    PID(float Kp, float Ki, float Kd, float dt){
        Kp = Kp;
        Ki = Ki;
        Kd = Kd;
        dt = dt;
    }
    float calculate(float setpoint, float pv){
        
        float error = setpoint - pv;

        // Proportional term
        float Pout = Kp * error;
        // Integral term
        integral += error * dt;
        float Iout = Ki * integral;
        // Derivative term
        float derivative = (error - pre_error) / dt;
        float Dout = Kd * derivative;

        pre_error = error;

        float res = (Pout + Iout + Dout);
        return res;
    }
private:
    float Kp;
    float Ki;
    float Kd;
    float dt;
    float pre_error = 0;
    float integral = 0;
};