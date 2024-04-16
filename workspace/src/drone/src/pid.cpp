class PID
{
public:
    PID(float Kp_, float Ki_, float Kd_, float dt_) : Kp(Kp_), Ki(Ki_), Kd(Kd_), dt(dt_) {}
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

        float output = (Pout + Iout + Dout);
        return output;
    }
private:
    float Kp;
    float Ki;
    float Kd;
    double dt;
    float pre_error = 0;
    float integral = 0;
};