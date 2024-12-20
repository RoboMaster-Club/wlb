#ifndef PID_H
#define PID_H

typedef struct pid_t
{
    float kp;
    float ki;
    float kd;
    float kf;
    float integral_limit;
    float feedforward_limit;
    float i_out;
    float f_out;
    float output_limit;
    float dead_zone;
    float ref;
    
    float prev_error;

    float output;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit, float integral_limit, float dead_zone);
void PID_Reset(PID_t *pid);
float PID(PID_t *pid, float error);
float PID_dt(PID_t *pid, float error, float dt);
#endif
