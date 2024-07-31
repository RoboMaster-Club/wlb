#ifndef THIGH_H
#define THIGH_H

#include "can.h"
#include "math.h"
#include "pid.h"
#include "user_math.h"

typedef struct
{
    float phi1;
    float phi4;
} Leg_Info_t;

typedef struct leg
{
    float phi1;
    float phi2;
    float phi3;
    float phi4;
    
    float phi1_dot;
    float phi2_dot;
    float phi3_dot;
    float phi4_dot;

    float phi0;
    float last_phi0;
    float phi0_dot;
    float length;
    float length_dot;

    float xe1, xe2, ye1, ye2;
    float torq1, torq4;

    float current_disp, current_vel, current_theta, current_theta_dot, last_theta;
    float target_leg_virtual_force, target_leg_virtual_torq, target_wheel_torq;
    float compensatioin_torq;
    float target_length;
    uint32_t current_tick, last_tick;
    float normal_force;
    float actual_virtual_torq;
} Leg_t;

void Leg_ForwardKinematics(Leg_t *leg, float phi1, float phi2, float phi1_dot, float phi2_dot);
void Leg_InverseKinematics(float height, float leg_angle, float *leg_1, float *leg_2);
void Leg_VMC(Leg_t *leg, float torq1, float torq4);
#endif
