#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdint.h>

typedef struct chassis_s
{
    float target_yaw;
    float target_yaw_speed;
    float last_yaw_raw;
    float current_yaw;
    int16_t total_turns;
    float target_vel;
    float wheel_x_turning_offset;
    float turning_radius;
    float centripetal_force;
    float angle_diff;
    float forward_speed;
    uint8_t chassis_killed_by_referee;
    float roll_compensation_height;
} Chassis_t;

typedef struct jump_state_machine_s
{
    uint8_t flag_jump_start;
    uint8_t flag_retracted;
    uint8_t flag_extended;
    uint8_t flag_reretracted;
    uint8_t flag_reextended;
} Jump_State_Machine_t;

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);

#endif // CHASSIS_TASK_H
