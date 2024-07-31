#ifndef BOARD_COMM_TASK_H
#define BOARD_COMM_TASK_H
typedef struct Board_Comm_Package_s
{
    float robot_pitch;
    float robot_pitch_rate;
    float x_acceleration;
    float yaw;
    float robot_roll;
    float robot_roll_rate;
} Board_Comm_Package_t;

void Board_Comm_Task_Init(void);
void Board_Comm_Task_Loop(void);

#endif
