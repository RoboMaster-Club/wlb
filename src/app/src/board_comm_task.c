#include "board_comm_task.h"
#include "bsp_can.h"
#include "robot.h"
#include "imu_task.h"
#include <stdint.h>
#include "robot_param.h"


Board_Comm_Package_t g_board_comm_package;
CAN_Instance_t *g_board_comm_part1;
CAN_Instance_t *g_board_comm_part2;
CAN_Instance_t *g_board_comm_part3;
uint8_t g_board_comm_first_part_sending_pending = 0;
uint8_t g_board_comm_second_part_sending_pending = 0;
uint8_t g_board_comm_third_part_sending_pending = 0;
uint8_t g_board_comm_initialized = 0;
uint8_t g_board_comm_first_part_established = 0;
uint8_t g_board_comm_second_part_established = 0;
void board_comm_recv_first_part(CAN_Instance_t *can_instance);
void board_comm_recv_second_part(CAN_Instance_t *can_instance);
void board_comm_recv_third_part(CAN_Instance_t *can_instance);

void Board_Comm_Task_Init()
{
    #ifdef MASTER
    g_board_comm_part1 = CAN_Device_Register(1, 0x060, 0x050, board_comm_recv_first_part);
    g_board_comm_part2 = CAN_Device_Register(1, 0x061, 0x051, board_comm_recv_second_part);
    g_board_comm_part3 = CAN_Device_Register(1, 0x062, 0x052, board_comm_recv_third_part);
    #else
    #pragma message "Board_Comm_Task_Init() is compiled"
    g_board_comm_part1 = CAN_Device_Register(1, 0x050, 0x060, board_comm_recv_first_part);
    g_board_comm_part2 = CAN_Device_Register(1, 0x051, 0x061, board_comm_recv_second_part);
    g_board_comm_part3 = CAN_Device_Register(1, 0x052, 0x062, board_comm_recv_third_part);
    #endif
}

void board_comm_recv_first_part(CAN_Instance_t *can_instance)
{
    g_board_comm_first_part_established = 1;
    memcpy(&g_board_comm_package.robot_pitch, can_instance->rx_buffer, 8);
}

void board_comm_recv_second_part(CAN_Instance_t *can_instance)
{
    g_board_comm_second_part_established = 1;
    memcpy(&g_board_comm_package.x_acceleration, can_instance->rx_buffer, 8);
}

void board_comm_recv_third_part(CAN_Instance_t *can_instance)
{
    g_board_comm_second_part_established = 1;
    memcpy(&g_board_comm_package.robot_roll, can_instance->rx_buffer, 8);
}

void Board_Comm_Task_Loop(void)
{
#ifndef MASTER
    if (g_robot_state.safely_started == 1) {
        memcpy(&(g_board_comm_part1->tx_buffer[0]), &(g_imu.rad_fusion.roll), 4);
        memcpy(&(g_board_comm_part1->tx_buffer[4]), &(g_imu.bmi088_raw.gyro[0]), 4);
        g_board_comm_first_part_sending_pending = 1;

        memcpy(&(g_board_comm_part2->tx_buffer[0]), &(g_imu.accel_body[1]), 4);
        memcpy(&(g_board_comm_part2->tx_buffer[4]), &(g_imu.rad_fusion.yaw), 4);
        g_board_comm_second_part_sending_pending = 1;

        memcpy(&(g_board_comm_part3->tx_buffer[0]), &(g_imu.rad_fusion.pitch), 4);
        memcpy(&(g_board_comm_part3->tx_buffer[4]), &(g_imu.bmi088_raw.gyro[1]), 4);
        g_board_comm_third_part_sending_pending = 1;
    }
#endif
}