#include "robot.h"

#include "robot_tasks.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "launch_task.h"
#include "remote.h"
#include "bsp_can.h"
#include "gimbal_task.h"
#include <math.h>
#include "imu_task.h"
#include "user_math.h"
#include "referee_system.h"
#include "buzzer.h"
#include "ui.h"
#include "Fusion.h"
#include "robot_param.h"
#include "board_comm_task.h"
extern DJI_Motor_Handle_t *g_yaw;
#define SPIN_TOP_OMEGA (1.0f)
#define ROBOT_TASK_PERIOD (0.002f)
#define MAX_YAW_SPEED_RAD (7.5f)

#define MAX_YAW_INCREMENT (MAX_YAW_SPEED_RAD * ROBOT_TASK_PERIOD)
#define KEYBOARD_RAMP_COEF (0.005f)
#define SPINTOP_COEF (0.003f)
#define CONTROLLER_RAMP_COEF (0.8f)
#define MAX_SPEED (1.2f)

Robot_State_t g_robot_state = {0, 0};
float g_chassis_height_arr[4] = {0.13f, 0.20f, 0.28f, 0.35f};
int8_t g_current_height_index = 0;
Key_Prev_t g_key_prev = {0};
extern Launch_Target_t g_launch_target;
extern Remote_t g_remote;
extern FusionAhrs g_fusion_ahrs;
extern uint8_t g_board_comm_first_part_established;
extern uint8_t g_board_comm_second_part_established;
uint8_t g_start_safely = 0;

void Robot_Cmd_Loop(void);
void _toggle_robot_state(uint8_t *state);

void Robot_Init()
{
    Buzzer_Init();
    FusionAhrsInitialise(&g_fusion_ahrs);
    CAN_Service_Init();
    Board_Comm_Task_Init();
#ifdef MASTER
    Melody_t system_init_melody = {
        .notes = SYSTEM_INITIALIZING,
        .loudness = 0.5f,
        .note_num = SYSTEM_INITIALIZING_NOTE_NUM,
    };
    Buzzer_Play_Melody(system_init_melody); // TODO: Change to non-blocking
    // Initialize all hardware
    Chassis_Task_Init();
    Gimbal_Task_Init();
    Launch_Task_Init();
    Remote_Init(&huart3);
    Referee_System_Init(&huart1);
    // Jetson_Orin_Init(&huart6);
#else
    Melody_t system_init_melody = {
        .notes = SYSTEM_READY,
        .loudness = 0.5f,
        .note_num = SYSTEM_READY_NOTE_NUM,
    };
    Buzzer_Play_Melody(system_init_melody); // TODO: Change to non-blocking
#endif

    //  Initialize all tasks
    Robot_Tasks_Start();
}

void Robot_Ctrl_Loop()
{
    // Control loop for the robot
    Robot_Cmd_Loop();
#ifdef MASTER
    Referee_Set_Robot_State();
    Chassis_Ctrl_Loop();
    Gimbal_Ctrl_Loop();
    Launch_Ctrl_Loop();
#endif
}
/**
 *  B - Flywheel On Off
 *  G - Spin Top Mode
 */
void Robot_Cmd_Loop()
{
    // safely startly will be 1 after first time right switch is down
    // this is to make sure the robot is disabled when it starts
    if (g_robot_state.safely_started == 1)
    {
        if ((g_remote.online_flag == REMOTE_OFFLINE) || (g_remote.controller.right_switch == DOWN))
        {
            g_robot_state.enabled = 0;
            g_launch_target.flywheel_enabled = 0;
            g_robot_state.gimbal_yaw_angle = g_imu.rad.yaw;
            g_current_height_index = 0;
        }
        else
        {
            g_robot_state.enabled = 1;

            /* Gimbal starts here */ // (Launch enable in last if statement)
            if (g_remote.keyboard.Shift == 1)
            {
                g_robot_state.chassis_move_speed_ratio = 1.5f;
            }
            else if ((g_remote.keyboard.Ctrl == 1) || (g_robot_state.chassis_height > 0.23f))
            {
                g_robot_state.chassis_move_speed_ratio = 0.5f;
            }
            else
            {
                g_robot_state.chassis_move_speed_ratio = 1.0f;
            }

            g_robot_state.vy_keyboard = ((1.0f - KEYBOARD_RAMP_COEF) * g_robot_state.vy_keyboard + g_remote.keyboard.W * KEYBOARD_RAMP_COEF - g_remote.keyboard.S * KEYBOARD_RAMP_COEF);
            g_robot_state.vx_keyboard = ((1.0f - KEYBOARD_RAMP_COEF) * g_robot_state.vx_keyboard - g_remote.keyboard.A * KEYBOARD_RAMP_COEF + g_remote.keyboard.D * KEYBOARD_RAMP_COEF);
            g_robot_state.vx = g_robot_state.vx_keyboard + g_remote.controller.left_stick.x / REMOTE_STICK_MAX;
            g_robot_state.vy = g_robot_state.vy_keyboard + g_remote.controller.left_stick.y / REMOTE_STICK_MAX;
            g_robot_state.vx *= MAX_SPEED * g_robot_state.chassis_move_speed_ratio;
            g_robot_state.vy *= MAX_SPEED * g_robot_state.chassis_move_speed_ratio;

            // Coordinate of the wheel legged chassis is opposite of the leg coordinate, therefore the sign is flipped
            g_robot_state.vx *= -1;
            g_robot_state.vy *= -1;

            g_robot_state.chassis_y_speed = g_robot_state.vy;
            g_robot_state.chassis_x_speed = g_robot_state.vx;

            if (fabs(g_robot_state.chassis_y_speed) > 0.05f || fabs(g_robot_state.chassis_x_speed) > 0.05f)
            {
                g_robot_state.spintop_mode = 0;
                // g_current_height_index = 0;
            }
            // Wheel Facing Mode
            if (fabs(g_robot_state.chassis_y_speed) < 0.05f && fabs(g_robot_state.chassis_x_speed) > 0.08f)
            {
                g_robot_state.wheel_facing_mode = 1;
            }
            if (fabs(g_robot_state.chassis_y_speed) > 0.08f && fabs(g_robot_state.chassis_x_speed) < 0.05f)
            {
                g_robot_state.wheel_facing_mode = 0;
            }
            if (g_remote.mouse.left == 1 && fabs(g_robot_state.chassis_y_speed) < 0.3f)
            {
                g_robot_state.wheel_facing_mode = 1;
            }
            if (g_remote.mouse.right == 1 && fabs(g_robot_state.chassis_x_speed) < 0.3f)
            {
                g_robot_state.wheel_facing_mode = 0;
            }
            if (g_remote.controller.left_switch == DOWN)
            {
                if (g_key_prev.prev_left_switch != DOWN)
                { // if previous state is not mid change, this allows keyboard to toggle spin top mode
                    g_robot_state.spintop_mode = 0;
                    g_launch_target.flywheel_enabled = 0;
                }
            }
            else if (g_remote.controller.left_switch == MID)
            {
                if (g_key_prev.prev_left_switch != MID)
                { // if previous state is not mid change, this allows keyboard to toggle spin top mode
                    g_robot_state.spintop_mode = 1;

                }
            }
            else if (g_remote.controller.left_switch == UP) // launch on - off
            {
                if (g_key_prev.prev_left_switch != UP)
                { // if previous state is not mid change, this allows keyboard to toggle spin top mode
                    g_robot_state.spintop_mode = 0;
                    g_launch_target.flywheel_enabled = 1;
                }
                g_launch_target.flywheel_enabled = 1;
            }
            g_key_prev.prev_left_switch = g_remote.controller.left_switch;

            // this does not interfere with remote control
            if (g_remote.controller.right_switch != UP)
            {
                g_robot_state.chassis_height = g_robot_state.chassis_height * 0.99f + 0.01f * g_chassis_height_arr[g_current_height_index];
            }
            /* Chassis ends here */

            /* Gimbal starts here */
            if ((g_remote.controller.right_switch == UP) || (g_remote.mouse.right == 1)) // mouse right button auto aim
            {
#ifdef ORIN
                if (g_orin_data.receiving.auto_aiming.yaw != 0 || g_orin_data.receiving.auto_aiming.pitch != 0)
                {
                    g_robot_state.gimbal_yaw_angle = (1 - 0.2f) * g_robot_state.gimbal_yaw_angle + (0.2f) * (g_imu.rad.yaw + g_orin_data.receiving.auto_aiming.yaw / 180.0f * PI);         // + orin
                    g_robot_state.gimbal_pitch_angle = (1 - 0.2f) * g_robot_state.gimbal_pitch_angle + (0.2f) * (g_imu.rad.pitch + g_orin_data.receiving.auto_aiming.pitch / 180.0f * PI); // + orin
                }
#endif
            }
            if ((g_remote.controller.right_switch == UP))
            {
                g_robot_state.chassis_height = g_robot_state.chassis_height * 0.99f + 0.01f * 0.35f;
            }
            else if (g_remote.controller.right_switch == MID)
            {
                float yaw_increment;
                if (g_remote.keyboard.Q == 1 && g_key_prev.prev_Q == 0 && fabs(g_robot_state.chassis_y_speed) < 0.4f)
                {
                    // -0.2 is to ensure q and e is turning it its desire direction, or angle wrapping will find the cloest way.
                    yaw_increment = -PI + 0.2f;
                    g_robot_state.gimbal_switching_dir_pending = 1;
                }
                else if (g_remote.keyboard.E == 1 && g_key_prev.prev_E == 0 && fabs(g_robot_state.chassis_y_speed) < 0.4f)
                {
                    yaw_increment = PI - 0.2f;
                    g_robot_state.gimbal_switching_dir_pending = 1;
                }
                else
                {
                    yaw_increment = (g_remote.controller.right_stick.x / 50000.0f + g_remote.mouse.x / 50000.0f);
                    yaw_increment = fabs(yaw_increment) > MAX_YAW_INCREMENT ? (fabs(yaw_increment) / (yaw_increment)) * MAX_YAW_INCREMENT : yaw_increment;
                }
                g_robot_state.gimbal_yaw_angle -= yaw_increment;                                                                   // controller and mouse
                g_robot_state.gimbal_pitch_angle -= (g_remote.controller.right_stick.y / 100000.0f - g_remote.mouse.y / 50000.0f); // controller and mouse
            }

            if (fabs(fmod(g_robot_state.gimbal_yaw_angle - g_imu.rad_fusion.yaw, 2 * PI)) < 0.03f)
            {
                g_robot_state.gimbal_switching_dir_pending = 0;
            }
            /* Gimbal ends here */

            /* Launch control starts here */
            if (1)
            {
                g_launch_target.prev_burst_launch_flag = g_launch_target.burst_launch_flag;
                if (g_remote.controller.left_switch == MID && g_key_prev.prev_left_switch == UP)
                { // dial wheel forward single fire
                    g_launch_target.single_launch_flag = 0;
                    g_launch_target.burst_launch_flag = 1;
                }
                // else if ((g_remote.controller.wheel > 50.0f) || (g_remote.mouse.left == 1) || (g_remote.mouse.right == 1))
                // { // dial wheel backward burst fire
                //     g_launch_target.single_launch_flag = 0;
                //     g_launch_target.burst_launch_flag = 1;
                // }
                else
                { // dial wheel mid stop fire
                    g_launch_target.single_launch_flag = 0;
                    g_launch_target.single_launch_finished_flag = 0;
                    g_launch_target.burst_launch_flag = 0;
                }
                if (g_launch_target.burst_launch_flag == 1)
                {
                    g_launch_target.flywheel_enabled = 1;
                }
                // reverse 1 projectile each time after shooting
                g_launch_target.prev_reverse_flag = g_launch_target.reverse_flag;
                if ((g_remote.keyboard.R == 1 && g_key_prev.prev_R == 0) 
                || (g_remote.controller.wheel < -50.0f && g_launch_target.flywheel_enabled) || 
                (!g_launch_target.burst_launch_flag && g_launch_target.prev_burst_launch_flag))
                {
                    g_launch_target.reverse_flag = 1;
                    g_launch_target.calculated_heat = Referee_Robot_State.Shooter_Heat_1;
                }
                else
                {
                    g_launch_target.reverse_flag = 0;
                }
                // if ((g_launch_target.burst_launch_flag == 0 && g_launch_target.prev_burst_launch_flag
                // || g_remote.keyboard.R == 1 && g_key_prev.prev_R == 0))
                // {
                //     g_launch_target.reverse_burst_launch_pending_flag = 1;
                // }
            }
            else
            {
                g_launch_target.single_launch_flag = 0;
                g_launch_target.single_launch_finished_flag = 0;
                g_launch_target.burst_launch_flag = 0;
            }
            /* Launch control ends here */

            /* Keyboard Toggles Start Here */
            if (g_remote.keyboard.B == 1 && g_key_prev.prev_B == 0)
            {
                _toggle_robot_state(&g_launch_target.flywheel_enabled);
            }
            if (g_remote.keyboard.G == 1 && g_key_prev.prev_G == 0)
            {
                _toggle_robot_state(&g_robot_state.spintop_mode);
            }
            if (g_remote.keyboard.V == 1 && g_key_prev.prev_V == 0)
            {
                _toggle_robot_state(&g_robot_state.UI_enabled);
            }
            if (g_remote.keyboard.F == 1 && g_key_prev.prev_F == 0)
            {
                // g_robot_state.chassis_height = g_robot_state.chassis_height * 0.995f + 0.05f * 0.30f;
                g_current_height_index++;
                __MAX_LIMIT(g_current_height_index, 0, 3);
            }
            if (g_remote.keyboard.C == 1 && g_key_prev.prev_C == 0)
            {
                // g_robot_state.chassis_height = g_robot_state.chassis_height * 0.995f + 0.05f * 0.13f;
                g_current_height_index--;
                __MAX_LIMIT(g_current_height_index, 0, 3);
            }
            if (g_remote.controller.right_switch == MID && g_key_prev.prev_right_switch == UP)
            {
                g_robot_state.chassis_height = 0.13f;
            }
            g_key_prev.prev_B = g_remote.keyboard.B;
            g_key_prev.prev_G = g_remote.keyboard.G;
            g_key_prev.prev_V = g_remote.keyboard.V;
            g_key_prev.prev_F = g_remote.keyboard.F;
            g_key_prev.prev_C = g_remote.keyboard.C;
            g_key_prev.prev_Q = g_remote.keyboard.Q;
            g_key_prev.prev_E = g_remote.keyboard.E;
            g_key_prev.prev_R = g_remote.keyboard.R;
            g_key_prev.prev_right_switch = g_remote.controller.right_switch;
            g_key_prev.prev_left_switch = g_remote.controller.left_switch;
            /* Keyboard Toggles Start Here */

            /* AutoAiming Flag, not used only for debug */
            if ((g_remote.mouse.right == 1) || (g_remote.controller.right_switch == UP))
            {
                g_robot_state.autoaiming_enabled = 1;
            }
            else
            {
                g_robot_state.autoaiming_enabled = 0;
            }

            /* Hardware Limits */
            g_robot_state.gimbal_yaw_angle = fmod(g_robot_state.gimbal_yaw_angle, 2 * PI);
            __MAX_LIMIT(g_robot_state.gimbal_pitch_angle, -0.4f, 0.4f);
            __MAX_LIMIT(g_robot_state.chassis_x_speed, -MAX_SPEED, MAX_SPEED);
            __MAX_LIMIT(g_robot_state.chassis_y_speed, -MAX_SPEED, MAX_SPEED);

            /* power buffer*/
            // float power_buffer = Referee_System.Power_n_Heat.Chassis_Power_Buffer / 60.0f;
            // if (power_buffer < 0.8f)
            // {
            //     g_robot_state.chassis_x_speed *= pow(power_buffer,1);
            //     g_robot_state.chassis_y_speed *= pow(power_buffer,1);
            //     g_robot_state.chassis_omega *= pow(power_buffer,1);
            // }
        }
    }
    else
    {
        // Ensure controller is down before enabling robot, ensure imu is initialized before enabling robot
#ifdef MASTER
        if ((g_remote.controller.right_switch == DOWN) && (g_imu.deg_fusion.pitch != 0.0f) && (g_imu.imu_ready_flag == 1) && (g_remote.controller.left_stick.y != -660) && g_board_comm_first_part_established == 1 && g_board_comm_second_part_established == 1)
#else
        if ((g_imu.deg_fusion.pitch != 0.0f) && (g_imu.imu_ready_flag == 1))
#endif
        {
            g_robot_state.safely_started = 1;
        }
    }
}

void _toggle_robot_state(uint8_t *state)
{
    *state ^= 0x01;
}