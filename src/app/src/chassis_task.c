#include "chassis_task.h"

#include "robot.h"
#include "mf_motor.h"
#include "remote.h"
#include "imu_task.h"
#include "bsp_can.h"
#include "five_bar_leg.h"
#include "wlb_lqr_controller.h"
#include "robot_param.h"
#include "kalman_filter.h"
#include "board_comm_task.h"
#include "gimbal_task.h"
#include "dji_motor.h"
#include "bsp_daemon.h"
#include "referee_system.h"

#define CHASSIS_POWER_OFF_TIMEOUT_MS (300)
extern DJI_Motor_Handle_t *g_yaw;
extern Referee_Robot_State_t Referee_Robot_State;
extern Board_Comm_Package_t g_board_comm_package;
float vel_kalman;
uint8_t last_spintop_mode;
/* Kalman Filter */
float vaEstimateKF_F[4] = {1.0f, 0.003f,
                           0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f}; // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.1f, 0.0f,
                           0.0f, 0.1f}; // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f,
                           0.0f, 100.0f};

float vaEstimateKF_K[4];

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f}; // 设置矩阵H为常量
#define KALMAN_FILTER
#define TASK_TIME (0.002f)
/* Statistics */
#define UP_ANGLE_ODD (-48.0f)
#define UP_ANGLE_EVEN (180.0f - (-48.0f))
#define UP_1 (19806.0f) //(58396.0f) // 19806
#define UP_2 (53819.0f - 65536.0f) //(17889.0f) // 53819
#define UP_3 (32624.0f) //(51440.0f) // 32624
#define UP_4 (36213.0f) //(22877.0f) // 36213

#define DOWN_ANGLE_ODD (87.0f)
#define DOWN_ANGLE_EVEN (180.0f - 87.0f)
#define DOWN_1 (60722.0f - 65536.0f) // (33540.0f) // 60722
#define DOWN_2 (12924.0f) // due to zero-crossing// (42464.0f) // 12924
#define DOWN_3 (7939.0f) // (26586.0f) // 7939
#define DOWN_4 (60973.0f) // (47774.0f) // 60973

#define FOOT_MOTOR_MAX_TORQ (2.4f)
#define FOOT_MF9025_MAX_TORQ_INT ((FOOT_MOTOR_MAX_TORQ / MF9025_TORQ_CONSTANT) / 16.5f * 2048.0f)
#define CHASSIS_DEFAULT_HEIGHT (0.13f)

float vel_acc[2];
KalmanFilter_t vaEstimateKF;
extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
extern IMU_t g_imu;
Chassis_t g_chassis;
Jump_State_Machine_t g_jump_state_machine;
uint8_t g_left_foot_initialized = 0;
uint8_t g_right_foot_initialized = 0;
MF_Motor_Handle_t *g_motor_lf, *g_motor_rf, *g_motor_lb, *g_motor_rb;
MF_Motor_Handle_t *g_left_foot_motor, *g_right_foot_motor;
Leg_t g_leg_left, g_leg_right;
lqr_ss_t g_lqr_left_state, g_lqr_right_state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
lqr_u_t g_u_left, g_u_right;
PID_t g_pid_left_leg_length, g_pid_left_leg_angle;
PID_t g_pid_right_leg_length, g_pid_right_leg_angle;
PID_t g_balance_angle_pid, g_balance_vel_pid;
PID_t g_pid_yaw_angle;
PID_t g_pid_anti_split;
PID_t g_pid_follow_gimbal;
PID_t g_pid_roll_compensation;
Leg_t test = {
    .phi1 = PI,
    .phi4 = 0,
    .phi1_dot = 0,
    .phi4_dot = 0,
};
Daemon_Instance_t *g_daemon_chassis_power_guard;
void _get_leg_statistics();
void _wheel_leg_estimation(float robot_yaw, float robot_pitch, float robot_pitch_dot);
void _leg_length_controller(float chassis_height, float robot_roll);
void _lqr_balancce_controller();
void _vmc_torq_calc();
float g_test_angle;

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel)
{
    // 卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] = vel; // 测量速度
    EstimateKF->MeasuredVector[1] = acc; // 测量加速度

    // 卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

void _power_chassis_power_callback()
{
    g_chassis.chassis_killed_by_referee = 1;
}

void Chassis_Task_Init()
{
    // uint16_t reload_value = CHASSIS_POWER_OFF_TIMEOUT_MS / DAEMON_PERIOD;
	// uint16_t initial_counter = reload_value;
	// g_daemon_chassis_power_guard = Daemon_Register(reload_value, initial_counter, _power_chassis_power_callback);
    // Initialize motors
    MF_Motor_Config_t motor_config = {
        .can_bus = 1,
        .tx_id = 0x141,
        .rx_id = 0x141,
    };
    g_motor_lf = MF_Motor_Init(motor_config);
    motor_config.tx_id = 0x142;
    motor_config.rx_id = 0x142;
    g_motor_lb = MF_Motor_Init(motor_config);
    motor_config.tx_id = 0x143;
    motor_config.rx_id = 0x143;
    g_motor_rb = MF_Motor_Init(motor_config);
    motor_config.tx_id = 0x144;
    motor_config.rx_id = 0x144;
    g_motor_rf = MF_Motor_Init(motor_config);

    /* Foot Motor Init */
    motor_config.tx_id = 0x147;
    motor_config.rx_id = 0x147;
    g_left_foot_motor = MF_Motor_Init(motor_config);
    motor_config.tx_id = 0x148;
    motor_config.rx_id = 0x148;
    g_right_foot_motor = MF_Motor_Init(motor_config);

    MF_Motor_Broadcast_Init(1);
    PID_Init(&g_pid_left_leg_length, 1000.0f, 0.0f, 300.0f, 400.0f, 0.0f, 0.0f);
    PID_Init(&g_pid_right_leg_length, 1000.0f, 0.0f, 300.0f, 400.0f, 0.0f, 0.0f);

    PID_Init(&g_pid_left_leg_angle, 15.0f, 0.0f, 5.75f, 10.0f, 0.0f, 0.0f);
    PID_Init(&g_pid_right_leg_angle, 15.0f, 0.0f, 5.75f, 10.0f, 0.0f, 0.0f);

    PID_Init(&g_balance_angle_pid, 600.0f, 0.0f, .065f, 10.0f, 0.0f, 0.0f);
    PID_Init(&g_balance_vel_pid, 10.0f, 0.0f, 0.001f, 10.0f, 0.0f, 0.0f);

    PID_Init(&g_pid_yaw_angle, 1.3f, 0.0f, 1.1f, 5.0f, 0.0f, 0.0f);
    PID_Init(&g_pid_anti_split, 100.0f, 0.0f, 5.0f, 40.0f, 0.0f, 0.0f);

    PID_Init(&g_pid_follow_gimbal, 8.0f, 0.0f, 0.95f, 6.0f, 0.0f, 0.0f);
    g_robot_state.chassis_height = CHASSIS_DEFAULT_HEIGHT;

    PID_Init(&g_pid_roll_compensation, 0.4f, 0.0000f, 0.0f, 0.20f, 0.15f, 0.0f);
    xvEstimateKF_Init(&vaEstimateKF);
}

uint8_t _is_turning()
{
    return (g_remote.controller.right_stick.x != 0) || (g_remote.keyboard.A) || (g_remote.keyboard.D) || (g_remote.mouse.x != 0);
}

void _hip_motor_torq_ctrl(float torq_lf, float torq_lb, float torq_rb, float torq_rf)
{
    int16_t torq1 = torq_lf / MG8016_TORQ_CONSTANT * (2048.0f / 33.0f);
    int16_t torq2 = torq_lb / MG8016_TORQ_CONSTANT * (2048.0f / 33.0f);
    int16_t torq3 = torq_rb / MG8016_TORQ_CONSTANT * (2048.0f / 33.0f);
    int16_t torq4 = torq_rf / MG8016_TORQ_CONSTANT * (2048.0f / 33.0f);
    __MAX_LIMIT(torq1, -2000, 2000);
    __MAX_LIMIT(torq2, -2000, 2000);
    __MAX_LIMIT(torq3, -2000, 2000);
    __MAX_LIMIT(torq4, -2000, 2000);
    MF_Motor_Broadcast_Torq_Ctrl(1, torq1, torq2, torq3, torq4);
}

void _foot_motor_torq_ctrl(float torq_left, float torq_right)
{
    int16_t torq_left_int = torq_left / MF9025_TORQ_CONSTANT * (2048.0f / 16.5f);
    int16_t torq_right_int = torq_right / MF9025_TORQ_CONSTANT * (2048.0f / 16.5f);
    __MAX_LIMIT(torq_left, -FOOT_MF9025_MAX_TORQ_INT, FOOT_MF9025_MAX_TORQ_INT);
    __MAX_LIMIT(torq_right, -FOOT_MF9025_MAX_TORQ_INT, FOOT_MF9025_MAX_TORQ_INT);
    MF_Motor_TorqueCtrl(g_left_foot_motor, torq_left_int);
    MF_Motor_TorqueCtrl(g_right_foot_motor, torq_right_int);
}

/**
 * @brief Wheel leg estimation
 *
 * @note robot pitch is in the positive direction, adjust sign and offset before passing into this function
 */
void _wheel_leg_estimation(float robot_yaw, float robot_pitch, float robot_pitch_dot)
{
    _get_leg_statistics();

    if (robot_yaw - g_chassis.last_yaw_raw > PI)
    {
        g_chassis.total_turns -= 1;
    }
    else if (robot_yaw - g_chassis.last_yaw_raw < -PI)
    {
        g_chassis.total_turns += 1;
    }
    g_chassis.last_yaw_raw = robot_yaw;
    g_chassis.current_yaw = robot_yaw + 2 * PI * g_chassis.total_turns;

    Leg_ForwardKinematics(&g_leg_left, g_leg_left.phi1, g_leg_left.phi4, g_leg_left.phi1_dot, g_leg_left.phi4_dot);
    g_lqr_left_state.theta = -(g_leg_left.phi0 - PI / 2 + robot_pitch);
    g_lqr_left_state.theta_dot = -(g_leg_left.phi0_dot + robot_pitch_dot);
    g_lqr_left_state.phi = robot_pitch;
    g_lqr_left_state.phi_dot = robot_pitch_dot;
    g_lqr_left_state.leg_len = g_leg_left.length;
    Leg_ForwardKinematics(&g_leg_right, g_leg_right.phi1, g_leg_right.phi4, g_leg_right.phi1_dot, g_leg_right.phi4_dot);
    g_lqr_right_state.theta = -(-g_leg_right.phi0 + PI / 2 + robot_pitch);
    g_lqr_right_state.theta_dot = -(-g_leg_right.phi0_dot + robot_pitch_dot);
    g_lqr_right_state.phi = (robot_pitch);
    g_lqr_right_state.phi_dot = (robot_pitch_dot);
    g_lqr_right_state.leg_len = g_leg_right.length;

    static float wr, wl = 0.0f;
    static float vrb, vlb = 0.0f;
    static float aver_v = 0.0f;

    if (g_imu.imu_ready_flag)
    {
        wl = (-g_left_foot_motor->stats->velocity * DEG_TO_RAD) + robot_pitch_dot + g_leg_left.phi3_dot;                                                                                       // 右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        vlb = wl * FOOT_WHEEL_RADIUS + g_leg_left.length * g_lqr_left_state.theta_dot * arm_cos_f32(g_lqr_left_state.theta) + g_leg_left.length_dot * arm_sin_f32(g_lqr_left_state.theta_dot); // 机体b系的速度

        wr = (g_right_foot_motor->stats->velocity * DEG_TO_RAD) + robot_pitch_dot - g_leg_right.phi2_dot;                                                                                           // 左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        vrb = wr * FOOT_WHEEL_RADIUS + g_leg_right.length * g_lqr_right_state.theta_dot * arm_cos_f32(g_lqr_right_state.theta) + g_leg_right.length_dot * arm_sin_f32(g_lqr_right_state.theta_dot); // 机体b系的速度

        aver_v = (vrb + vlb) / 2.0f; // 取平均
        xvEstimateKF_Update(&vaEstimateKF, g_board_comm_package.x_acceleration, aver_v);

        // 原地自转的过程中v_filter和x_filter应该都是为0
        vel_kalman = vel_acc[0]; // 得到卡尔曼滤波后的速度
                                 //  chassis_move.x_filter=chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);

        g_lqr_left_state.x_dot = vel_kalman;
        g_lqr_right_state.x_dot = vel_kalman;
        if (vel_kalman > 0.01f || vel_kalman < -0.01f)
        {
            g_lqr_left_state.x += g_lqr_left_state.x_dot * TASK_TIME;
            g_lqr_right_state.x += g_lqr_right_state.x_dot * TASK_TIME;
        }
    }
}

void _get_leg_statistics()
{
# pragma message "we are too lazy to write the legit function, if you see this, feel free to correct the angle overflow :)"
    float angle_lf = g_motor_lf->stats->angle;
    float angle_lb = g_motor_lb->stats->angle;
    float angle_rb = g_motor_rb->stats->angle;
    float angle_rf = g_motor_rf->stats->angle;
#define MF8016_MAX_TICK (65536)
    // 40000 is an arbitrary number, it is because the motor task space includes the zero-crosing point of the encoder
    if (angle_lf > 40000)
    {
        angle_lf -= MF8016_MAX_TICK;
    }
    if (angle_lb > 40000)     {
        angle_lb -= MF8016_MAX_TICK;
    }

    g_leg_left.phi1 = ((angle_lb - DOWN_2) * ((UP_ANGLE_EVEN - DOWN_ANGLE_EVEN) / (UP_2 - DOWN_2)) + DOWN_ANGLE_EVEN) * DEG_TO_RAD;
    // g_leg_left.phi1_dot = g_motor_lb->stats->velocity;
    g_leg_left.phi4 = ((angle_lf - DOWN_1) * ((UP_ANGLE_ODD - DOWN_ANGLE_ODD) / (UP_1 - DOWN_1)) + DOWN_ANGLE_ODD) * DEG_TO_RAD;
    // g_leg_left.phi4_dot = g_motor_lf->stats->velocity;

    g_leg_right.phi1 = ((angle_rf - DOWN_4) * ((UP_ANGLE_EVEN - DOWN_ANGLE_EVEN) / (UP_4 - DOWN_4)) + DOWN_ANGLE_EVEN) * DEG_TO_RAD;
    // g_leg_right.phi1_dot = g_motor_rf->stats->velocity;
    g_leg_right.phi4 = ((angle_rb - DOWN_3) * ((UP_ANGLE_ODD - DOWN_ANGLE_ODD) / (UP_3 - DOWN_3)) + DOWN_ANGLE_ODD) * DEG_TO_RAD;
    // g_leg_right.phi4_dot = g_motor_rb->stats->velocity;
}

void _target_state_reset()
{
    g_lqr_left_state.target_x = g_lqr_left_state.x;
    g_lqr_right_state.target_x = g_lqr_right_state.x;
    g_robot_state.chassis_height = CHASSIS_DEFAULT_HEIGHT;
    g_chassis.target_yaw = g_chassis.current_yaw;
    g_chassis.roll_compensation_height = 0;
    PID_Reset(&g_pid_left_leg_length);
    PID_Reset(&g_pid_right_leg_length);
    PID_Reset(&g_pid_left_leg_angle);
    PID_Reset(&g_pid_right_leg_angle);
    PID_Reset(&g_balance_angle_pid);
    PID_Reset(&g_balance_vel_pid);
    PID_Reset(&g_pid_roll_compensation);
}

void _target_state_update(float forward_speed, float turning_speed, float chassis_height)
{

    // g_chassis.target_yaw_speed = -remote->controller.right_stick.x / 660.0f * 12.0f;
    g_chassis.target_yaw_speed = turning_speed;
    g_chassis.target_yaw += g_chassis.target_yaw_speed * TASK_TIME;
    // g_chassis.target_vel = 0.995f * g_chassis.target_vel + 0.005f * (-remote->controller.left_stick.y / 660.0f * 2.0f);
    g_chassis.target_vel = 0.995f * g_chassis.target_vel + 0.005f * (forward_speed);
    g_lqr_left_state.target_x_dot = g_chassis.target_vel; // - g_chassis.target_yaw_speed * HALF_WHEEL_DISTANCE;
    g_lqr_left_state.target_x += g_lqr_left_state.target_x_dot * TASK_TIME;
    if (g_robot_state.spintop_mode)
    {
    g_lqr_left_state.target_x_dot += g_chassis.target_yaw_speed * HALF_WHEEL_DISTANCE;
    }
    g_lqr_right_state.target_x_dot = g_chassis.target_vel; // + g_chassis.target_yaw_speed * HALF_WHEEL_DISTANCE;
    g_lqr_right_state.target_x += g_lqr_right_state.target_x_dot * TASK_TIME;
    if (g_robot_state.spintop_mode)
    {
    g_lqr_right_state.target_x_dot -= g_chassis.target_yaw_speed * HALF_WHEEL_DISTANCE; // + g_chassis.target_yaw_speed * HALF_WHEEL_DISTANCE;
    }
    // g_robot_state.chassis_height += remote->controller.right_stick.y / 660.0f * 0.2f * TASK_TIME;
    g_robot_state.chassis_height = chassis_height;

    // turning centripetal force
    // float turning_radius = turning_speed > 0.4f ? forward_speed / turning_speed : 99.0f;
    // float centripetal_force = turning_speed > 0.4f ? 16.0f * turning_speed * turning_speed / turning_radius : 0; // m * v^2 / r
    // turning_radius = turning_radius < 0.5f ? 0.5f : turning_radius;
    // g_chassis.turning_radius = turning_radius;
    // g_chassis.centripetal_force = centripetal_force;
    // __MAX_LIMIT(g_robot_state.chassis_height, 0.1f, 0.35f);
}

void _leg_length_controller(float chassis_height, float robot_roll)
{
    float chassis_height_temp = chassis_height;
    float feedforward_weight = 90.0f;

    if (g_jump_state_machine.flag_jump_start)
    {
        feedforward_weight = 20.0f;
        g_pid_left_leg_length.kp = 5000.0f * 5.0f;
        g_pid_left_leg_length.kd = 150.0f * 5.0f;
        g_pid_right_leg_length.kp = 5000.0f * 5.0f;
        g_pid_right_leg_length.kd = 150.0f * 5.0f;
        chassis_height_temp = 0.1f;
    } else if (g_jump_state_machine.flag_retracted)
    {
        feedforward_weight = 200.0f;
        chassis_height_temp = 0.35f;
    } else if (g_jump_state_machine.flag_extended)
    {
        feedforward_weight = -220.0f;
        chassis_height_temp = 0.1f;
    } else if (g_jump_state_machine.flag_reretracted)
    {
        feedforward_weight = 80.0f;
        chassis_height_temp = 0.35f;
        g_pid_left_leg_length.kp = 5000.0f * 1.0f;
        g_pid_left_leg_length.kd = 150.0f * 1.0f;
        g_pid_right_leg_length.kp = 5000.0f * 1.0f;
        g_pid_right_leg_length.kd = 150.0f * 1.0f;
    } else if (g_jump_state_machine.flag_reextended)
    {
        // hi :)
        feedforward_weight = 60.0f;
        g_pid_left_leg_length.kp = 5000.0f * 1.0f;
        g_pid_left_leg_length.kd = 150.0f * 1.0f;
        g_pid_right_leg_length.kp = 5000.0f * 1.0f;
        g_pid_right_leg_length.kd = 150.0f * 1.0f;
    }

    if ((g_jump_state_machine.flag_jump_start) && (g_leg_left.length < 0.12f) && (g_leg_right.length < 0.12f))
    {
        g_jump_state_machine.flag_jump_start = 0;
        g_jump_state_machine.flag_retracted = 1;
    } else if (g_jump_state_machine.flag_retracted && (g_leg_left.length > 0.33f) && (g_leg_right.length > 0.33f))
    {
        g_jump_state_machine.flag_retracted = 0;
        g_jump_state_machine.flag_extended = 1;
    } else if (g_jump_state_machine.flag_extended && (g_leg_left.length < 0.16f) && (g_leg_right.length < 0.16f))
    {
        g_jump_state_machine.flag_extended = 0;
        g_jump_state_machine.flag_reretracted = 1;
    } else if (g_jump_state_machine.flag_reretracted && (g_leg_left.length > 0.2f) && (g_leg_right.length > 0.2f))
    {
        g_jump_state_machine.flag_reretracted = 0;
        g_jump_state_machine.flag_reextended = 1;
    } else if (g_jump_state_machine.flag_reextended && (g_leg_left.length < 0.12f) && (g_leg_right.length < 0.12f))
    {
        g_jump_state_machine.flag_reextended = 0;
    }

    // g_leg_left.compensatioin_torq = -g_chassis.centripetal_force * 0.5f;
    // g_leg_right.compensatioin_torq = +g_chassis.centripetal_force * 0.5f;

    g_chassis.roll_compensation_height = g_chassis.roll_compensation_height * 0.99f + 0.01f * PID(&g_pid_roll_compensation, robot_roll);
    g_leg_left.target_leg_virtual_force = PID_dt(&g_pid_left_leg_length, chassis_height_temp + g_chassis.roll_compensation_height - g_leg_left.length, TASK_TIME) + feedforward_weight;
    g_leg_right.target_leg_virtual_force = PID_dt(&g_pid_right_leg_length, chassis_height_temp - g_chassis.roll_compensation_height - g_leg_right.length, TASK_TIME) + feedforward_weight;
    // g_leg_left.target_leg_virtual_force += g_leg_left.compensatioin_torq;
    // g_leg_right.target_leg_virtual_force += g_leg_right.compensatioin_torq;
    g_leg_left.target_leg_virtual_torq = -g_u_left.T_B;
    g_leg_right.target_leg_virtual_torq = g_u_right.T_B;
}

void _lqr_balancce_controller()
{
    /* LQR */
    LQR_Output(&g_u_left, &g_lqr_left_state);
    LQR_Output(&g_u_right, &g_lqr_right_state);
    static float left_leg_angle, right_leg_angle = 0;

    /* Anti Split */
    left_leg_angle = g_leg_left.phi0;
    right_leg_angle = PI - g_leg_right.phi0;
    PID_dt(&g_pid_anti_split, left_leg_angle - right_leg_angle, TASK_TIME);

    g_u_left.T_B += g_pid_anti_split.output;
    g_u_right.T_B -= g_pid_anti_split.output;
}
void _vmc_torq_calc()
{
    Leg_VMC(&g_leg_left, 
        -g_motor_lb->stats->current * MG8016_CURRENT_INT_TO_TORQ_NM, \
        -g_motor_lf->stats->current * MG8016_CURRENT_INT_TO_TORQ_NM);
    Leg_VMC(&g_leg_right,
        -g_motor_rf->stats->current * MG8016_CURRENT_INT_TO_TORQ_NM, \
        -g_motor_rb->stats->current * MG8016_CURRENT_INT_TO_TORQ_NM);

    PID_dt(&g_pid_yaw_angle, g_chassis.current_yaw - g_chassis.target_yaw, TASK_TIME);
    g_u_left.T_A -= g_pid_yaw_angle.output;
    g_u_right.T_A += g_pid_yaw_angle.output;

    if (g_robot_state.spintop_mode)
    {
        g_u_left.T_A -= (0.1f*(-1.0f));
        g_u_right.T_A += 0.3f;
    }
}

void Chassis_Disable()
{
    _hip_motor_torq_ctrl(0.0f, 0.0f, 0.0f, 0.0f);
    _foot_motor_torq_ctrl(0.0f, 0.0f);
}
void _chassis_cmd()
{
    g_chassis.chassis_killed_by_referee = !Referee_Robot_State.Chassis_Power_Is_On;
    float ideal_angle_diff;
    // Update Target
    if (g_robot_state.wheel_facing_mode == 0)
    {
        ideal_angle_diff = DJI_Motor_Get_Total_Angle(g_yaw);
        // either facing forward or backward
        float forward_angle_diff = fmod(ideal_angle_diff, 2 * PI);
        if (forward_angle_diff > PI)
        {
            forward_angle_diff -= 2 * PI;
        }
        else if (forward_angle_diff < -PI)
        {
            forward_angle_diff += 2 * PI;
        }
        float backward_angle_diff = fmod(ideal_angle_diff + PI, 2 * PI);
        if (backward_angle_diff > PI)
        {
            backward_angle_diff -= 2 * PI;
        }
        else if (backward_angle_diff < -PI)
        {
            backward_angle_diff += 2 * PI;
        }
        if (fabs(forward_angle_diff) < fabs(backward_angle_diff))
        {
            ideal_angle_diff = forward_angle_diff;
            g_chassis.forward_speed = g_robot_state.vy;
        }
        else
        {
            ideal_angle_diff = backward_angle_diff;
            g_chassis.forward_speed = -g_robot_state.vy;
        }
    }
    else
    {
        ideal_angle_diff = DJI_Motor_Get_Total_Angle(g_yaw);
        // either facing forward or backward
        float forward_angle_diff = fmod(ideal_angle_diff - PI / 2, 2 * PI);
        if (forward_angle_diff > PI)
        {
            forward_angle_diff -= 2 * PI;
        }
        else if (forward_angle_diff < -PI)
        {
            forward_angle_diff += 2 * PI;
        }
        float backward_angle_diff = fmod(ideal_angle_diff + PI / 2, 2 * PI);
        while (backward_angle_diff > PI)
        {
            backward_angle_diff -= 2 * PI;
        }
        while (backward_angle_diff < -PI)
        {
            backward_angle_diff += 2 * PI;
        }
        if (fabs(forward_angle_diff) < fabs(backward_angle_diff))
        {
            ideal_angle_diff = forward_angle_diff;
            g_chassis.forward_speed = g_robot_state.vx;
        }
        else
        {
            ideal_angle_diff = backward_angle_diff;
            g_chassis.forward_speed = -g_robot_state.vx;
        }
    }
    // g_chassis.angle_diff = g_chassis.angle_diff * 0.99f + 0.01f * ideal_angle_diff;
    g_chassis.angle_diff = ideal_angle_diff;
        // Spintop vs Follow Gimbal
    if (g_robot_state.spintop_mode)
    {
        if (g_remote.controller.left_switch == MID)
        {
            g_chassis.target_yaw_speed = g_chassis.target_yaw_speed * 0.9f + 0.1f * 4.0f;
        } else {
            g_chassis.target_yaw_speed = g_chassis.target_yaw_speed * 0.9f + 0.1f * -4.0f;
        }
    }
    else if (g_robot_state.gimbal_switching_dir_pending == 1)
    {
        g_chassis.target_yaw_speed = 0 * 0.1f + 0.9f * g_chassis.target_yaw_speed;
    }
    else
    {
        g_chassis.target_yaw_speed = g_chassis.target_yaw_speed * 0.95f + 0.05f * PID_dt(&g_pid_follow_gimbal, g_chassis.angle_diff, TASK_TIME);
    }

    // this interfere with shooting//g_robot_state.chassis_height += g_remote.controller.wheel / 660.0f * 0.003f;
    __MAX_LIMIT(g_robot_state.chassis_height, 0.1f, 0.39f);
}

void Chassis_Ctrl_Loop()
{

    _chassis_cmd();
    _wheel_leg_estimation(g_board_comm_package.yaw, g_board_comm_package.robot_pitch, g_board_comm_package.robot_pitch_rate);
    _target_state_update(g_chassis.forward_speed, g_chassis.target_yaw_speed, g_robot_state.chassis_height);
    _leg_length_controller(g_robot_state.chassis_height, g_board_comm_package.robot_roll);
    _lqr_balancce_controller();
    _vmc_torq_calc();
    if (last_spintop_mode == 1 && g_robot_state.spintop_mode == 0)
    {
        _target_state_reset();
    }
    // if ((g_robot_state.enabled) && (!g_chassis.chassis_killed_by_referee)) // add wheel offline detection
    if ((g_robot_state.enabled))// add wheel offline detection
    {
        _hip_motor_torq_ctrl(-g_leg_left.torq4, -g_leg_left.torq1, -g_leg_right.torq4, -g_leg_right.torq1);
        _foot_motor_torq_ctrl(-g_u_left.T_A, g_u_right.T_A);
    }
    else
    {
        Chassis_Disable();
        _target_state_reset();
    }
}