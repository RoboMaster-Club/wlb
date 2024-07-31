#include "five_bar_leg.h"
#include "robot_param.h"
#include "arm_math.h"

void Leg_ForwardKinematics(Leg_t *leg, float phi1, float phi4, float phi1_dot, float phi4_dot)
{
    leg->current_tick = SysTick->VAL;
    leg->phi1 = phi1;
    leg->phi1_dot = phi1_dot;
    leg->phi4 = phi4;
    leg->phi4_dot = phi4_dot;

    float x_B = -HALF_THIGH_DISTANCE + THIGH_LENGTH * arm_cos_f32(leg->phi1);
    float y_B = THIGH_LENGTH * arm_sin_f32(leg->phi1);
    float x_D = HALF_THIGH_DISTANCE + THIGH_LENGTH * arm_cos_f32(leg->phi4);
    float y_D = THIGH_LENGTH * arm_sin_f32(leg->phi4);

    float xD_minus_xB = x_D - x_B;
    float yD_minus_yB = y_D - y_B;

    float A = 2 * CALF_LENGTH * xD_minus_xB;
    float B = 2 * CALF_LENGTH * yD_minus_yB;
    float C = pow(xD_minus_xB, 2) + pow(yD_minus_yB, 2);

    arm_atan2_f32(B + sqrt(pow(A, 2) + pow(B, 2) - pow(C, 2)), A + C, &leg->phi2);
    leg->phi2 = leg->phi2 * 2;
    float x_C = x_B + CALF_LENGTH * arm_cos_f32(leg->phi2);
    float y_C = y_B + CALF_LENGTH * arm_sin_f32(leg->phi2);

    arm_atan2_f32(y_C - y_D, x_C - x_D, &leg->phi3);

    leg->xe1 = xD_minus_xB;
    leg->ye1 = yD_minus_yB;
    leg->xe2 = x_D;
    leg->ye2 = y_D;
    leg->length = sqrt(pow(x_C, 2) + pow(y_C, 2));
    leg->last_phi0 = leg->phi0;
    arm_atan2_f32(y_C, x_C, & leg->phi0);
#pragma message "jidegaihuiqv"
    leg->phi0_dot = (leg->phi0 - leg->last_phi0) / (0.002f);

    leg->last_tick = leg->current_tick;
}

void Leg_InverseKinematics(float height, float leg_angle, float *leg_1, float *leg_2)
{
    float x_toe = height / (tan(leg_angle) == 0 ? 0.000001f : tan(leg_angle));
    float y_toe = height;

    float a_1 = 2 * (-HALF_THIGH_DISTANCE - x_toe) * THIGH_LENGTH;
    float b = -2 * y_toe * THIGH_LENGTH;
    float c_1 = pow(-HALF_THIGH_DISTANCE - x_toe, 2) + pow(y_toe, 2) + pow(THIGH_LENGTH, 2) - pow(CALF_LENGTH, 2);

    x_toe = -x_toe;
    float a_2 = 2 * (-HALF_THIGH_DISTANCE - x_toe) * THIGH_LENGTH;
    float c_2 = pow(-HALF_THIGH_DISTANCE - x_toe, 2) + pow(y_toe, 2) + pow(THIGH_LENGTH, 2) - pow(CALF_LENGTH, 2);
    arm_atan2_f32(-b + sqrt(pow(a_1, 2) + pow(b, 2) - pow(c_1, 2)), c_1 - a_1, leg_1);
    *leg_1 = 2 * (*leg_1);
    arm_atan2_f32(-b + sqrt(pow(a_2, 2) + pow(b, 2) - pow(c_2, 2)), c_2 - a_2, leg_2);
    *leg_2 = -(2 * (*leg_2) - PI);
}

void Leg_VMC(Leg_t *leg, float torq1, float torq4)
{
    // float leg_length = leg->length;
    // float theta = leg->phi0 - PI / 2;
    // float phi1 = leg->phi1;
    // float phi4 = leg->phi4;

    // float sin_phi1 = arm_sin_f32(phi1);
    // float cos_phi1 = arm_cos_f32(phi1);
    // float sin_phi4 = arm_sin_f32(phi4);
    // float cos_phi4 = arm_cos_f32(phi4);
    // float sin_theta = arm_sin_f32(theta);
    // float cos_theta = arm_cos_f32(theta);

    // float xC_minus_xB = (-leg_length * sin_theta) - (-HALF_THIGH_DISTANCE + THIGH_LENGTH * cos_phi1);
    // float yC_minus_yB = (leg_length * cos_theta) - THIGH_LENGTH * sin_phi1;
    // float xC_minus_xD = (-leg_length * sin_theta) - (HALF_THIGH_DISTANCE + THIGH_LENGTH * cos_phi4);

    // float yC_minus_yD = (leg_length * cos_theta) - THIGH_LENGTH * sin_phi4;

    // float M11 = ((-sin_theta * xC_minus_xB + cos_theta * yC_minus_yB) /
    //              (-sin_phi1 * xC_minus_xB + cos_phi1 * yC_minus_yB)) /
    //             THIGH_LENGTH;
    // float M12 = (leg_length / THIGH_LENGTH) * ((-cos_theta * xC_minus_xB - sin_theta * yC_minus_yB) /
    //                                            (-sin_phi1 * xC_minus_xB + cos_phi1 * yC_minus_yB));
    // float M21 = ((-sin_theta * xC_minus_xD + cos_theta * yC_minus_yD) /
    //              (-sin_phi4 * xC_minus_xD + cos_phi4 * yC_minus_yD)) /
    //             THIGH_LENGTH;
    // float M22 = (leg_length / THIGH_LENGTH) * ((-cos_theta * xC_minus_xD - sin_theta * yC_minus_yD) /
    //                                            (-sin_phi4 * xC_minus_xD + cos_phi4 * yC_minus_yD));

    // float one_over_deter = 1 / (M11 * M22 - M12 * M21);
    // float J11 = one_over_deter * M22;
    // float J12 = -one_over_deter * M12;
    // float J21 = -one_over_deter * M21;
    // float J22 = one_over_deter * M11;

    // leg->torq1 = J11 * leg->target_leg_virtual_force + J21 * leg->target_leg_virtual_torq;
    // leg->torq4 = J12 * leg->target_leg_virtual_force + J22 * leg->target_leg_virtual_torq;
    float phi0 = leg->phi0;
    float phi3 = leg->phi3;
    float phi2 = leg->phi2;
    float phi4 = leg->phi4;
    float phi1 = leg->phi1;

    float j11 = (THIGH_LENGTH*arm_sin_f32(phi0-phi3)*arm_sin_f32(phi1-phi2))/arm_sin_f32(phi3-phi2);
    float j12 = (THIGH_LENGTH*arm_cos_f32(phi0-phi3)*arm_sin_f32(phi1-phi2))/leg->length*arm_sin_f32(phi3-phi2);
    float j21 = (THIGH_LENGTH*arm_sin_f32(phi0-phi2)*arm_sin_f32(phi3-phi4))/arm_sin_f32(phi3-phi2);
    float j22 = (THIGH_LENGTH*arm_cos_f32(phi0-phi2)*arm_sin_f32(phi3-phi4))/leg->length*arm_sin_f32(phi3-phi2);

    leg->torq1 = j11 * leg->target_leg_virtual_force + j12 * leg->target_leg_virtual_torq;
    leg->torq4 = j21 * leg->target_leg_virtual_force + j22 * leg->target_leg_virtual_torq;

    float det = j11 * j22 - j12 * j21;
    float j11_inv = j22 / det;
    float j12_inv = -j12 / det;
    float j21_inv = -j21 / det;
    float j22_inv = j11 / det;
    leg->normal_force = leg->normal_force * 0.95f + 0.05f * (j11_inv * torq1 + j12_inv * torq4); // normal force calculated using the actual torq current
    leg->actual_virtual_torq = leg->actual_virtual_torq * 0.95f + 0.05f * (j21_inv * torq1 + j22_inv * torq4); // virtual torq calculated using the actual torq current
}