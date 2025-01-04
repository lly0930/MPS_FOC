//
// Created by 29787 on 2024/4/20.
//

#ifndef PROJECT_MOTOR_CONTROL_H
#define PROJECT_MOTOR_CONTROL_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define Control_Mode_Vel   1
#define Control_Mode_Pos   2
#define Control_Mode_Cur   3
#define Control_Mode_Null  4

#define current_mea_period 0.000040F
#define direction          1

typedef struct motor_{

    uint8_t Control_Mode;

    bool Motor_is_Calibrate;

    float   Abc_Offstes[3];
    float   Abc_Current[3];
    float   AlphaBete_Current[2];
    float   Qd_Current[2];
    float   Tar_Qd_Current[2];

    float   Tar_Vel;
    float   Cur_Vel;
    float   Tar_Pos;
    float   Cur_Pos;

    float   elec_rad_per_enc_abz;
    float   elec_rad_per_enc_spi;
    int8_t  Encoder_Mode;
    int8_t  Encoder_type;
    int32_t Encoder_Offset_Spi;
    int32_t Encoder_Theta_analog;
}Motor_;
extern Motor_ Motor;

void Motor_Init();
void Motor_Calibrate();
void Motor_Measure_Velocity(int32_t Encoder_Theta_analog, float* Motor_Volcity, float* Motor_Position);

inline float wrap_pm(float x, float y) {
    float intval = nearbyintf(x / y);
    return x - intval * y;
}

inline float fmodf_pos(float x, float y) {
    float res = wrap_pm(x, y);
    if (res < 0) res += y;
    return res;
}
#endif //PROJECT_MOTOR_CONTROL_H
