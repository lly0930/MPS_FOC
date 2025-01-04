//
// Created by 29787 on 2024/4/20.
//

#ifndef PROJECT_PID_H
#define PROJECT_PID_H

#define V_to_mod               0.0625f               ///1.0f / ((2.0f / 3.0f) * 24.f)

#define Vq_limit          0.9f
#define Vd_limit          0.05f
#define Vel_total_limit   1.8
#define Pos_total_limit   4.5
#define Vel_intergra_limit 10;

typedef struct pid_{
    float Cur_Kp;
    float Cur_Ki;
    float Cur_proportion_q;
    float Cur_proportion_d;
    float Cur_intergration_q;
    float Cur_intergration_d;


    float Vel_Kp;
    float Vel_Ki;
    float Vel_Kd;
    float Vel_proportion;
    float Vel_intergration;
    float Vel_differential;

    float Pos_Kp;
    float Pos_Ki;
    float Pos_Kd;
    float Pos_proportion;
    float Pos_intergration;
    float Pos_differential;
}Pid_;

void Pid_Current(float Tar_q, float Tar_d, float Cur_q, float Cur_d, float* Out_Vq, float* Out_Vd);
void Pid_Velocity(float Tar_Vel, float Cur_Vel, float* Out_Cur);
void Pid_Position(float Tar_Pos, float Cur_Pos, float* Out_Cur);

extern Pid_ Pid;

#endif //PROJECT_PID_H
