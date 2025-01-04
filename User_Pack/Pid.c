//
// Created by 29787 on 2024/4/20.
//
#include "main.h"
#include "Pid.h"
#include "arm_math.h"
#include "Foc.h"
#include "Motor_Control.h"

//float p_gain = m_config_.current_control_bandwidth * inductance;
//float plant_pole = resistance / inductance;
//pi_gains = {p_gain,plant_pole*p_gain};
//
//p=1000*25.8*0.001=25.8
//i=p*20/25.8*1000=1000
Pid_ Pid={
    .Cur_Kp=0.2f,
    .Cur_Ki=500.f,

    .Vel_Kp=0.8f,
    .Vel_Ki=5.f,
    .Vel_Kd=500.F,

    .Pos_Kp=5,
    .Pos_Ki=0,
    .Pos_Kd=0,
};

void Pid_Current(float Tar_q, float Tar_d, float Cur_q, float Cur_d, float* Out_Vq, float* Out_Vd)
{
    float Ierr_q = Tar_q - Cur_q;
    float Ierr_d = Tar_d - Cur_d;

    Pid.Cur_proportion_q=Ierr_q*Pid.Cur_Kp;
    Pid.Cur_proportion_d=Ierr_d*Pid.Cur_Kp;
    float mod_q=V_to_mod *(Pid.Cur_proportion_q+ Pid.Cur_intergration_q);
    float mod_d=V_to_mod *(Pid.Cur_proportion_d+ Pid.Cur_intergration_d);

    if(Tar_q==0)
    {
        Pid.Cur_intergration_q=0;
        Pid.Cur_intergration_d=0;
    }

    float result;
    arm_sqrt_f32(mod_d * mod_d + mod_q * mod_q,&result);
    float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / result ;        ///TODO:这个是决定q轴输出大小的,乘可以增大最大力矩输出

    if (mod_scalefactor < 1.0f) {                                       ///dq太大了，进行限幅
        mod_q *= mod_scalefactor;
        mod_d *= mod_scalefactor;
        Pid.Cur_intergration_q*=0.99f;
        Pid.Cur_intergration_d*=0.99f;
    }

    else {
        Pid.Cur_intergration_q+=Ierr_q*Pid.Cur_Ki*current_mea_period;
        Pid.Cur_intergration_d+=Ierr_d*Pid.Cur_Ki*current_mea_period;
    }

//    *Out_Vq = mod_q >=  Vq_limit  ?    Vq_limit : mod_q  ;
//    *Out_Vq = mod_q <= (-Vq_limit)  ?   (-Vq_limit) : mod_q  ;
//    *Out_Vd = mod_d >=  Vd_limit  ?    Vd_limit : mod_d  ;
//    *Out_Vd = mod_d <= (-Vd_limit)  ?   (-Vd_limit) : mod_d  ;



    if (mod_q > Vq_limit) {
        *Out_Vq = Vq_limit;
    } else if (mod_q < -Vq_limit) {
        *Out_Vq = -Vq_limit;
    } else {
        *Out_Vq = mod_q;
    }

// 限制 Out_Vd 的值在 [-Vd_limit, Vd_limit] 范围内
    if (mod_d > Vd_limit) {
        *Out_Vd = Vd_limit;
    } else if (mod_d < -Vd_limit) {
        *Out_Vd = -Vd_limit;
    } else {
        *Out_Vd = mod_d;
    }
}

void Pid_Velocity(float Tar_Vel, float Cur_Vel, float* Out_Cur)
{
    static float last_error=0;
    float Vel_err=Tar_Vel-Cur_Vel;

    Pid.Vel_proportion=Pid.Vel_Kp*Vel_err;
    Pid.Vel_differential=Pid.Vel_Kd*(Vel_err-last_error);    last_error=Vel_err;

    Pid.Vel_intergration = Pid.Vel_intergration >  Vel_total_limit ?  Vel_total_limit : Pid.Vel_intergration;
    Pid.Vel_intergration = Pid.Vel_intergration < -Vel_total_limit ? -Vel_total_limit : Pid.Vel_intergration;

    *Out_Cur=Pid.Vel_proportion+Pid.Vel_intergration+Pid.Vel_differential;

    *Out_Cur = *Out_Cur >  Vel_total_limit ?  Vel_total_limit : *Out_Cur;
    *Out_Cur = *Out_Cur < -Vel_total_limit ? -Vel_total_limit : *Out_Cur;

    bool limit=false;
    if(*Out_Cur >= Vel_total_limit || *Out_Cur <= -Vel_total_limit) limit = true;

    if(limit) Pid.Vel_intergration*=0.99f;
    else      Pid.Vel_intergration+=current_mea_period*Vel_err*Pid.Vel_Ki;
}


void Pid_Position(float Tar_Pos, float Cur_Pos, float* Out_Cur)
{
    static float last_error=0;
    float Pos_err=Tar_Pos-Cur_Pos;

    Pid.Pos_proportion=Pid.Pos_Kp*Pos_err;
    Pid.Pos_differential=Pid.Pos_Kd*(Pos_err-last_error);    last_error=Pos_err;
    *Out_Cur=Pid.Pos_proportion+Pid.Pos_intergration+Pid.Pos_differential;

    *Out_Cur = *Out_Cur >  Pos_total_limit ?  Pos_total_limit : *Out_Cur;
    *Out_Cur = *Out_Cur < -Pos_total_limit ? -Pos_total_limit : *Out_Cur;

//    bool limit = false;
//    if(*Out_Cur >= Pos_total_limit || *Out_Cur <= -Pos_total_limit) limit = true;
//
//    if(limit) Pid.Pos_intergration*=0.99f;
//    else
//      if(Pos_err<=2)
        Pid.Pos_intergration+=current_mea_period*Pos_err*Pid.Pos_Ki;
        Pid.Pos_intergration = Pid.Pos_intergration >  Pos_total_limit ?  Pos_total_limit : Pid.Pos_intergration;
        Pid.Pos_intergration = Pid.Pos_intergration < -Pos_total_limit ? -Pos_total_limit : Pid.Pos_intergration;
//      else
//          Pid.Pos_intergration=0;
}
