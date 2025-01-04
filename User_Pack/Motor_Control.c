//
// Created by 29787 on 2024/4/20.
//
#include "usbd_cdc_if.h"
#include "Motor_Control.h"
#include "Encoder.h"
#include "Foc.h"
#include "Adc_Sample.h"
#include "gpio.h"
#include "CAN_communicate.h"

extern float vofa_trans[16];

Motor_ Motor={
   .Control_Mode=Control_Mode_Pos,
   .Encoder_Mode=EncMode_ABZ,
   .Encoder_type=Encoder_Type_MT6816,
   .Motor_is_Calibrate=false,
};


void Motor_Init()
{
    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, 1);
    Encoder_Init();
    foc_Init();
    Adc_Init();
}

void Motor_Calibrate()
{
    Adc_calibrate_Offsets(Motor.Abc_Offstes);
    HAL_Delay(200);
    Encoder_calibration(&foc.Vq, &foc.Vd, &foc.Theta);
    HAL_Delay(1000);
    Motor.Motor_is_Calibrate=true;
}

void Motor_Measure_Velocity(int32_t Encoder_Theta_analog, float* Motor_Volcity, float* Motor_Position)
{
    static float   pos_estimate_counts=0;
    static float   pos_cpr_counts_=0;
    static float   vel_estimate_counts=0;
    static float   delta_pos_counts=0;
    static float   last_delta_pos_counts=0;
    static float   delta_pos_cpr_counts=0;
    static float   total_position=0;
    static float   pll_kp_= 100.f;
    float          pll_ki_= 0.25f * (pll_kp_ * pll_kp_);

    last_delta_pos_counts= pos_cpr_counts_;
    pos_estimate_counts  += current_mea_period * vel_estimate_counts;
    pos_cpr_counts_      += current_mea_period * vel_estimate_counts;

    delta_pos_counts     = (float)(Encoder_Theta_analog - (int32_t)floorf(pos_estimate_counts));
    delta_pos_cpr_counts = (float)(Encoder_Theta_analog - (int32_t)floorf(pos_cpr_counts_));

    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float) (Spi_cpr));
//    if(Motor.Encoder_Mode==EncMode_ABZ) {
//        delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float) (Cpr));
//    }

    ///pll feedback
    pos_estimate_counts += current_mea_period * pll_kp_ * delta_pos_counts / (float)Spi_cpr;      ///在一圈内的位置
//    if(Motor.Encoder_Mode==EncMode_ABZ) {
//        pos_estimate_counts += current_mea_period * pll_kp_ * delta_pos_counts / (float)Cpr;      ///在一圈内的位置
//    }


    vel_estimate_counts += current_mea_period * pll_ki_ * delta_pos_cpr_counts;
    pos_cpr_counts_+=current_mea_period*pll_kp_*delta_pos_cpr_counts;
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(Spi_cpr));
//    if(Motor.Encoder_Mode==EncMode_ABZ) {
//        pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(Cpr));
//    }
    total_position+= wrap_pm((pos_cpr_counts_-last_delta_pos_counts)/Spi_cpr,1.0f);
//    if(Motor.Encoder_Mode==EncMode_ABZ) {
//        total_position+= wrap_pm((pos_cpr_counts_-last_delta_pos_counts)/Cpr,1.0f);
//    }
    * Motor_Position = total_position;
    * Motor_Volcity  = vel_estimate_counts / (float)Spi_cpr;
//    if(Motor.Encoder_Mode==EncMode_ABZ) {
//        * Motor_Volcity  = vel_estimate_counts / (float)Cpr;
//    }
                                                              ///TODO: 调了很长时间，静止速度都不是0，最终发现转换在这里进行是最好的，
                                                              ///      之前在pll里面计算的时候，输出的速度一直是个很大的不为0的值
                                                              ///      很长时间才发现是在每次计算的时候，归0过程，delta小，除完之后都变成0了
                                                              ///      只有在delta大的时候才有数，所以也就造成了，会记录上升而不会记录归0过程
}