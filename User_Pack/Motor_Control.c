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
    pos_estimate_counts += current_mea_period * pll_kp_ * delta_pos_counts / (float)Spi_cpr;      ///��һȦ�ڵ�λ��
//    if(Motor.Encoder_Mode==EncMode_ABZ) {
//        pos_estimate_counts += current_mea_period * pll_kp_ * delta_pos_counts / (float)Cpr;      ///��һȦ�ڵ�λ��
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
                                                              ///TODO: ���˺ܳ�ʱ�䣬��ֹ�ٶȶ�����0�����շ���ת���������������õģ�
                                                              ///      ֮ǰ��pll��������ʱ��������ٶ�һֱ�Ǹ��ܴ�Ĳ�Ϊ0��ֵ
                                                              ///      �ܳ�ʱ��ŷ�������ÿ�μ����ʱ�򣬹�0���̣�deltaС������֮�󶼱��0��
                                                              ///      ֻ����delta���ʱ�������������Ҳ������ˣ����¼�����������¼��0����
}