//
// Created by lly on 2024/4/20.
//
#include "main.h"
#include <stdbool.h>
#include "tim.h"
#include "spi.h"
#include "Encoder.h"
#include "Motor_Control.h"
#include "Foc.h"
#include "usbd_cdc_if.h"
extern float vofa_trans[16];
void Encoder_Init()
{
    HAL_TIM_Encoder_Start(&Timer_Encoder,TIM_CHANNEL_ALL);
    Motor.elec_rad_per_enc_abz=pole_pares * 2 * M_PI_ * (1.0f / Cpr);
    Motor.elec_rad_per_enc_spi=pole_pares * 2 * M_PI_ * (1.0f / Spi_cpr);
}

void Encoder_calibration(float* Vq, float* Vd, float* Theta)
{
    *Vq=0.f;
    *Vd=0.08f;
    *Theta=0;
    HAL_Delay(2000);
    switch (Motor.Encoder_Mode) {
        case EncMode_ABZ:{
            Timer_Encoder.Instance->CNT=0;
        }break;

        case EncMode_SPI:{
            Motor.Encoder_Offset_Spi=Motor.Encoder_Theta_analog;
        }break;
    }
    *Vq=0;
    *Vd=0;
}

void Encoder_ABZ_Get_Angle(int32_t* analog_Theta_Value)
{
    * analog_Theta_Value=Timer_Encoder.Instance->CNT;
}

///16bit low 2edge
void Encoder_SPI_Get_Angle_AS5047P(int32_t* analog_Theta_Value)
{
    int32_t pre_angle=0;
    int16_t command = 0x3FFF;
    CS_ENC(0);																			//cs拉低
    HAL_SPI_TransmitReceive(&Spi_Encoder,(uint8_t *)&command,(uint8_t *)&pre_angle,1,0xffff);
    CS_ENC(1);																			//cs拉高
    * analog_Theta_Value=pre_angle&0X3FFF;
}

///8bit high 2edge
void Encoder_SPI_Get_Angle_MT6816(int32_t* analog_Theta_Value)
{
    uint8_t TxData1  = 0x83;
    uint8_t TxData2  = 0x84;
    uint8_t u8Data11 = 0x00;
    uint8_t u8Data12 = 0x00;
    uint16_t sample_data;
    bool pc_flag;
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&Spi_Encoder, &TxData1, 1, 1);
    HAL_SPI_TransmitReceive(&Spi_Encoder, &TxData2, &u8Data11, 1, 1);
    HAL_SPI_TransmitReceive(&Spi_Encoder, &TxData1, &u8Data12, 1, 1);
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);

    sample_data = ((u8Data11 & 0x00FF) << 8) | (u8Data12 & 0x00FF);
    uint8_t h_count = 0;
    for (uint8_t j = 0; j < 16; j++) {
        if (sample_data & (0x0001 << j))
            h_count++;
    }
    if (h_count & 0x01) {
        pc_flag = false;
    } else {
        pc_flag = true;
    }

    if (pc_flag) {
        * analog_Theta_Value=(sample_data >> 2);  //前14bits 角度数据
    }

}

void Encoder_Get_Angle_analog(int32_t* analog_Theta_Value)
{
    switch (Motor.Encoder_Mode) {
        case EncMode_ABZ: {
            Encoder_ABZ_Get_Angle(analog_Theta_Value);
        }break;

        case EncMode_SPI: {
            switch (Motor.Encoder_type) {
                case Encoder_Type_As5047p: {
                    Encoder_SPI_Get_Angle_AS5047P(analog_Theta_Value);
                }break;

                case Encoder_Type_MT6816:{
                    Encoder_SPI_Get_Angle_MT6816(analog_Theta_Value);
                }break;
            }
        }
    }
}

void Encoder_Get_Rle_Angle(int32_t analog_Theta_Value,float* Ele_Angle)
{

    switch (Motor.Encoder_Mode) {
        case EncMode_ABZ:{
            *Ele_Angle= (float)(Motor.Encoder_Theta_analog) * Motor.elec_rad_per_enc_abz;
        }break;

        case EncMode_SPI:{
            if (analog_Theta_Value - Motor.Encoder_Offset_Spi >= 0)
            {
                *Ele_Angle= (float)(Motor.Encoder_Theta_analog-Motor.Encoder_Offset_Spi) * Motor.elec_rad_per_enc_spi;
            }

            else
            {
                *Ele_Angle=(float)(Spi_cpr+Motor.Encoder_Theta_analog-Motor.Encoder_Offset_Spi) * Motor.elec_rad_per_enc_spi;
            }
        }break;
    }
}



