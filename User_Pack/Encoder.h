//
// Created by lly on 2024/4/20.
//
#ifndef PROJECT_ENCODER_H
#define PROJECT_ENCODER_H
#include <stdint.h>

#define Timer_Encoder  htim3
#define Spi_Encoder    hspi1
#define CS_ENC(X)    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,X)

#define Cpr               4096
#define Spi_cpr           4096
#define pole_pares        14
#define EncMode_ABZ       1
#define EncMode_SPI       2
#define Encoder_Type_As5047p 1
#define Encoder_Type_MT6816  2



void Encoder_Init();
void Encoder_calibration(float* Vq, float* Vd, float* Theta);
void Encoder_ABZ_Get_Angle(int32_t* analog_Theta_Value);
void Encoder_SPI_Get_Angle_AS5047P(int32_t* analog_Theta_Value);
void Encoder_SPI_Get_Angle_MT6816(int32_t* analog_Theta_Value);
void Encoder_Get_Angle_analog(int32_t* analog_Theta_Value);
void Encoder_Get_Rle_Angle(int32_t analog_Theta_Value,float* Ele_Angle);

#endif //PROJECT_ENCODER_H
