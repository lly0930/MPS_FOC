//
// Created by lly on 2024/4/20.
//
#include "Adc_Sample.h"
#include "adc.h"

float Cur_V_to_R=0;
void Adc_Init()
{
    Cur_V_to_R=(3.3f / 4095.f) / gain / Resistance;
    HAL_ADCEx_Calibration_Start( &hadc1, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_EOC);
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    HAL_ADCEx_Calibration_Start( &hadc2, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG( &hadc2, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG( &hadc2, ADC_FLAG_EOC);
    HAL_ADCEx_InjectedStart_IT(&hadc2);


    HAL_Delay(100);
}


void Adc_calibrate_Offsets(float Offsets[])
{
    const int calibration_rounds = 100;

    for (int i = 0; i < calibration_rounds; i++) {
        Offsets[0] += AD_CuA;
        Offsets[1] += AD_CuB;
        Offsets[2] += AD_CuC;
        HAL_Delay(1);
    }

    Offsets[0]=Offsets[0]/(float)calibration_rounds;
    Offsets[1]=Offsets[1]/(float)calibration_rounds;
    Offsets[2]=Offsets[2]/(float)calibration_rounds;
}

void Adc_Get_Current_Vbus(float Currents[],  float Offsets[], float* Vbus)
{
    Currents[0]  = (float)((AD_CuA-Offsets[0])*Cur_V_to_R);
    Currents[1]  = (float)((AD_CuB-Offsets[1])*Cur_V_to_R);
    Currents[2]  = (float)((AD_CuC-Offsets[2])*Cur_V_to_R);
//    Currents[1]  = -(Currents[0]+Currents[2]);
    *Vbus=(float)(V_BUS*Vbus_V_to_R);
    *Vbus=(float)24;
}

