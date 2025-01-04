//
// Created by lly on 2024/4/20.
//

#ifndef PROJECT_ADC_SAMPLE_H
#define PROJECT_ADC_SAMPLE_H

#define AD_CuA      ADC1->JDR1
#define AD_CuB      ADC1->JDR2
#define AD_CuC      ADC1->JDR3
#define V_BUS       ADC2->JDR1

#define gain  20
#define Resistance 0.0005f

#define Vbus_V_to_R 0.012890625f


void Adc_Init();
void Adc_calibrate_Offsets(float Offsets[]);
void Adc_Get_Current_Vbus(float Currents[], float Offsets[], float* Vbus);

#endif //PROJECT_ADC_SAMPLE_H
