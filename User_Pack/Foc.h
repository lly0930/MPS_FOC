//
// Created by lly on 2024/4/19.
//

#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#define Timer_gate_drive     htim1

#define one_by_sqrt3  0.57735026919F
#define two_by_sqrt3  1.15470053838F
#define sqrt3_by_2    0.86602540378f
#define sqrt3         1.732050807568877F
#define M_PI_         3.1415926535f

typedef struct Foc_ {
    float Theta;
    float Vq;
    float Vd;
    float Valpha;
    float Vbeta;
    float Udc;
    uint32_t Tpwm;
    float CCRs[3];
} Foc;

void foc_Init();

void foc_Revpark(float Uq, float Ud, float Theta, float *Ualpha, float *Ubeta);

void foc_Park(float Ialpha, float Ibete, float Theta, float *Iq, float *Id);

void foc_Clark(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);

void svpwm(float Valpha, float Vbeta, float Udc, float CCRS[]);

void foc_SetCCR(float CCRs[]);

extern Foc foc;
#endif //PROJECT_FOC_H
