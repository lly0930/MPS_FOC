//
// Created by lly on 2024/4/19.
//
#include "main.h"
#include "tim.h"
#include "Foc.h"
#include "arm_math.h"
#include "usbd_cdc_if.h"
#include "Motor_Control.h"

Foc foc = {
        .Tpwm=3400,
        .Udc=24,
};

void foc_Init() {
    HAL_TIM_Base_Start_IT(&Timer_gate_drive);

    HAL_TIM_PWM_Start(&Timer_gate_drive, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&Timer_gate_drive, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&Timer_gate_drive, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&Timer_gate_drive, TIM_CHANNEL_4);
    Timer_gate_drive.Instance->CCR4 = foc.Tpwm-10;

    HAL_TIMEx_PWMN_Start(&Timer_gate_drive, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&Timer_gate_drive, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&Timer_gate_drive, TIM_CHANNEL_3);
}

void foc_Park(float Ialpha, float Ibete, float Theta, float *Iq, float *Id) {
    float c_I = arm_cos_f32(Theta);
    float s_I = arm_sin_f32(Theta);

    *Iq = -(-c_I * Ibete + s_I * Ialpha) * (direction);
    *Id = -(-c_I * Ialpha - s_I * Ibete) * (direction);
}

void foc_Clark(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta) {
//    temp_clark_alpha=Foc.currentA;
//    temp_clark_beta =one_by_sqrt3*(Foc.currentB-Foc.currentC);
    *Ialpha = Ia;
    *Ibeta = one_by_sqrt3 * (Ib- Ic);
}

void foc_Revpark(float Vq, float Vd, float Theta, float *Valpha, float *Vbeta) {
    float cosTheta = arm_cos_f32(Theta);
    float sinTheta = arm_sin_f32(Theta);

    *Valpha = (Vd * cosTheta - Vq * sinTheta);
    *Vbeta = (Vd * sinTheta + Vq * cosTheta);
}

void svpwm(float alpha, float beta, float Udc, float CCRS[]) {
    float tA, tB, tC;
    uint8_t Sextant;
    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2
        } else {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t1 - t2) * 0.5f;
            tB = tA + t1;
            tC = tB + t2;
        }
            break;

            // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t2 - t3) * 0.5f;
            tA = tB + t3;
            tC = tA + t2;
        }
            break;

            // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t3 - t4) * 0.5f;
            tC = tB + t3;
            tA = tC + t4;
        }
            break;

            // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t4 - t5) * 0.5f;
            tB = tC + t5;
            tA = tB + t4;
        }
            break;

            // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t5 - t6) * 0.5f;
            tA = tC + t5;
            tB = tA + t6;
        }
            break;

            // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t6 - t1) * 0.5f;
            tC = tA + t1;
            tB = tC + t6;
        }
            break;
    }

    if ((tA >= 0.0f && tA <= 1.0f)
        && (tB >= 0.0f && tB <= 1.0f)
        && (tC >= 0.0f && tC <= 1.0f)
            ) {
        CCRS[0] = (tA * (float) (foc.Tpwm));
        CCRS[1] = (tB * (float) (foc.Tpwm));
        CCRS[2] = (tC * (float) (foc.Tpwm));
    }
}

void   foc_SetCCR(float CCRs[]) {
    CCRs[0] = CCRs[0] > foc.Tpwm ? foc.Tpwm : CCRs[0];
    CCRs[0] = CCRs[0] < 0 ? 0 : CCRs[0];
    CCRs[1] = CCRs[1] > foc.Tpwm ? foc.Tpwm : CCRs[1];
    CCRs[1] = CCRs[1] < 0 ? 0 : CCRs[1];
    CCRs[2] = CCRs[2] > foc.Tpwm ? foc.Tpwm : CCRs[2];
    CCRs[2] = CCRs[2] < 0 ? 0 : CCRs[2];
    Timer_gate_drive.Instance->CCR1 = (uint32_t) CCRs[0];
    Timer_gate_drive.Instance->CCR2 = (uint32_t) CCRs[2];
    Timer_gate_drive.Instance->CCR3 = (uint32_t) CCRs[1];
}




