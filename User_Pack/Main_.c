
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "Main.h"
#include "Motor_Control.h"
#include "tim.h"
#include "Foc.h"
#include "Encoder.h"
#include "Adc_Sample.h"
#include "Pid.h"
#include "adc.h"
#include "CAN_communicate.h"
#include "fdcan.h"

float vofa_trans[32];
uint8_t vofa_trans_[132];
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8];
uint8_t TxData[8] = {'L', 'a', 'o', 'L', 'i', '1', '2', '3'};

FDCAN_TxHeaderTypeDef fdcan1_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN1_RXFilter;

uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t len)
{
    fdcan1_TxHeader.Identifier=0x12;                           //32λID
    fdcan1_TxHeader.IdType=FDCAN_STANDARD_ID;                  //��׼ID
    fdcan1_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //����֡
    fdcan1_TxHeader.DataLength=len;                            //���ݳ���
    fdcan1_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    fdcan1_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
    fdcan1_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    fdcan1_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
    fdcan1_TxHeader.MessageMarker=0;

    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&fdcan1_TxHeader,msg)!=HAL_OK) return 1;//����
    return 0;
}


/**
  * ��������: ���ò���ʼ���˲���0,��FDCAN1ʹ��
  * �������: void
  * ����ֵ��  void
  */
void FDCAN1_RX_Filter_Init(void)
{
    FDCAN_FilterTypeDef hfdcan1_RX_Filter;   /* FDCAN1�˲���0������ */

    hfdcan1_RX_Filter.IdType = FDCAN_STANDARD_ID;              /* ֻ���ձ�׼֡ID */
    hfdcan1_RX_Filter.FilterIndex = 0;                         /* �˲�������0 */
    hfdcan1_RX_Filter.FilterType = FDCAN_FILTER_RANGE;          /* �˲������� */
    hfdcan1_RX_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  /* �˲���������RXFIFO0 */
    hfdcan1_RX_Filter.FilterID1 = 0x000;                       /* �˲�ID1: 0x00 */
    hfdcan1_RX_Filter.FilterID2 = 0x7FF; /* �˲�ID2: 0x00 */
    HAL_FDCAN_ConfigFilter(&hfdcan1,&hfdcan1_RX_Filter);       /* �����˲�����û�д����ɹ� */
    /* HAL_FDCAN_ConfigGlobalFilter()
     * ����2�����ñ�׼֡ID�����յı���IDû��ƥ�����˲���ʱ��ѡ��ܾ�����(û��ƥ����ʱ,����ѡ�����FIFO0����FIFO1)��
     * ����3��������չ֡ID�����յı���IDû��ƥ�����˲���ʱ��ѡ��ܾ����ա�
     * ����4�������Ƿ�ܾ�Զ�̱�׼֡��ENABLE����ܾ����ա�
     * ����5�������Ƿ�ܾ�Զ����չ֡��ENABLE����ܾ����ա�
     */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,DISABLE,ENABLE); /* ����FDCAN1�˲���0ȫ������  */
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);

}

_Noreturn void Main() {
    HAL_Delay(5000);
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
    HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
    HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
    FDCAN1_RX_Filter_Init();
    Motor_Init();
    Motor_Calibrate();
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
    HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
    HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);

    while (true) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        HAL_Delay(500);
    }
}

static uint16_t a = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (&htim->Instance == &Timer_gate_drive.Instance) {
        Encoder_Get_Angle_analog(&Motor.Encoder_Theta_analog);
        Adc_Get_Current_Vbus(Motor.Abc_Current, Motor.Abc_Offstes, &foc.Udc);
        foc_Clark(Motor.Abc_Current[0], Motor.Abc_Current[1], Motor.Abc_Current[2], &Motor.AlphaBete_Current[0],
                  &Motor.AlphaBete_Current[1]);
        foc_Park(Motor.AlphaBete_Current[0], Motor.AlphaBete_Current[1], foc.Theta, &Motor.Qd_Current[0],
                 &Motor.Qd_Current[1]);

        Motor_Measure_Velocity(Motor.Encoder_Theta_analog, &Motor.Cur_Vel, &Motor.Cur_Pos);
        
        if (Motor.Motor_is_Calibrate) {
            Encoder_Get_Rle_Angle(Motor.Encoder_Theta_analog, &foc.Theta); //������������ֵ��ת��Ϊ��Ƕ�
            switch (Motor.Control_Mode) {
                case Control_Mode_Pos: {
                    Pid_Position(Motor.Tar_Pos, Motor.Cur_Pos, &Motor.Tar_Vel);
                    Pid_Velocity(Motor.Tar_Vel, Motor.Cur_Vel, &Motor.Tar_Qd_Current[0]);
                    Pid_Current(Motor.Tar_Qd_Current[0], 0, Motor.Qd_Current[0], Motor.Qd_Current[1], &foc.Vq, &foc.Vd);
                }
                    break;

                case Control_Mode_Vel: {
                    Pid_Velocity(Motor.Tar_Vel, Motor.Cur_Vel, &Motor.Tar_Qd_Current[0]);
                    Pid_Current(Motor.Tar_Qd_Current[0], 0, Motor.Qd_Current[0], Motor.Qd_Current[1], &foc.Vq, &foc.Vd);
                }
                    break;

                case Control_Mode_Cur: {
                   Pid_Current(Motor.Tar_Qd_Current[0], 0, Motor.Qd_Current[0], Motor.Qd_Current[1], &foc.Vq, &foc.Vd);
                }
                    break;

                case Control_Mode_Null: {

                }
                    break;
            }
//            foc.Vq = 0.4f;
//            foc.Vd = 0.0f;
//            foc.Theta+=0.003f;
        }

        foc_Revpark(foc.Vq, foc.Vd, foc.Theta, &foc.Valpha, &foc.Vbeta);
        svpwm(foc.Valpha, foc.Vbeta, foc.Udc, foc.CCRs);
        foc_SetCCR(foc.CCRs);

        ///ͼ����ʾ����
        vofa_trans[0] = foc.CCRs[0];
        vofa_trans[1] = foc.CCRs[1];
        vofa_trans[2] = foc.CCRs[2];

        vofa_trans[3] = Motor.Abc_Current[0];
        vofa_trans[4] = Motor.Abc_Current[1];
        vofa_trans[5] = Motor.Abc_Current[2];

        vofa_trans[6] = Motor.Tar_Qd_Current[0];
        vofa_trans[7] = Motor.Qd_Current[0];
        vofa_trans[8] = Motor.Tar_Qd_Current[1];
        vofa_trans[9] = Motor.Qd_Current[1];

        vofa_trans[10] = Motor.Tar_Pos;
        vofa_trans[11] = Motor.Cur_Pos;

        vofa_trans[12] = Pid.Vel_proportion;
        vofa_trans[13] = Pid.Vel_intergration;
        vofa_trans[14] = Pid.Vel_differential;

        vofa_trans[15] = Pid.Cur_proportion_q;
        vofa_trans[16] = Pid.Cur_intergration_q;

        vofa_trans[17] = foc.Vq;
        vofa_trans[18] = Motor.Tar_Vel;

        vofa_trans[19] = foc.Theta;

        vofa_trans[20] = Motor.Qd_Current[0];
        vofa_trans[21] = Motor.Qd_Current[1];

        vofa_trans[22] = Motor.AlphaBete_Current[0];
        vofa_trans[23] = Motor.AlphaBete_Current[1];
        vofa_trans[24] = Motor.Encoder_Theta_analog;
        vofa_trans[25] = Motor.Encoder_Offset_Spi;
        vofa_trans[26] = Motor.Cur_Vel;
        vofa_trans[27] = Motor.AlphaBete_Current[0];
        vofa_trans[28] = Motor.AlphaBete_Current[1];
        vofa_trans[29] = AD_CuB;
        vofa_trans[30] = AD_CuC;
        vofa_trans[31] = Motor.Encoder_Theta_analog;

        memcpy(vofa_trans_, (uint8_t *) vofa_trans, sizeof(vofa_trans));
        vofa_trans_[128] = 0X00;
        vofa_trans_[129] = 0X00;
        vofa_trans_[130] = 0X80;
        vofa_trans_[131] = 0X7F;
        CDC_Transmit_FS((uint8_t *) (vofa_trans_), 132);

    }


}


void Pack(uint8_t *buf) {                  ///usb���ƣ�����
    uint32_t state;
    float num, kp = 0, ki = 0, kd = 0;
    sscanf((const char *) buf, "%ld,%f,%f,%f,%f", &state, &num, &kp, &ki, &kd);
    Motor.Tar_Pos = num;
//    Pid.Vel_Kp = kp;
//    Pid.Vel_Ki = ki;
//    Pid.Vel_Kd = kd;
}