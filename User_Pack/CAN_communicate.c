//
// Created by 29787 on 2024/10/25.
//

#include "CAN_communicate.h"
#include "fdcan.h"
#include "Motor_Control.h"
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t RX_DATA1[8];
#define motor_id 0x08
uint8_t Motor_id=motor_id;
void fdcan_init() {

    FDCAN_FilterTypeDef sfdcanFilterConfig;
    sfdcanFilterConfig.IdType = FDCAN_STANDARD_ID;
    sfdcanFilterConfig.FilterIndex = 0;
    sfdcanFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sfdcanFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sfdcanFilterConfig.FilterID1 = 0x000;
    sfdcanFilterConfig.FilterID2 = 0x7fff;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sfdcanFilterConfig) == HAL_OK ? ((void) NULL) : Error_Handler();

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)
    == HAL_OK
    ? ((void) (NULL))
    : Error_Handler();

    HAL_FDCAN_Start(&hfdcan1) == HAL_OK ? ((void) NULL) : Error_Handler();

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK
    ? ((void) NULL)
    : Error_Handler();

}


void fdcan_send_msg(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t byte_size, uint8_t *pTxData) {
    byte_size > FDCAN_DLC_BYTES_64 ? byte_size = FDCAN_DLC_BYTES_64 : ((void) NULL);
    FDCAN_TxHeaderTypeDef fdcanTxHeader = {
            .Identifier = (uint32_t) id,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = byte_size,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0x00,
    };
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &fdcanTxHeader, pTxData);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    __HAL_FDCAN_DISABLE_IT(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    if (hfdcan->Instance == FDCAN1) {
        if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader1,RX_DATA1)==HAL_OK)
        {
            if(RxHeader1.Identifier==Motor_id)
            {
                int16_t temp_angle=0;
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                if(Motor_id>4)
                    temp_angle= (RX_DATA1[(Motor_id-4-1)*2]<<8)  +  (RX_DATA1[(Motor_id-4-1)*2+1]);
                else
                    temp_angle= (RX_DATA1[(Motor_id - 1)*2]<<8)  +  (RX_DATA1[(Motor_id-1)*2+1]);
                Motor.Tar_Pos=(float)temp_angle/10.f;
            }
        }
    }
    __HAL_FDCAN_ENABLE_IT(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}