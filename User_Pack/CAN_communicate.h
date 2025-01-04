  //
// Created by 29787 on 2024/10/25.
//

#ifndef PROJECT_CAN_COMMUNICATE_H
#define PROJECT_CAN_C
#include "main.h"
void fdcan_init();
void fdcan_send_msg(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t byte_size, uint8_t *pTxData);





#endif //PROJECT_CAN_COMMUNICATE_H
