#include "main.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "tim.h"
#include "M3508_Motor.h"
extern uint8_t tx_data[8];
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mail_box_;
extern uint8_t rx_data[8];
extern uint8_t stop_flag;
M3508_Motor Motor(19.2, 30);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == htim6.Instance)
    {
        if (stop_flag)
        {
            Motor.SetPosition(Motor.target_angle_, 0.0f, 0.0f);
        }
        Motor.handle();
        tx_data[4] = Motor.given_current_ >> 8;
        tx_data[5] = Motor.given_current_ & 0xFF;
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    }
    if (rx_header.StdId == 0x203)
    {
        Motor.canRxMsgCallback(rx_data);
    }
}


