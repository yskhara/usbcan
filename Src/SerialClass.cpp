/*
 * SerialClass.cpp
 *
 *  Created on: Dec 26, 2018
 *      Author: yusaku
 */

#include "SerialClass.hpp"



extern UART_HandleTypeDef huart1;


SerialClass serial(huart1);

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // compare pointer
    if (huart->Instance == serial.get_handle()->Instance)
    {
        serial.tx_cplt_callback();
    }
}

