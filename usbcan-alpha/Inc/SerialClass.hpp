/*
 * SerialClass.h
 *
 *  Created on: Aug 15, 2018
 *      Author: yusaku
 */

#ifndef ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_
#define ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

static constexpr uint16_t BUF_SIZE = 1024;

class SerialClass
{
private:
    static constexpr uint16_t buf_mask = BUF_SIZE - 1;
    uint8_t tx_buf[BUF_SIZE];
    uint8_t rx_buf[BUF_SIZE];
    bool tx_cplt = true;
    uint16_t rx_tail = 0;
    uint16_t tx_head = 0;
    uint16_t tx_tail = 0;
    UART_HandleTypeDef &huart;

public:
    SerialClass(UART_HandleTypeDef &huart)
            : huart(huart)
    {
        //this->huart = huart;
    }

    inline UART_HandleTypeDef * const get_handle(void)
    {
        return &huart;
    }

    inline void start_dma(void)
    {
        tx_cplt = true;
        rx_tail = 0;
        HAL_UART_Receive_DMA(&huart, (uint8_t *) rx_buf, BUF_SIZE);
    }

    inline int read(void)
    {
        uint16_t rx_head = (BUF_SIZE - huart.hdmarx->Instance->CNDTR) & buf_mask;
        if (rx_tail == rx_head)
        {
            return -1;
        }

        int c = (int) rx_buf[rx_tail++];
        rx_tail &= buf_mask;
        return c;
    }

#if 0
    inline int read_all(uint8_t * const buf, int len)
    {
        uint16_t rx_head = (BUF_SIZE - huart.hdmarx->Instance->CNDTR) & buf_mask;

        int pending = (rx_head - rx_tail + 1 + BUF_SIZE) & buf_mask;

        for (int i = 0; i < pending; i++)
        {
            if ((i < len))
            {
                buf[i] = rx_buf[rx_tail++];
                rx_tail &= buf_mask;
            }
        }

        if (rx_tail == rx_head)
        {
            return -1;
        }

        int c = (int) rx_buf[rx_tail++];
        rx_tail &= buf_mask;
        return c;
    }
#endif

    inline void write(const uint8_t * const c, const int length)
    {
        if (length > BUF_SIZE || length < 1)
        {
            return;
        }

        uint32_t offset = HAL_GetTick();
        uint32_t timeout = 10;
        while (!tx_cplt)
        {
            if (offset + timeout < HAL_GetTick())
            {
                // timeout
                tx_cplt = true;
                return;
            }
        }

        for (int i = 0; i < length; i++)
        {
            tx_buf[i] = c[i];
        }

        if (tx_cplt)
        {
            tx_cplt = false;
            auto ret = HAL_UART_Transmit_DMA(&huart, tx_buf, length);
            if (ret != HAL_OK)
            {
                // error
                tx_cplt = true;
                return;
            }
        }
    }

    inline void tx_cplt_callback(void)
    {
        tx_cplt = true;
    }
};

extern SerialClass serial;

#endif /* ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_ */
