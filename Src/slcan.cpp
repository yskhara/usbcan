/*
 * slcan.c
 *
 *  based on a version from linklayer/cantact-fw
 */

#include "stm32f1xx_hal.h"
#include "can.hpp"
#include "slcan.h"

static uint32_t current_filter_id = 0;
static uint32_t current_filter_mask = 0;

int8_t slcan_parse_frame(uint8_t *buf, CAN_RxHeaderTypeDef *rx_header, uint8_t (&rx_payload)[CAN_MTU])
{
    uint8_t i = 0;
    uint8_t id_len, j;
    uint32_t tmp;

    for (j = 0; j < SLCAN_MTU; j++)
    {
        buf[j] = '\0';
    }

    // add character for frame type
    if (rx_header->RTR == CAN_RTR_DATA)
    {
        buf[i] = 't';
    }
    else if (rx_header->RTR == CAN_RTR_REMOTE)
    {
        buf[i] = 'r';
    }

    // assume standard identifier
    id_len = SLCAN_STD_ID_LEN;
    tmp = rx_header->StdId;
    // check if extended
    if (rx_header->IDE == CAN_ID_EXT)
    {
        // convert first char to upper case for extended frame
        buf[i] -= 32;
        id_len = SLCAN_EXT_ID_LEN;
        tmp = rx_header->ExtId;
    }
    i++;

    // add identifier to buffer
    for (j = id_len; j > 0; j--)
    {
        // add nybble to buffer
        buf[j] = (tmp & 0xF);
        tmp = tmp >> 4;
        i++;
    }

    // add DLC to buffer
    buf[i++] = rx_header->DLC;

    // add data bytes
    for (j = 0; j < rx_header->DLC; j++)
    {
        buf[i++] = (rx_payload[j] >> 4);
        buf[i++] = (rx_payload[j] & 0x0F);
    }

    // convert to ASCII (2nd character to end)
    for (j = 1; j < i; j++)
    {
        if (buf[j] < 0xA)
        {
            buf[j] += 0x30;
        }
        else
        {
            buf[j] += 0x37;
        }
    }

    // add carrage return (slcan EOL)
    buf[i++] = '\r';

    // return number of bytes in string
    return i;
}

CAN_TxHeaderTypeDef tx_header;
int8_t slcan_parse_str(uint8_t *buf, uint8_t len)
{
    uint8_t tx_payload[CAN_MTU];
    uint8_t i;

    if (len == 0)
    {
        return -1;
    }

    // convert from ASCII (2nd character to end)
    for (i = 1; i < len; i++)
    {
        // lowercase letters
        if (buf[i] >= 'a')
            buf[i] = buf[i] - 'a' + 10;
        // uppercase letters
        else if (buf[i] >= 'A')
            buf[i] = buf[i] - 'A' + 10;
        // numbers
        else
            buf[i] = buf[i] - '0';
    }

    if (buf[0] == 'O')
    {
        // open channel command
        can_enable();
        return 0;

    }
    else if (buf[0] == 'C')
    {
        // close channel command
        can_disable();
        return 0;

    }
    else if (buf[0] == 'S')
    {
        // set bitrate command
        switch (buf[1])
        {
            case 0:
                can_set_bitrate(CAN_BITRATE_10K);
                break;
            case 1:
                can_set_bitrate(CAN_BITRATE_20K);
                break;
            case 2:
                can_set_bitrate(CAN_BITRATE_50K);
                break;
            case 3:
                can_set_bitrate(CAN_BITRATE_100K);
                break;
            case 4:
                can_set_bitrate(CAN_BITRATE_125K);
                break;
            case 5:
                can_set_bitrate(CAN_BITRATE_250K);
                break;
            case 6:
                can_set_bitrate(CAN_BITRATE_500K);
                break;
            case 7:
                can_set_bitrate(CAN_BITRATE_750K);
                break;
            case 8:
                can_set_bitrate(CAN_BITRATE_1000K);
                break;
            default:
                // invalid setting
                return -1;
        }
        return 0;

    }
    else if (buf[0] == 'm' || buf[0] == 'M')
    {
        // set mode command
        if (buf[1] == 1)
        {
            // mode 1: silent
            can_set_silent(1);
        }
        else
        {
            // default to normal mode
            can_set_silent(0);
        }
        return 0;

    }
    else if (buf[0] == 'F')
    {
        // set filter command
        uint32_t id = 0;
        for (i = 1; i < len; i++)
        {
            id *= 16;
            id += buf[i];
        }
        current_filter_id = id;
        can_set_filter(current_filter_id, current_filter_mask);

    }
    else if (buf[0] == 'K')
    {
        // set mask command
        uint32_t mask = 0;
        for (i = 1; i < len; i++)
        {
            mask *= 16;
            mask += buf[i];
        }
        current_filter_mask = mask;
        can_set_filter(current_filter_id, current_filter_mask);

    }
    else if (buf[0] == 't' || buf[0] == 'T')
    {
        // transmit data frame command
        tx_header.RTR = CAN_RTR_DATA;

    }
    else if (buf[0] == 'r' || buf[0] == 'R')
    {
        // transmit remote frame command
        tx_header.RTR = CAN_RTR_REMOTE;

    }
    else
    {
        // error, unknown command
        return -1;
    }

    if (buf[0] == 't' || buf[0] == 'r')
    {
        tx_header.IDE = CAN_ID_STD;
    }
    else if (buf[0] == 'T' || buf[0] == 'R')
    {
        tx_header.IDE = CAN_ID_EXT;
    }
    else
    {
        // error
        return -1;
    }

    tx_header.StdId = 0;
    tx_header.ExtId = 0;
    if (tx_header.IDE == CAN_ID_EXT)
    {
        uint8_t id_len = SLCAN_EXT_ID_LEN;
        i = 1;
        while (i <= id_len)
        {
            tx_header.ExtId *= 16;
            tx_header.ExtId += buf[i++];
        }
    }
    else
    {
        uint8_t id_len = SLCAN_STD_ID_LEN;
        i = 1;
        while (i <= id_len)
        {
            tx_header.StdId *= 16;
            tx_header.StdId += buf[i++];
        }
    }

    tx_header.DLC = buf[i++];
    if (tx_header.DLC < 0 || tx_header.DLC > 8)
    {
        return -1;
    }

    uint8_t j;
    for (j = 0; j < tx_header.DLC; j++)
    {
        tx_payload[j] = (buf[i] << 4) + buf[i + 1];
        i += 2;
    }

    // send the message
    can_tx(&tx_header, tx_payload);

    return 0;
}

