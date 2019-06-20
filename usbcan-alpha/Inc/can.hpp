#ifndef _CAN_H
#define _CAN_H

#ifdef __cplusplus
 extern "C" {
#endif

#define CAN_MTU 8

enum can_bitrate {
    CAN_BITRATE_10K,
    CAN_BITRATE_20K,
    CAN_BITRATE_50K,
    CAN_BITRATE_100K,
    CAN_BITRATE_125K,
    CAN_BITRATE_250K,
    CAN_BITRATE_500K,
    CAN_BITRATE_750K,
    CAN_BITRATE_1000K,
};

enum can_bus_state {
    OFF_BUS,
    ON_BUS
};

void can_init(void);
void can_enable(void);
void can_disable(void);
void can_set_bitrate(enum can_bitrate bitrate);
void can_set_silent(uint8_t silent);
uint32_t can_tx(CAN_TxHeaderTypeDef *tx_header, uint8_t (&buf)[CAN_MTU]);
//uint32_t can_tx(CAN_TxHeaderTypeDef *tx_header,uint8_t aData[]);
uint32_t can_rx(CAN_RxHeaderTypeDef *rx_header, uint8_t (&buf)[CAN_MTU]);
uint8_t is_can_msg_pending(uint8_t fifo);
void can_set_filter(uint32_t id, uint32_t mask);

#ifdef __cplusplus
 }
#endif


#define CAN_MTU 8

template<typename T>
union _Encapsulator
{
    T data;
    uint64_t i;
};

template<typename T>
static void can_unpack(const uint8_t (&buf)[CAN_MTU], T &data);
template<typename T>
static void can_pack(uint8_t (&buf)[CAN_MTU], const T data);

 // unpacks can payload
 template<typename T>
 void can_unpack(const uint8_t (&buf)[CAN_MTU], T &data)
 {
     _Encapsulator<T> _e;

     for (int i = 0; i < sizeof(T); i++)
     {
         _e.i = (_e.i << 8) | (uint64_t) (buf[i]);
     }

     data = _e.data;
 }

 // packs can payload
 template<typename T>
 void can_pack(uint8_t (&buf)[CAN_MTU], const T data)
 {
     _Encapsulator<T> _e;
     _e.data = data;

     for (int i = sizeof(T); i > 0;)
     {
         i--;
         buf[i] = _e.i & 0xff;
         _e.i >>= 8;
     }
 }

#endif // _CAN_H
