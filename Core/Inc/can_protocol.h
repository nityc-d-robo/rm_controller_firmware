#ifndef CAN_PROTOCOL_H_
#define CAN_PROTOCOL_H_

#include <stdbool.h>

#include "stm32g4xx_hal.h"
#include "can_comm.h"

// Masks
#define CAN_MASK_FUNC           0b11100000000U
#define CAN_MASK_BOARD_CLASS    0b00011100000U
#define CAN_MASK_ADDR           0b00000011111U
#define CAN_MASK_DEVICE         0b00011111111U

#define CAN_MASK_BOARD_1        0b00000011111U
#define CAN_MASK_BOARD_2        0b00000011110U
#define CAN_MASK_BOARD_4        0b00000011100U
#define CAN_MASK_BOARD_8        0b00000011000U
#define CAN_MASK_BOARD_16       0b00000010000U

#define CAN_MASK_DEVICE_1       0b00000000000U
#define CAN_MASK_DEVICE_2       0b00000000001U
#define CAN_MASK_DEVICE_4       0b00000000011U
#define CAN_MASK_DEVICE_8       0b00000000111U
#define CAN_MASK_DEVICE_16      0b00000001111U

#define CAN_Master_ID       0b00000000U

// Func Code
typedef enum {
    CAN_FUNC_EMCY      = 0,
    CAN_FUNC_SYNC_TIME = 1,
    CAN_FUNC_WRITE     = 2,
    CAN_FUNC_READ      = 3,
    CAN_FUNC_REPORT    = 4,
    CAN_FUNC_HEARTBEAT = 5,
    CAN_FUNC_DEBUG     = 6
} CANFunc;

typedef enum {
    CAN_BOARD_CLASS_1  = 0,
    CAN_BOARD_CLASS_2  = 1,
    CAN_BOARD_CLASS_4  = 2,
    CAN_BOARD_CLASS_8  = 3,
    CAN_BOARD_CLASS_16 = 4,
} CANBoardClass;

// CAN Packet <-> Key/Value Fromat

// Key Value Format Data
typedef struct {
    CANFunc func;
    CANBoardClass class;
    uint8_t addr;
    uint8_t key;
    uint8_t value;
} KvData;

// CAN Packet -> Key/Value
bool CANPacket2Kv(const RxCanFrame *frame, KvData *outs, uint8_t *count);

// Key/Value -> CAN Packet
bool Data2CANPacket(const KvData *datas, const uint8_t count, TxCanFrame *out);
bool Kv2CANPacket(const KvData datas, TxCanFrame *out);

#endif