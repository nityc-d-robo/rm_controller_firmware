#include "can_protocol.h"

static inline CANFunc CAN_GetFunc(uint16_t id) {
    return (CANFunc)((id & CAN_MASK_FUNC) >> 8U);
}

static inline uint8_t CAN_GetBoardClass(uint16_t id) {
    return (CANBoardClass)((id & CAN_MASK_BOARD_CLASS) >> 5U);
}

static inline uint8_t CAN_GetAddr(uint16_t id) {
    return (uint8_t)(id & CAN_MASK_ADDR);
}

static inline uint16_t CAN_MakeId(CANFunc func, CANBoardClass board_class, uint8_t addr) {
    return (uint16_t)(
        ((uint16_t)func << 8U) |
        ((uint16_t)board_class << 5U) |
        ((uint16_t)addr & CAN_MASK_ADDR));
}

// CAN Packet -> Key/Value
bool CANPacket2Kv(const RxCanFrame *frame, KvData *outs, uint8_t *count) {
    outs[0].addr = CAN_GetAddr(frame->Identifier);
    outs[0].func = CAN_GetFunc(frame->Identifier);
    outs[0].class = CAN_GetBoardClass(frame->Identifier);

    uint_fast8_t stride = (outs[0].func == CAN_FUNC_READ) ? 1U : 2U;
    *count = (uint8_t)(frame->Length / stride);

    for (int_fast8_t i = 0; i < *count; i++) {
        outs[i].addr = outs[0].addr;
        outs[i].func = outs[0].func;
        outs[i].class = outs[0].class;
        outs[i].key = frame->data[(i * stride)];

        outs[i].value = (stride == 2U) ? frame->data[(i * stride) + 1] : 0U;
    }
    return true;
}

// Func, Id, *Key/Value -> CAN Packet ([1~4]->[1])
bool Data2CANPacket(const KvData *datas, const uint8_t count, TxCanFrame *out) {
    out->Identifier = CAN_MakeId(datas[0].func, datas[0].class, datas[0].addr);

    uint_fast8_t stride = (datas[0].func == CAN_FUNC_READ) ? 1U : 2U;

    for (int_fast8_t i = 0; i < count; i++) {
        if (out->Identifier != CAN_MakeId(datas[i].func, datas[i].class, datas[i].addr)) {
            return false;
        }
        out->data[(i * stride)] = datas[i].key;
        out->data[(i * stride) + 1U] = datas[i].value;
    }
    out->Length = (uint8_t)(count << 1);
    return true;
}

// Key/Value -> CAN Packet ([1]->[1])
bool Kv2CANPacket(const KvData datas, TxCanFrame *out) {
    return Data2CANPacket(&datas, 1U, out);
}
