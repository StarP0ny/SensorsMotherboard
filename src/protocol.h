#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define PROTO_VERSION 1U
#define PROTO_MAX_PAYLOAD 1024U
#define PROTO_HEADER_SIZE 20U
#define PROTO_OVERHEAD 28U
#define PROTO_MAX_FRAME (PROTO_MAX_PAYLOAD + PROTO_OVERHEAD)
#define UTC_THRESHOLD_US UINT64_C(631152000000000)
enum MessageType {
    HELLO_REQ=0x0001, HELLO=0x0002, ENUM_REQ=0x0010, ENUM_REPLY=0x0011,
    GET_CONFIG=0x0020, SET_CONFIG=0x0021, CONFIG=0x0022,
    START=0x0030, STOP=0x0031, DATA=0x0040, STATUS=0x0041, EVENT=0x0042,
    TIME_SYNC_REQ=0x0050, TIME_SYNC_RESP=0x0051, TIME_SYNC_SET=0x0052,
    LOG_INFO_REQ=0x0060, LOG_INFO=0x0061, LOG_READ=0x0062, LOG_DATA=0x0063,
    LOG_ACK=0x0064, RESULT=0x0070, PING=0x0080, PONG=0x0081
};
enum ResultCode { RES_OK, RES_BAD_LENGTH, RES_UNSUPPORTED, RES_BAD_VALUE, RES_BUSY };
uint16_t rd16(const uint8_t *p);
uint32_t rd32(const uint8_t *p);
uint64_t rd64(const uint8_t *p);
void wr16(uint8_t *p, uint16_t v);
void wr32(uint8_t *p, uint32_t v);
void wr64(uint8_t *p, uint64_t v);
typedef void (*ProtoHandler)(void *ctx, uint16_t type, uint32_t seq,
                             uint64_t time, const uint8_t *payload, uint16_t length);
typedef struct {
    uint8_t buf[PROTO_MAX_FRAME];
    uint16_t used;
    uint32_t bad_frames;
    ProtoHandler handler;
    void *ctx;
} ProtoParser;
void proto_init(ProtoParser *p, ProtoHandler handler, void *ctx);
void proto_feed(ProtoParser *p, const uint8_t *data, size_t length);
void proto_expire(ProtoParser *p); /* abandon an incomplete frame after an inter-byte gap */
size_t proto_encode(uint8_t *out, uint16_t type, uint32_t seq, uint64_t time,
                    const uint8_t *payload, uint16_t length);
