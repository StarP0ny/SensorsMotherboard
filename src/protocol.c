#include "protocol.h"
#include "crc.h"
#include <string.h>
static const uint8_t begin[4]={'V','S','L','B'}, end[4]={'B','L','S','V'};
uint16_t rd16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1]<<8); }
uint32_t rd32(const uint8_t *p) { return (uint32_t)rd16(p) | ((uint32_t)rd16(p+2)<<16); }
uint64_t rd64(const uint8_t *p) { return (uint64_t)rd32(p) | ((uint64_t)rd32(p+4)<<32); }
void wr16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
void wr32(uint8_t *p,uint32_t v) { wr16(p,(uint16_t)v); wr16(p+2,(uint16_t)(v>>16)); }
void wr64(uint8_t *p,uint64_t v) { wr32(p,(uint32_t)v); wr32(p+4,(uint32_t)(v>>32)); }
size_t proto_encode(uint8_t *out,uint16_t type,uint32_t seq,uint64_t time,
                    const uint8_t *payload,uint16_t length) {
    if(length>PROTO_MAX_PAYLOAD) return 0;
    memcpy(out,begin,4); wr32(out+4,seq); wr64(out+8,time);
    wr16(out+16,type); wr16(out+18,length);
    if(length) memcpy(out+20,payload,length);
    wr32(out+20+length,getCRC32(out+4,16+length));
    memcpy(out+24+length,end,4); return length+PROTO_OVERHEAD;
}
void proto_init(ProtoParser *p,ProtoHandler handler,void *ctx) {
    memset(p,0,sizeof(*p)); p->handler=handler; p->ctx=ctx;
}
static void discard(ProtoParser *p,uint16_t n) {
    p->used-=n; memmove(p->buf,p->buf+n,p->used);
}
static void parse(ProtoParser *p) {
    while(p->used>=4) {
        if(memcmp(p->buf,begin,4)) { discard(p,1); continue; }
        if(p->used<PROTO_HEADER_SIZE) return;
        uint16_t length=rd16(p->buf+18);
        if(length>PROTO_MAX_PAYLOAD) { p->bad_frames++; discard(p,1); continue; }
        uint16_t total=length+PROTO_OVERHEAD;
        if(p->used<total) return;
        if(memcmp(p->buf+24+length,end,4) ||
           rd32(p->buf+20+length)!=getCRC32(p->buf+4,16+length)) {
            p->bad_frames++; discard(p,1); continue;
        }
        p->handler(p->ctx,rd16(p->buf+16),rd32(p->buf+4),rd64(p->buf+8),p->buf+20,length);
        discard(p,total);
    }
}
void proto_feed(ProtoParser *p,const uint8_t *data,size_t length) {
    while(length--) {
        if(p->used==sizeof(p->buf)) { p->bad_frames++; discard(p,1); }
        p->buf[p->used++]=*data++; parse(p);
    }
}
void proto_expire(ProtoParser *p) {
    if(p->used) { p->bad_frames++; discard(p,1); parse(p); p->used=0; }
}
