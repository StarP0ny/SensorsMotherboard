#include "app.h"
#include "board.h"
#include "board_config.h"
#include "protocol.h"
#include "sensors.h"
#include "transport.h"
#include "ch32v30x.h"
#include <string.h>
typedef struct {
    ProtoParser parser;
    uint32_t tx_sequence;
    uint32_t ping_sequence;
    uint32_t missed;
    uint64_t next_ping,last_rx;
    unsigned port;
    bool hello,linked,pending_ping;
} Session;
static Session sessions[2];
static Session *owner;
static bool auto_mode,button_raw,button_stable;
static uint64_t button_changed,next_status;
static uint16_t requested_mask;
static uint32_t ping_ms=1000,miss_limit=1,tx_interval_us=0,sync_interval_ms=30000;
static bool synced;
static int64_t utc_offset;
static uint32_t boot_id;
static uint8_t batch[PROTO_MAX_PAYLOAD];
static uint16_t batch_used=2,batch_count;
static uint64_t batch_base,batch_due;
static uint64_t wire_time(uint64_t mono) { return synced?(uint64_t)((int64_t)mono+utc_offset):mono; }
static bool send_at(Session *s,uint16_t type,uint64_t time,const uint8_t *data,uint16_t len,bool control) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint32_t seq=s->tx_sequence++;
    size_t size=proto_encode(frame,type,seq,time,data,len);
    return transport_send(s->port,frame,size,control);
}
static void send(Session *s,uint16_t type,const uint8_t *data,uint16_t len) {
    send_at(s,type,wire_time(mono_us()),data,len,true);
}
static void result(Session *s,uint32_t req,uint16_t code) {
    uint8_t data[6]; wr32(data,req); wr16(data+4,code); send(s,RESULT,data,sizeof(data));
}
static void flush_batch(void) {
    if(batch_count && owner && owner->linked) {
        wr16(batch,batch_count); send_at(owner,DATA,batch_base,batch,batch_used,false);
    }
    batch_used=2; batch_count=0;
}
static uint16_t run_mask(void) { return (auto_mode?3:0)|((owner&&owner->linked)?requested_mask:0); }
static void status(Session *s) {
    uint8_t data[20]={0}; data[0]=auto_mode; data[1]=s->linked; wr16(data+2,run_mask());
    wr32(data+4,s->missed); wr32(data+8,transport_drops()); wr32(data+12,s->parser.bad_frames);
    wr16(data+16,(sensors[0].present?1:0)|(sensors[1].present?2:0));
    /* bits: automatic=1, ping-linked=2; reserved journal fields remain zero */
    send(s,STATUS,data,sizeof(data));
}
static void sample(uint16_t id,uint64_t mono,const uint16_t *words,uint16_t count) {
    if(!owner || !owner->linked) return; /* offline journal is the next implementation stage */
    uint64_t time=wire_time(mono); uint16_t size=8+count*2;
    if(batch_count && (time<batch_base || time-batch_base>UINT32_MAX || batch_used+size>PROTO_MAX_PAYLOAD)) flush_batch();
    if(!batch_count) { batch_base=time; batch_due=mono+tx_interval_us; }
    uint8_t *p=batch+batch_used; wr16(p,id); wr32(p+2,(uint32_t)(time-batch_base)); wr16(p+6,count*2);
    for(uint16_t i=0;i<count;i++) wr16(p+8+i*2,words[i]);
    batch_used+=size; batch_count++;
    if(!tx_interval_us) flush_batch();
}
static void sensor_error(uint16_t id,uint16_t error) {
    if(owner && owner->linked) { uint8_t data[4]; wr16(data,id); wr16(data+2,error); send(owner,EVENT,data,4); }
}
static void config_reply(Session *s,uint32_t req,uint16_t object) {
    uint8_t data[32]; wr32(data,req); wr16(data+4,object);
    if(object==0) {
        uint32_t values[4]={ping_ms,miss_limit,tx_interval_us,sync_interval_ms}; wr16(data+6,4);
        for(unsigned i=0;i<4;i++) { wr16(data+8+i*6,i+1); wr32(data+10+i*6,values[i]); }
        send(s,CONFIG,data,32);
    } else { wr16(data+6,1); wr16(data+8,16); wr32(data+10,sensors[object-1].period_us); send(s,CONFIG,data,14); }
}
static void handle(void *ctx,uint16_t type,uint32_t seq,uint64_t time,const uint8_t *p,uint16_t n) {
    Session *s=ctx; uint64_t now=mono_us();
    if(type==HELLO_REQ) {
        if(n!=2) { result(s,seq,RES_BAD_LENGTH); return; }
        if(rd16(p)!=PROTO_VERSION) { result(s,seq,RES_UNSUPPORTED); return; }
        if(owner && owner!=s && owner->linked) { result(s,seq,RES_BUSY); return; }
        flush_batch(); owner=s; requested_mask=0;
        s->hello=true; s->linked=true; s->pending_ping=false; s->missed=0; s->next_ping=now+(uint64_t)ping_ms*1000;
        uint8_t data[38]; wr32(data,seq); wr16(data+4,PROTO_VERSION);
        wr16(data+6,0); wr16(data+8,1); wr16(data+10,0); wr16(data+12,PROTO_MAX_PAYLOAD);
        board_uid(data+14); wr32(data+26,RCC->RSTSCKR); wr32(data+30,boot_id);
        wr32(data+34,3); /* capabilities: acquisition + time sync; no flash log yet */
        send(s,HELLO,data,sizeof(data)); return;
    }
    if(!s->hello || owner!=s) { result(s,seq,RES_BUSY); return; }
    if(type==PING) {
        if(n) { result(s,seq,RES_BAD_LENGTH); return; }
        s->linked=true; s->missed=0; uint8_t data[4]; wr32(data,seq); send(s,PONG,data,4); return;
    }
    if(type==PONG) {
        if(n!=4) { result(s,seq,RES_BAD_LENGTH); return; }
        if(s->pending_ping && rd32(p)==s->ping_sequence) {
            s->pending_ping=false; s->missed=0; s->linked=true;
        }
        return;
    }
    if(type==ENUM_REQ) {
        if(n) { result(s,seq,RES_BAD_LENGTH); return; }
        uint8_t data[38]={0}; wr32(data,seq); wr16(data+4,2);
        for(unsigned i=0;i<2;i++) {
            uint8_t *d=data+6+i*16; wr16(d,i+1); wr16(d+2,i+1); d[4]=1; d[5]=MUX_ADDRESS;
            d[6]=i?SHT_CHANNEL:SCD_CHANNEL; d[7]=i?0x44:0x62;
            wr32(d+8,sensors[i].period_us); wr16(d+12,sensors[i].present);
        }
        send(s,ENUM_REPLY,data,sizeof(data)); return;
    }
    if(type==GET_CONFIG) {
        if(n!=2) { result(s,seq,RES_BAD_LENGTH); return; }
        if(rd16(p)>2) { result(s,seq,RES_BAD_VALUE); return; }
        config_reply(s,seq,rd16(p)); return;
    }
    if(type==SET_CONFIG) {
        if(n<4 || n!=4+rd16(p+2)*6U) { result(s,seq,RES_BAD_LENGTH); return; }
        uint16_t object=rd16(p),count=rd16(p+2);
        uint32_t values[4]={ping_ms,miss_limit,tx_interval_us,sync_interval_ms};
        uint32_t period=object>=1&&object<=2?sensors[object-1].period_us:0;
        for(unsigned i=0;i<count;i++) {
            uint16_t key=rd16(p+4+i*6); uint32_t value=rd32(p+6+i*6);
            if(object==0 && key>=1 && key<=4) {
                if((key==1 && (value<100 || value>60000)) || (key==2 && (value<1 || value>100)) ||
                   (key==3 && value>1000000) || (key==4 && value && (value<1000 || value>3600000))) goto bad_config;
                values[key-1]=value;
            } else if(object>=1 && object<=2 && key==16 &&
                      value>=(object==1?5000000U:10000U) && value<=60000000U) period=value;
            else goto bad_config;
        }
        if(object>2) goto bad_config;
        flush_batch();
        if(object==0) {
            ping_ms=values[0]; miss_limit=values[1]; tx_interval_us=values[2]; sync_interval_ms=values[3];
            s->pending_ping=false; s->missed=0; s->next_ping=now+(uint64_t)ping_ms*1000;
        } else sensors[object-1].period_us=period;
        result(s,seq,RES_OK); return;
    bad_config: result(s,seq,RES_BAD_VALUE); return;
    }
    if(type==START || type==STOP) {
        if(n<2 || n!=2+rd16(p)*2U) { result(s,seq,RES_BAD_LENGTH); return; }
        uint16_t mask=rd16(p)?0:3;
        for(unsigned i=0;i<rd16(p);i++) {
            uint16_t id=rd16(p+2+i*2); if(id<1 || id>2) { result(s,seq,RES_BAD_VALUE); return; }
            mask|=1U<<(id-1);
        }
        if(type==START) requested_mask|=mask; else requested_mask&=~mask;
        result(s,seq,RES_OK); return;
    }
    if(type==TIME_SYNC_REQ) {
        if(n || time<UTC_THRESHOLD_US || time>INT64_MAX) { result(s,seq,RES_BAD_VALUE); return; }
        uint8_t data[20]; wr32(data,seq); wr64(data+4,time); wr64(data+12,now);
        send_at(s,TIME_SYNC_RESP,mono_us(),data,20,true); return;
    }
    if(type==TIME_SYNC_SET) {
        if(n!=8) { result(s,seq,RES_BAD_LENGTH); return; }
        int64_t offset=(int64_t)rd64(p);
        if(offset<0 || (uint64_t)offset>INT64_MAX-now || now+(uint64_t)offset<UTC_THRESHOLD_US) {
            result(s,seq,RES_BAD_VALUE); return;
        }
        flush_batch(); utc_offset=offset; synced=true; result(s,seq,RES_OK); return;
    }
    result(s,seq,RES_UNSUPPORTED);
}
void app_init(void) {
    for(unsigned i=0;i<2;i++) { sessions[i].port=i; proto_init(&sessions[i].parser,handle,&sessions[i]); }
    button_raw=button_stable=board_button_pressed(); button_changed=mono_us();
    boot_id=(uint32_t)mono_us()^RCC->RSTSCKR;
    sensors_init(sample,sensor_error);
}
void app_poll(void) {
    uint64_t now=mono_us(); uint8_t data[128];
    for(unsigned i=0;i<2;i++) {
        Session *s=&sessions[i]; size_t n=transport_read(i,data,sizeof(data));
        if(n) { s->last_rx=now; proto_feed(&s->parser,data,n); }
        else if(s->parser.used && now-s->last_rx>250000) proto_expire(&s->parser);
    }
    bool pressed=board_button_pressed();
    if(pressed!=button_raw) { button_raw=pressed; button_changed=now; }
    if(button_raw!=button_stable && now-button_changed>=30000) {
        button_stable=button_raw;
        if(button_stable) { auto_mode=!auto_mode; board_auto_led(auto_mode); if(owner) status(owner); }
    }
    if(owner && owner->hello && now>=owner->next_ping) {
        if(owner->pending_ping && ++owner->missed>=miss_limit) { owner->linked=false; batch_count=0; batch_used=2; }
        owner->ping_sequence=owner->tx_sequence;
        owner->pending_ping=send_at(owner,PING,wire_time(mono_us()),NULL,0,true);
        if(!owner->pending_ping && ++owner->missed>=miss_limit) owner->linked=false;
        owner->next_ping=now+(uint64_t)ping_ms*1000;
    }
    sensors_poll(run_mask());
    if(batch_count && mono_us()>=batch_due) flush_batch();
    if(owner && owner->linked && now>=next_status) { status(owner); next_status=now+1000000; }
}
