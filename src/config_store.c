#include "config_store.h"
#include "flash.h"
#include <string.h>
FLASH_STORE_AREA(topology, 4);
static uint32_t checksum(const void *data,uint32_t length) { return getCRC32(data,(int32_t)length); }
static const FlashStoreCfg_t cfg={_fs_region_topology,4,CONFIG_BYTES,checksum,NULL};
static FlashStore_t store;
static bool ready;
static void init(void) {
    if(!ready) { flashStoreInit(&store,&cfg); ready=true; }
}
bool config_load(uint8_t data[CONFIG_BYTES]) {
    init(); return flashStoreReadLast(&store,data)==flOk;
}
bool config_save(const uint8_t data[CONFIG_BYTES]) {
    uint8_t old[CONFIG_BYTES];
    init();
    if(flashStoreReadLast(&store,old)==flOk && !memcmp(old,data,CONFIG_BYTES)) return true;
    if(flashStoreWriteNext(&store,data)!=flOk) return false;
    return flashStoreReadLast(&store,old)==flOk && !memcmp(old,data,CONFIG_BYTES);
}
