#include "board.h"
#include "transport.h"
#include "i2c_bus.h"
#include "app.h"
int main(void) {
    board_init(); i2c_bus_init(); transport_init(); app_init();
    while(1) { transport_poll(); app_poll(); }
}
