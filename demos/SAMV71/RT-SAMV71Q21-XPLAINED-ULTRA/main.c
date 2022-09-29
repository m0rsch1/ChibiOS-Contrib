#include "hal.h"
#include "ch.h"
#include "samv71.h"

int main(void)
{   
    // Initialize HAL (including clock, systick)
    halInit();
    // Initialize ChibiOS
    chSysInit();

    while(1) {}
}
