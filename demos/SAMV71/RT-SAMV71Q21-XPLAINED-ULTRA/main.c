#include "hal.h"
#include "ch.h"
#include "samv71.h"

int main(void)
{   
    // Initialize HAL (including clock, systick)
    halInit();
    // Initialize ChibiOS
    chSysInit();

    palSetLineMode(LINE_LED0, PAL_MODE_OUTPUT_PUSHPULL | PAL_MODE_PULLUP);
    palClearLine(LINE_LED0);
    palSetLineMode(LINE_LED1, PAL_MODE_OUTPUT_PUSHPULL | PAL_MODE_PULLUP);
    palClearLine(LINE_LED1);

    while(1) {
        for (volatile uint32_t i = 0; i < 1000000; ++i);
        palSetLine(LINE_LED1);
        for (volatile uint32_t j = 0; j < 1000000; ++j);
        palClearLine(LINE_LED1);
    }
}
