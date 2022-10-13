#include "hal.h"
#include "ch.h"
#include "samv71.h"

int main(void)
{   
    // Initialize HAL (including clock, systick)
    halInit();
    // Initialize ChibiOS
    chSysInit();

    // Setup LEDs
    palSetLineMode(LINE_LED0, PAL_MODE_OUTPUT_PUSHPULL | PAL_MODE_PULLUP);
    palSetLine(LINE_LED0);
    palSetLineMode(LINE_LED1, PAL_MODE_OUTPUT_PUSHPULL | PAL_MODE_PULLUP);
    palClearLine(LINE_LED1);

    // Setup USART0 or SD0 respectively
    sdStart(&SD0, NULL);
    if (SD0.state == SD_READY)
    {
        palClearLine(LINE_LED0);
    }

    while(1) {
        palToggleLine(LINE_LED1);
        msg_t b = sdGet(&SD0);
        sdPut(&SD0, b);
    }
}
