#include "board.h"
#include "samv71.h"

void boardInit(void)
{
    /* Disable the watchdog */
    WDT->WDT_MR = WDT_MR_WDDIS;
    SystemInit();
    SystemCoreClockUpdate();
}
