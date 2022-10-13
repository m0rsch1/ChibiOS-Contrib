#pragma once

#include "system_samv71.h"

#define LINE_LED0 PAL_LINE(IOPORT1, 23U)
#define LINE_LED1 PAL_LINE(IOPORT3, 9U)

/* USART LINES */
#define LINE_USART0_RXD PAL_LINE(IOPORT2, 0U)
#define LINE_USART0_TXD PAL_LINE(IOPORT2, 1U)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */
