#pragma once

// TODO: The board init function has to setup the clock and the initial pin config
#include "system_samv71.h"

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */
