/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_st_lld.c
 * @brief   PLATFORM ST subsystem low level driver source.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   IRQ handling code.
 */
static void st_lld_serve_interrupt(void) {
    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ST_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  st_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void) {
    // TODO: In here, we need to setup the SYSTICK or TIMER
#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC
    /* Periodic systick mode, the Cortex-Mx internal systick timer is used
       in this mode.*/
    SysTick->LOAD = (SYSTICK_CK / OSAL_ST_FREQUENCY) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_ENABLE_Msk |
                    SysTick_CTRL_TICKINT_Msk;

    /* IRQ enabled.*/
    nvicSetSystemHandlerPriority(HANDLER_SYSTICK, ST_NVIC_PRIORITY);
#else
#error "hal_st_lld.c of SAMV71 does not yet support tickless mode"
#endif
}


#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
