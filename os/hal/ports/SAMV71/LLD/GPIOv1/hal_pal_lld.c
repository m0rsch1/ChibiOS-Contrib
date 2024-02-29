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
 * @file    hal_pal_lld.c
 * @brief   PLATFORM PAL subsystem low level driver source.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"
#include "hal_pmc_lld.h"

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
#if defined(ID_PIOE)
palevent_t _pal_events[32*5];
#elif defined(ID_PIOD)
palevent_t _pal_events[32*4];
#elif defined(ID_PIOC)
palevent_t _pal_events[32*3];
#elif defined(ID_PIOB)
palevent_t _pal_events[32*2];
#elif defined(ID_PIOA)
palevent_t _pal_events[32*1];
#else
palevent_t _pal_events[1];
#endif
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void pio_lld_serve_interrupt(Pio *pio, uint32_t num) {
  uint32_t isr = pio->PIO_ISR;
  uint32_t imr = pio->PIO_IMR;
  isr &= imr;
  for(int i = 0; i < 32; i++) {
    if((isr & (i << i)) == 0)
      continue;

    _pal_isr_code(num*32+i);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(ID_PIOA) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PIOA_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    pio_lld_serve_interrupt(PIOA, 0);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if defined(ID_PIOB) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PIOB_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    pio_lld_serve_interrupt(PIOB, 1);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if defined(ID_PIOC) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PIOC_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    pio_lld_serve_interrupt(PIOC, 2);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if defined(ID_PIOD) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PIOD_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    pio_lld_serve_interrupt(PIOD, 3);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if defined(ID_PIOE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PIOE_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    pio_lld_serve_interrupt(PIOE, 4);
    OSAL_IRQ_EPILOGUE();
}
#endif


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   SAMV71 I/O ports configuration.
 *
 * @notapi
 */
void _pal_lld_init(void) {
    // Setup the clocks
#if defined(ID_PIOA)
    pmc_enable_periph_clk(ID_PIOA);
#endif
#if defined(ID_PIOB)
    pmc_enable_periph_clk(ID_PIOB);
#endif
#if defined(ID_PIOC)
    pmc_enable_periph_clk(ID_PIOC);
#endif
#if defined(ID_PIOD)
    pmc_enable_periph_clk(ID_PIOD);
#endif
#if defined(ID_PIOE)
    pmc_enable_periph_clk(ID_PIOE);
#endif
}

void _pal_lld_setgroupPUPD(ioportid_t port,
                         ioportmask_t mask,
                         iomode_t mode)
{
    /* NOTE: pull down 'wins' over pull up if both have been set */
    if (mode & PAL_MODE_PULLDOWN)
    {
        port->PIO_PUDR = mask;
        port->PIO_PPDER = mask;
    } else if (mode & PAL_MODE_PULLUP)
    {
        port->PIO_PPDDR = mask;
        port->PIO_PUER = mask;
    } else {
        port->PIO_PPDDR = mask;
        port->PIO_PUDR = mask;
    }
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {
    /* Handle special mode: peripheral controlled port */
    if (mode & PAL_MODE_PERIPHERAL_CONTROLLED)
    {
        /* Disable interrupts on the pin(s) */
        port->PIO_IDR = mask;
        /* Set the ABCDSR register */
        uint32_t abcdsr = port->PIO_ABCDSR[0];
        port->PIO_ABCDSR[0] = mode & 0x1 ? (mask | abcdsr) : (~mask & abcdsr);
        abcdsr = port->PIO_ABCDSR[1];
        port->PIO_ABCDSR[1] = mode & 0x2 ? (mask | abcdsr) : (~mask & abcdsr);
        /* Remove the pin(s) from under the control of PIO */
        port->PIO_PDR = mask;
        /* Set pull-up or pull-down resistors */
        _pal_lld_setgroupPUPD(port, mask, mode);
        return;
    }

    /* 
     * Handle standard modes
     * NOTE: The lower 15 values define the standard modes
     */
    switch (mode & 0xFF)
    {
        case PAL_MODE_UNCONNECTED:
            /* Set pull-down and ... */
            mode = mode | PAL_MODE_PULLDOWN;
            /* fall through */
        case PAL_MODE_RESET:
        case PAL_MODE_INPUT_ANALOG:
            /* NOTE: There is no special analog functionality.
             * Please refer to the peripheral selection.
             *
             * When enabling a channel on the AFECs, the output driver is
             * disconnected on that pin and configuration changes through PIO
             * are blocked. The AFEC sampling order is ignored for this.
             */
        case PAL_MODE_INPUT:
            /* Disable interrupts on the pin(s) */
            port->PIO_IDR = mask;
            /* Set pull-up or pull-down resistors */
            _pal_lld_setgroupPUPD(port, mask, mode);
            /* Activate debounce and/or deglitch filters */
            if (mode & (PAL_MODE_INPUT_DEBOUNCE | PAL_MODE_INPUT_DEGLITCH))
            {
                port->PIO_IFER = mask;
                if (mode & PAL_MODE_INPUT_DEBOUNCE)
                {
                    port->PIO_IFSCER = mask;
                } else {
                    port->PIO_IFSCDR = mask;
                }
            } else {
                port->PIO_IFDR = mask;
            }
            /* Configure pin(s) as input(s) */
            port->PIO_ODR = mask;
            port->PIO_PER = mask;
            break;
        case PAL_MODE_OUTPUT_PUSHPULL:
        case PAL_MODE_OUTPUT_OPENDRAIN:
            /* Disable interrupts on the pin(s) */
            port->PIO_IDR = mask;
            /* Set pull-up or pull-down resistors */
            _pal_lld_setgroupPUPD(port, mask, mode);
            /* Set as opendrain or push-pull if needed */
            if ((mode & 0xFF) == PAL_MODE_OUTPUT_OPENDRAIN)
            {
                port->PIO_MDER = mask;
            } else {
                port->PIO_MDDR = mask;
            }
            /* Configure pin(s) as output(s) */
            port->PIO_OER = mask;
            port->PIO_PER = mask;
            /* TODO: Maybe we need to disable this at one point ... in the future*/
            port->PIO_OWER = mask;
            break;
        default:
            break;
    }
}

void _pal_lld_enablepadevent(ioportid_t port,
                             iopadid_t pad,
                             ioeventmode_t mode) {
  if (mode == PAL_EVENT_MODE_DISABLED) {
    port->PIO_IDR = PIO_IDR_P0 << pad;
    return;
  }

  if (mode == PAL_EVENT_MODE_BOTH_EDGES) {
    port->PIO_AIMDR = PIO_AIMDR_P0 << pad;
  } else {
    port->PIO_AIMER = PIO_AIMER_P0 << pad;

    if ((mode & PAL_EVENT_MODE_LEVEL_MASK) != 0) {
      port->PIO_LSR = PIO_LSR_P0 << pad;
    } else {
      port->PIO_ESR = PIO_ESR_P0 << pad;
    }
    if ((mode & PAL_EVENT_MODE_FALLING_EDGE) != 0) {
      port->PIO_FELLSR = PIO_FELLSR_P0 << pad;
    } else {
      port->PIO_REHLSR = PIO_REHLSR_P0 << pad;
    }
  }

  port->PIO_IER = PIO_IER_P0 << pad;
}

void _pal_lld_disablepadevent(ioportid_t port, iopadid_t pad) {
  port->PIO_IDR = PIO_IDR_P0 << pad;
}

#endif /* HAL_USE_PAL == TRUE */

/** @} */
