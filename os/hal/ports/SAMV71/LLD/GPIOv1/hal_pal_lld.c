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

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

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
    if (mode & PAL_MODE_PULLUP)
    {
        port->PIO_PPDDR = mask;
        port->PIO_PUER = mask;
    }
    if (mode & PAL_MODE_PULLDOWN)
    {
        port->PIO_PUDR = mask;
        port->PIO_PPDER = mask;
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
     * NOTE: The lower 15 Bits define the standard modes
     */
    switch (mode & 0xFF)
    {
        case PAL_MODE_UNCONNECTED:
            /* Set pull-down and ... */
            mode = mode | PAL_MODE_PULLDOWN;
            /* fall through */
        case PAL_MODE_RESET:
        case PAL_MODE_INPUT_ANALOG:
            /* NOTE: There is no special analog functionality. Please refer to the peripheral selection */
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

#endif /* HAL_USE_PAL == TRUE */

/** @} */
