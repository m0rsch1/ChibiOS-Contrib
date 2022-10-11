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
 * @file    hal_serial_lld.c
 * @brief   PLATFORM serial subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 serial driver identifier.*/
#if (PLATFORM_SERIAL_USE_USART0 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD0;
#endif
#if (PLATFORM_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif
#if (PLATFORM_SERIAL_USE_USART2 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  38400,
  US_MR_CHRL_8_BIT,
  US_MR_PAR_NO,
  US_MR_NBSTOP_1_BIT,
  US_MR_CHMODE_NORMAL,
  0U
};

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
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {
#if PLATFORM_SERIAL_USE_USART0 == TRUE
  sdObjectInit(&SD0, NULL, notify0);
#endif
#if PLATFORM_SERIAL_USE_USART1 == TRUE
  sdObjectInit(&SD1, NULL, notify1);
#endif
#if PLATFORM_SERIAL_USE_USART2 == TRUE
  sdObjectInit(&SD2, NULL, notify2);
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL) {
    config = &default_config;
  }

  if (sdp->state == SD_STOP) {
#if PLATFORM_SERIAL_USE_USART0 == TRUE
    if (&SD0 == sdp) {

    }
#endif
  }
  /* Configures the peripheral.*/
  (void)config; /* Warning suppression, remove this.*/
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
#if PLATFORM_SERIAL_USE_USART0 == TRUE
    if (&SD0 == sdp) {

    }
#endif
  }
}

/*
 * @brief Common IRQ handler for serial drivers
 */
void sd_lld_serve_interrupt(SerialDriver *sdp)
{
    (void)sdp;
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
