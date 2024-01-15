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
#if (SAMV71_SERIAL_USE_USART0 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD0;
#endif
#if (SAMV71_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif
#if (SAMV71_SERIAL_USE_USART2 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  US_MR_CHRL_8_BIT,
  US_MR_PAR_NO,
  US_MR_NBSTOP_1_BIT,
  US_MR_CHMODE_NORMAL | US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK,
  0U
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if (SAMV71_SERIAL_USE_USART0 == TRUE) || defined(__DOXYGEN__)
static void onotify0(io_queue_t *qp)
{
    // This function is called whenever there is something to be sent
    // Here we should enable the TX interrupt
    if (!oqIsEmptyI(qp))
    {
        SD0.device->US_IER = US_IER_TXRDY | US_IER_TXEMPTY;
    }
}
#endif
#if (SAMV71_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
static void onotify1(io_queue_t *qp)
{
    // This function is called whenever there is something to be sent
    // Here we should enable the TX interrupt
    if (!oqIsEmptyI(qp))
    {
        SD1.device->US_IER = US_IER_TXRDY | US_IER_TXEMPTY;
    }
}
#endif
#if (SAMV71_SERIAL_USE_USART2 == TRUE) || defined(__DOXYGEN__)
static void onotify2(io_queue_t *qp)
{
    // This function is called whenever there is something to be sent
    // Here we should enable the TX interrupt
    if (!oqIsEmptyI(qp))
    {
        SD2.device->US_IER = US_IER_TXRDY | US_IER_TXEMPTY;
    }
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (SAMV71_SERIAL_USE_USART0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(USART0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_interrupt(&SD0);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(USART1_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_interrupt(&SD1);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SERIAL_USE_USART2 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(USART2_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_interrupt(&SD2);
    OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {
#if SAMV71_SERIAL_USE_USART0 == TRUE
  sdObjectInit(&SD0, NULL, onotify0);
  SD0.device = USART0;
  SD0.state = SD_STOP;
#endif
#if SAMV71_SERIAL_USE_USART1 == TRUE
  sdObjectInit(&SD1, NULL, onotify1);
  SD1.device = USART1;
  SD1.state = SD_STOP;
#endif
#if SAMV71_SERIAL_USE_USART2 == TRUE
  sdObjectInit(&SD2, NULL, onotify2);
  SD2.device = USART2;
  SD2.state = SD_STOP;
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
#if SAMV71_SERIAL_USE_USART0 == TRUE
    if (&SD0 == sdp) {
        // Enable interrupt
        nvicEnableVector(USART0_NVIC_NUMBER, USART_NVIC_PRIORITY);
        // Enable clock source
        pmc_enable_periph_clk(ID_USART0);
    }
#endif
#if SAMV71_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {
        // Enable interrupt
        nvicEnableVector(USART1_NVIC_NUMBER, USART_NVIC_PRIORITY);
        // Enable clock source
        pmc_enable_periph_clk(ID_USART1);
    }
#endif
#if SAMV71_SERIAL_USE_USART2 == TRUE
    if (&SD2 == sdp) {
        // Enable interrupt
        nvicEnableVector(USART2_NVIC_NUMBER, USART_NVIC_PRIORITY);
        // Enable clock source
        pmc_enable_periph_clk(ID_USART2);
    }
#endif
    // Configure peripheral
    // Disable write protection
    sdp->device->US_WPMR = US_WPMR_WPKEY_PASSWD;
    // Reset registers
    sdp->device->US_MR = 0;
    sdp->device->US_RTOR = 0;
    sdp->device->US_TTGR = 0;
    sdp->device->US_CR = US_CR_RSTRX | US_CR_RXDIS | US_CR_RSTTX | US_CR_TXDIS | US_CR_RSTSTA | US_CR_RTSDIS;
    // Configure registers
    uint32_t mr = config->char_length | config->parity_type | config->stop_bits | config->channel_mode;
    sdp->device->US_MR |= mr;
    // Calculate and set baudrate
    uint32_t over;
    uint32_t cd_fp;
    uint32_t cd;
    uint32_t fp;
    /* Calculate the receiver sampling divide of baudrate clock. */
    if (USART_MAIN_CLOCK >= HIGH_FRQ_SAMPLE_DIV * config->speed) {
        over = HIGH_FRQ_SAMPLE_DIV;
    } else {
        over = LOW_FRQ_SAMPLE_DIV;
    }
    /* Calculate clock divider according to the fraction calculated formula. */
    cd_fp = (8 * USART_MAIN_CLOCK + (over * config->speed) / 2) / (over * config->speed);
    cd = cd_fp >> 3;
    fp = cd_fp & 0x07;
    cd = cd < MIN_CD_VALUE ? MIN_CD_VALUE : cd;
    cd = cd > MAX_CD_VALUE ? MAX_CD_VALUE : cd;
    /* Configure the OVER bit in MR register. */
    if (over == 8) {
        sdp->device->US_MR |= US_MR_OVER;
    }
    /* Configure the baudrate generate register. */
    sdp->device->US_BRGR = (cd << US_BRGR_CD_Pos) | (fp << US_BRGR_FP_Pos);
    // Enable TX and RX
    sdp->device->US_CR = US_CR_RXEN | US_CR_TXEN;
    // Enable RX interrupts (tx interrupts will be enabled by onotify functions)
    sdp->device->US_IER = US_IER_RXRDY;
    // Mark the driver as ready
    sdp->state = SD_READY;
  }
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
      // Disable all enabled interrupts
      sdp->device->US_IDR = sdp->device->US_IMR;
#if SAMV71_SERIAL_USE_USART0 == TRUE
    if (&SD0 == sdp) {
        // Stop the clock
        pmc_disable_periph_clk(ID_USART0);
        // Disable interrupt vector
        nvicDisableVector(USART0_NVIC_NUMBER);
    }
#endif
#if SAMV71_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {
        // Stop the clock
        pmc_disable_periph_clk(ID_USART1);
        // Disable interrupt vector
        nvicDisableVector(USART1_NVIC_NUMBER);
    }
#endif
#if SAMV71_SERIAL_USE_USART2 == TRUE
    if (&SD2 == sdp) {
        // Stop the clock
        pmc_disable_periph_clk(ID_USART2);
        // Disable interrupt vector
        nvicDisableVector(USART2_NVIC_NUMBER);
    }
#endif
    sdp->state = SD_STOP;
  }
}

/*
 * @brief Common IRQ handler for serial drivers
 */
void sd_lld_serve_interrupt(SerialDriver *sdp)
{
    Usart* dev = sdp->device;

    /* Read and clear status*/
    uint32_t status = dev->US_CSR;
    // NOTE: Clearing the status will not affect TXRDY, RXRDY and TXEMPTY
    dev->US_CR = US_CR_RSTSTA;

    // TODO: Handle error conditions (e.g. parity etc)

    /* Handle received data */
    while (status & US_CSR_RXRDY)
    {
        // Read in the received character
        // NOTE: Reading from RHR automatically clears RXRDY unless another character is received
        osalSysLockFromISR();
        sdIncomingDataI(sdp, dev->US_RHR);
        osalSysUnlockFromISR();

        // Reread status
        status = dev->US_CSR;
    }

    /* Handle data transmission */
    if (dev->US_IMR & US_IMR_TXRDY)
    {
        // When TXRDY interrupt has been enabled we read all characters we can from queue and transmit them
        while (status & US_CSR_TXRDY)
        {
            msg_t b;

            osalSysLockFromISR();
            b = oqGetI(&sdp->oqueue);
            if (b < MSG_OK)
            {
                // Nothing to send anymore, so disable TXRDY interrupts
                dev->US_IDR = US_IDR_TXRDY;
                chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
                osalSysUnlockFromISR();
                break;
            }
            // Transmit data
            // NOTE: The TXRDY is automatically cleared once we write to it
            dev->US_THR = b;
            osalSysUnlockFromISR();

            // Reread status
            status = dev->US_CSR;
        }
    }

    /* Handle end of transmission */
    if ((dev->US_IMR & US_IMR_TXEMPTY) && (status & US_CSR_TXEMPTY))
    {
        osalSysLockFromISR();
        if (oqIsEmptyI(&sdp->oqueue)) {
            // Nothing to send anymore, so disable TXEMPTY interrupt
            dev->US_IDR = US_IDR_TXEMPTY;
            chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
        }
        osalSysUnlockFromISR();
    }
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
