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
 * @brief   SAMV71 serial subsystem low level driver source.
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
#if (SAMV71_SERIAL_USE_UART0 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD3;
#endif
#if (SAMV71_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD4;
#endif
#if (SAMV71_SERIAL_USE_UART2 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD5;
#endif
#if (SAMV71_SERIAL_USE_UART3 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD6;
#endif
#if (SAMV71_SERIAL_USE_UART4 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD7;
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

#define USART_NOTIFY_HANDLER(SDno)                                         \
  static void onotify##SDno(io_queue_t *qp)                                \
  {                                                                        \
      /* This function is called whenever there is something to be sent */ \
      /* Here we should enable the TX interrupt */                         \
      if (!oqIsEmptyI(qp))                                                 \
      {                                                                    \
          SD##SDno.usart->US_IER = US_IER_TXRDY | US_IER_TXEMPTY;          \
      }                                                                    \
  }

#define UART_NOTIFY_HANDLER(SDno)                                          \
  static void onotify##SDno(io_queue_t *qp)                                \
  {                                                                        \
      /* This function is called whenever there is something to be sent */ \
      /* Here we should enable the TX interrupt */                         \
      if (!oqIsEmptyI(qp))                                                 \
      {                                                                    \
          SD##SDno.uart->UART_IER = UART_IER_TXRDY | UART_IER_TXEMPTY;     \
      }                                                                    \
  }

#if (SAMV71_SERIAL_USE_USART0 == TRUE) || defined(__DOXYGEN__)
USART_NOTIFY_HANDLER(0)
#endif
#if (SAMV71_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
USART_NOTIFY_HANDLER(1)
#endif
#if (SAMV71_SERIAL_USE_USART2 == TRUE) || defined(__DOXYGEN__)
USART_NOTIFY_HANDLER(2)
#endif
#if (SAMV71_SERIAL_USE_UART0 == TRUE) || defined(__DOXYGEN__)
UART_NOTIFY_HANDLER(3)
#endif
#if (SAMV71_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
UART_NOTIFY_HANDLER(4)
#endif
#if (SAMV71_SERIAL_USE_UART2 == TRUE) || defined(__DOXYGEN__)
UART_NOTIFY_HANDLER(5)
#endif
#if (SAMV71_SERIAL_USE_UART3 == TRUE) || defined(__DOXYGEN__)
UART_NOTIFY_HANDLER(6)
#endif
#if (SAMV71_SERIAL_USE_UART4 == TRUE) || defined(__DOXYGEN__)
UART_NOTIFY_HANDLER(7)
#endif

#if (SAMV71_SERIAL_USE_USART0 == TRUE) || (SAMV71_SERIAL_USE_USART1 == TRUE) || (SAMV71_SERIAL_USE_USART2 == TRUE) || defined(__DOXYGEN__)
/**
 * @brief Common IRQ handler for serial drivers using usarts
 */
static void sd_lld_serve_usart_interrupt(SerialDriver *sdp)
{
    Usart* dev = sdp->usart;

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
#endif

#if (SAMV71_SERIAL_USE_UART0 == TRUE) || (SAMV71_SERIAL_USE_UART1 == TRUE) || (SAMV71_SERIAL_USE_UART2 == TRUE) || (SAMV71_SERIAL_USE_UART3 == TRUE) || (SAMV71_SERIAL_USE_UART4 == TRUE) || defined(__DOXYGEN__)
/**
 * @brief Common IRQ handler for serial drivers using uarts
 */
static void sd_lld_serve_uart_interrupt(SerialDriver *sdp)
{
    Uart* dev = sdp->uart;

    /* Read and clear status*/
    uint32_t status = dev->UART_SR;
    // NOTE: Clearing the status will not affect TXRDY, RXRDY and TXEMPTY
    dev->UART_CR = UART_CR_RSTSTA;

    // TODO: Handle error conditions (e.g. parity etc)

    /* Handle received data */
    while (status & UART_SR_RXRDY)
    {
        // Read in the received character
        // NOTE: Reading from RHR automatically clears RXRDY unless another character is received
        osalSysLockFromISR();
        sdIncomingDataI(sdp, dev->UART_RHR);
        osalSysUnlockFromISR();

        // Reread status
        status = dev->UART_SR;
    }

    /* Handle data transmission */
    if (dev->UART_IMR & UART_IMR_TXRDY)
    {
        // When TXRDY interrupt has been enabled we read all characters we can from queue and transmit them
        while (status & UART_SR_TXRDY)
        {
            msg_t b;

            osalSysLockFromISR();
            b = oqGetI(&sdp->oqueue);
            if (b < MSG_OK)
            {
                // Nothing to send anymore, so disable TXRDY interrupts
                dev->UART_IDR = UART_IDR_TXRDY;
                chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
                osalSysUnlockFromISR();
                break;
            }
            // Transmit data
            // NOTE: The TXRDY is automatically cleared once we write to it
            dev->UART_THR = b;
            osalSysUnlockFromISR();

            // Reread status
            status = dev->UART_SR;
        }
    }

    /* Handle end of transmission */
    if ((dev->UART_IMR & UART_IMR_TXEMPTY) && (status & UART_SR_TXEMPTY))
    {
        osalSysLockFromISR();
        if (oqIsEmptyI(&sdp->oqueue)) {
            // Nothing to send anymore, so disable TXEMPTY interrupt
            dev->UART_IDR = UART_IDR_TXEMPTY;
            chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
        }
        osalSysUnlockFromISR();
    }
}
#endif

static void calculate_usart_baudrate(uint32_t clock, uint32_t speed,
                                     uint32_t *over, uint32_t *cd, uint32_t *fp,
                                     int *abs_error) {
  uint32_t cd_fp;
  /* Calculate the receiver sampling divide of baudrate clock. */
  if (clock >= HIGH_FRQ_SAMPLE_DIV * speed) {
    *over = HIGH_FRQ_SAMPLE_DIV;
  } else {
    *over = LOW_FRQ_SAMPLE_DIV;
  }
  /* Calculate clock divider according to the fraction calculated formula. */
  cd_fp = (8 * clock + (*over * speed) / 2) / (*over * speed);
  *cd = cd_fp >> 3;
  *fp = cd_fp & 0x07;
  *cd = *cd < MIN_CD_VALUE ? MIN_CD_VALUE : *cd;
  *cd = *cd > MAX_USART_CD_VALUE ? MAX_USART_CD_VALUE : *cd;

  *abs_error = speed - 8 * clock / (((*cd << 3) | *fp) * *over);
  *abs_error = *abs_error < 0 ? -*abs_error : *abs_error;
}

static void calculate_uart_baudrate(uint32_t clock, uint32_t speed,
                                    uint32_t *cd, int *abs_error) {
      /* Calculate clock divider according to the fraction calculated formula. */
      *cd = (clock / 16 + speed / 2) / (speed);
      *cd = *cd < MIN_CD_VALUE ? MIN_CD_VALUE : *cd;
      *cd = *cd > MAX_UART_CD_VALUE ? MAX_UART_CD_VALUE : *cd;
      *abs_error = speed - clock / 16 / *cd;
      *abs_error = *abs_error < 0 ? -*abs_error : *abs_error;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (SAMV71_SERIAL_USE_USART0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(USART0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_usart_interrupt(&SD0);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(USART1_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_usart_interrupt(&SD1);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SERIAL_USE_USART2 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(USART2_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_usart_interrupt(&SD2);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SERIAL_USE_UART0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(UART0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_uart_interrupt(&SD3);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(UART1_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_uart_interrupt(&SD4);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SERIAL_USE_UART2 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(UART2_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_uart_interrupt(&SD5);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SERIAL_USE_UART3 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(UART3_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_uart_interrupt(&SD6);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SERIAL_USE_UART4 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(UART4_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    sd_lld_serve_uart_interrupt(&SD7);
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
  SD0.usart = USART0;
  SD0.state = SD_STOP;
#endif
#if SAMV71_SERIAL_USE_USART1 == TRUE
  sdObjectInit(&SD1, NULL, onotify1);
  SD1.usart = USART1;
  SD1.state = SD_STOP;
#endif
#if SAMV71_SERIAL_USE_USART2 == TRUE
  sdObjectInit(&SD2, NULL, onotify2);
  SD2.usart = USART2;
  SD2.state = SD_STOP;
#endif
#if SAMV71_SERIAL_USE_UART0 == TRUE
  sdObjectInit(&SD3, NULL, onotify3);
  SD3.uart = UART0;
  SD3.state = SD_STOP;
#endif
#if SAMV71_SERIAL_USE_UART1 == TRUE
  sdObjectInit(&SD4, NULL, onotify4);
  SD4.uart = UART1;
  SD4.state = SD_STOP;
#endif
#if SAMV71_SERIAL_USE_UART2 == TRUE
  sdObjectInit(&SD5, NULL, onotify5);
  SD5.uart = UART2;
  SD5.state = SD_STOP;
#endif
#if SAMV71_SERIAL_USE_UART3 == TRUE
  sdObjectInit(&SD6, NULL, onotify6);
  SD6.uart = UART3;
  SD6.state = SD_STOP;
#endif
#if SAMV71_SERIAL_USE_UART4 == TRUE
  sdObjectInit(&SD7, NULL, onotify7);
  SD7.uart = UART4;
  SD7.state = SD_STOP;
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
#define U_ART_START(SD, U_ART)                                      \
    if (&SD == sdp) {                                               \
        /* Enable interrupt */                                      \
        nvicEnableVector(U_ART##_NVIC_NUMBER, USART_NVIC_PRIORITY); \
        /* Enable clock source */                                   \
        pmc_enable_periph_clk(ID_##U_ART);                          \
    }
#if SAMV71_SERIAL_USE_USART0 == TRUE
    U_ART_START(SD0, USART0)
#endif
#if SAMV71_SERIAL_USE_USART1 == TRUE
    U_ART_START(SD1, USART1)
#endif
#if SAMV71_SERIAL_USE_USART2 == TRUE
    U_ART_START(SD2, USART2)
#endif
#if SAMV71_SERIAL_USE_UART0 == TRUE
    U_ART_START(SD3, UART0)
#endif
#if SAMV71_SERIAL_USE_UART1 == TRUE
    U_ART_START(SD4, UART1)
#endif
#if SAMV71_SERIAL_USE_UART2 == TRUE
    U_ART_START(SD5, UART2)
#endif
#if SAMV71_SERIAL_USE_UART3 == TRUE
    U_ART_START(SD6, UART3)
#endif
#if SAMV71_SERIAL_USE_UART4 == TRUE
    U_ART_START(SD7, UART4)
#endif
    if(sdp->usart) {
      // Configure peripheral
      // Disable write protection
      sdp->usart->US_WPMR = US_WPMR_WPKEY_PASSWD;
      // Reset registers
      sdp->usart->US_MR = 0;
      sdp->usart->US_RTOR = 0;
      sdp->usart->US_TTGR = 0;
      sdp->usart->US_CR = US_CR_RSTRX | US_CR_RXDIS | US_CR_RSTTX | US_CR_TXDIS | US_CR_RSTSTA | US_CR_RTSDIS;
      // Configure registers
      uint32_t mr = config->char_length | config->parity_type | config->stop_bits | config->channel_mode;
      // Calculate and set baudrate
      uint32_t over_main;
      uint32_t cd_main;
      uint32_t fp_main;
      int error_main;
      uint32_t over_pclk;
      uint32_t cd_pclk;
      uint32_t fp_pclk;
      int error_pclk;
      calculate_usart_baudrate(USART_MAIN_CLOCK, config->speed,
                               &over_main, &cd_main, &fp_main, &error_main);
      calculate_usart_baudrate(USART_PCLK_CLOCK, config->speed,
                               &over_pclk, &cd_pclk, &fp_pclk, &error_pclk);
      //There is also a DIV clock, that is MAIN_CLOCK divided by 8 and
      //SLK, that is the external serial clock. Both are not of interest.
      if(error_main < error_pclk) {
        /* Configure the OVER bit in MR register. */
        if (over_main == 8) {
            mr |= US_MR_OVER;
        }
        mr |= US_MR_USCLKS_MCK;
        /* Configure the baudrate generate register. */
        sdp->usart->US_BRGR = (cd_main << US_BRGR_CD_Pos) | (fp_main << US_BRGR_FP_Pos);
      } else {
        /* Configure the OVER bit in MR register. */
        if (over_pclk == 8) {
            mr |= US_MR_OVER;
        }
        mr |= US_MR_USCLKS_PCK;
        /* Configure the baudrate generate register. */
        sdp->usart->US_BRGR = (cd_pclk << US_BRGR_CD_Pos) | (fp_pclk << US_BRGR_FP_Pos);
      }

      sdp->usart->US_MR |= mr;
      // Enable TX and RX
      sdp->usart->US_CR = US_CR_RXEN | US_CR_TXEN;
      // Enable RX interrupts (tx interrupts will be enabled by onotify functions)
      sdp->usart->US_IER = US_IER_RXRDY;
    }
    if(sdp->uart) {
      // Configure peripheral
      // Disable write protection
      sdp->uart->UART_WPMR = UART_WPMR_WPKEY_PASSWD;
      // Reset registers
      sdp->uart->UART_MR = 0;
      sdp->uart->UART_CR = UART_CR_RSTRX | UART_CR_RXDIS | UART_CR_RSTTX | UART_CR_TXDIS | UART_CR_RSTSTA;
      // Configure registers
      uint32_t mr = config->char_length | config->parity_type | config->stop_bits | config->channel_mode;
      // Calculate and set baudrate
      uint32_t cd_main;
      uint32_t cd_pclk;
      int err_main;
      int err_pclk;
      calculate_uart_baudrate(USART_MAIN_CLOCK, config->speed,
                              &cd_main, &err_main);
      calculate_uart_baudrate(USART_PCLK_CLOCK, config->speed,
                              &cd_pclk, &err_pclk);
      /* Calculate clock divider according to the fraction calculated formula. */
      if(err_main <= err_pclk) {
        mr |= UART_MR_BRSRCCK_PERIPH_CLK;
        /* Configure the baudrate generate register. */
        sdp->uart->UART_BRGR = (cd_main << UART_BRGR_CD_Pos);
      } else {
        mr |= UART_MR_BRSRCCK_PMC_PCK;
        /* Configure the baudrate generate register. */
        sdp->uart->UART_BRGR = (cd_pclk << UART_BRGR_CD_Pos);
      }
      sdp->uart->UART_MR |= mr;
      // Enable TX and RX
      sdp->uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
      // Enable RX interrupts (tx interrupts will be enabled by onotify functions)
      sdp->uart->UART_IER = UART_IER_RXRDY;
    }
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
      if(sdp->usart)
            sdp->usart->US_IDR = sdp->usart->US_IMR;
      if(sdp->uart)
            sdp->uart->UART_IDR = sdp->uart->UART_IMR;
#define U_ART_STOP(SD, U_ART)                   \
    if (&SD == sdp) {                           \
        /* Stop the clock */                    \
        pmc_disable_periph_clk(ID_##U_ART);     \
        /* Disable interrupt vector */          \
        nvicDisableVector(U_ART##_NVIC_NUMBER); \
    }
#if SAMV71_SERIAL_USE_USART0 == TRUE
    U_ART_STOP(SD0, USART0)
#endif
#if SAMV71_SERIAL_USE_USART1 == TRUE
    U_ART_STOP(SD1, USART1)
#endif
#if SAMV71_SERIAL_USE_USART2 == TRUE
    U_ART_STOP(SD2, USART2)
#endif
#if SAMV71_SERIAL_USE_UART0 == TRUE
    U_ART_STOP(SD3, UART0)
#endif
#if SAMV71_SERIAL_USE_UART1 == TRUE
    U_ART_STOP(SD4, UART1)
#endif
#if SAMV71_SERIAL_USE_UART2 == TRUE
    U_ART_STOP(SD5, UART2)
#endif
#if SAMV71_SERIAL_USE_UART3 == TRUE
    U_ART_STOP(SD6, UART3)
#endif
#if SAMV71_SERIAL_USE_UART4 == TRUE
    U_ART_STOP(SD7, UART4)
#endif
    sdp->state = SD_STOP;
  }
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
