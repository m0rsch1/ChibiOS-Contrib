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
 * @file    hal_gpt_lld.c
 * @brief   PLATFORM GPT subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "hal.h"

#if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   GPTD0 driver identifier.
 */
#if (SAMV71_GPT_USE_GPT0 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD0;
#endif
#if (SAMV71_GPT_USE_GPT1 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD1;
#endif
#if (SAMV71_GPT_USE_GPT2 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD2;
#endif
#if (SAMV71_GPT_USE_GPT3 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD3;
#endif
#if (SAMV71_GPT_USE_GPT4 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD4;
#endif
#if (SAMV71_GPT_USE_GPT5 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD5;
#endif
#if (SAMV71_GPT_USE_GPT6 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD6;
#endif
#if (SAMV71_GPT_USE_GPT7 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD7;
#endif
#if (SAMV71_GPT_USE_GPT8 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD8;
#endif
#if (SAMV71_GPT_USE_GPT9 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD9;
#endif
#if (SAMV71_GPT_USE_GPT10 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD10;
#endif
#if (SAMV71_GPT_USE_GPT11 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD11;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void gpt_lld_serve_interrupt(GPTDriver *gptp)
{
    uint32_t sr = gptp->driver->TC_CHANNEL[gptp->channel].TC_SR;
    sr &= gptp->driver->TC_CHANNEL[gptp->channel].TC_IMR;
    if (sr & TC_SR_CPCS)
    {
        // Trigger on RC compare occurred. Call callback
        _gpt_isr_invoke_cb(gptp);
    }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#define TC_IRQ_HANDLER(num)             \
OSAL_IRQ_HANDLER(TC##num##_HANDLER) {   \
  OSAL_IRQ_PROLOGUE();                  \
  gpt_lld_serve_interrupt(&GPTD##0);    \
  OSAL_IRQ_EPILOGUE();                  \
}

#if SAMV71_GPT_USE_GPT0
TC_IRQ_HANDLER(0)
#endif
#if SAMV71_GPT_USE_GPT1
TC_IRQ_HANDLER(1)
#endif
#if SAMV71_GPT_USE_GPT2
TC_IRQ_HANDLER(2)
#endif
#if SAMV71_GPT_USE_GPT3
TC_IRQ_HANDLER(3)
#endif
#if SAMV71_GPT_USE_GPT4
TC_IRQ_HANDLER(4)
#endif
#if SAMV71_GPT_USE_GPT5
TC_IRQ_HANDLER(5)
#endif
#if SAMV71_GPT_USE_GPT6
TC_IRQ_HANDLER(6)
#endif
#if SAMV71_GPT_USE_GPT7
TC_IRQ_HANDLER(7)
#endif
#if SAMV71_GPT_USE_GPT8
TC_IRQ_HANDLER(8)
#endif
#if SAMV71_GPT_USE_GPT9
TC_IRQ_HANDLER(9)
#endif
#if SAMV71_GPT_USE_GPT10
TC_IRQ_HANDLER(10)
#endif
#if SAMV71_GPT_USE_GPT11
TC_IRQ_HANDLER(11)
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {

#define TC_OBJECT_INIT(num,block,ch)       \
  gptObjectInit(&GPTD##num);               \
  GPTD0.driver = TC##block;                \
  GPTD0.channel = ch;                      \
  gpt_lld_stop_timer(&GPTD##num)

  /* Driver initialization.*/
#if SAMV71_GPT_USE_GPT0 == TRUE
  TC_OBJECT_INIT(0, 0, 0);
#endif
#if SAMV71_GPT_USE_GPT1 == TRUE
  TC_OBJECT_INIT(1, 0, 1);
#endif
#if SAMV71_GPT_USE_GPT2 == TRUE
  TC_OBJECT_INIT(2, 0, 2);
#endif
#if SAMV71_GPT_USE_GPT3 == TRUE
  TC_OBJECT_INIT(3, 1, 0);
#endif
#if SAMV71_GPT_USE_GPT4 == TRUE
  TC_OBJECT_INIT(4, 1, 1);
#endif
#if SAMV71_GPT_USE_GPT5 == TRUE
  TC_OBJECT_INIT(5, 1, 2);
#endif
#if SAMV71_GPT_USE_GPT6 == TRUE
  TC_OBJECT_INIT(6, 2, 0);
#endif
#if SAMV71_GPT_USE_GPT7 == TRUE
  TC_OBJECT_INIT(7, 2, 1);
#endif
#if SAMV71_GPT_USE_GPT8 == TRUE
  TC_OBJECT_INIT(8, 2, 2);
#endif
#if SAMV71_GPT_USE_GPT9 == TRUE
  TC_OBJECT_INIT(9, 3, 0);
#endif
#if SAMV71_GPT_USE_GPT10 == TRUE
  TC_OBJECT_INIT(10, 3, 1);
#endif
#if SAMV71_GPT_USE_GPT11 == TRUE
  TC_OBJECT_INIT(11, 3, 2);
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {

  if (gptp->state == GPT_STOP) {
    /* Enables the peripheral.*/

#define TC_START_DEV(num)                                            \
    if (&GPTD##num == gptp) {                                        \
        /* First enable the clock of the timer  */                   \
        pmc_enable_periph_clk(ID_TC##num);                           \
        /* Enable the NVIC */                                        \
        nvicEnableVector(TC##num##_NVIC_NUMBER, GPT_NVIC_PRIORITY);  \
    }
#if SAMV71_GPT_USE_GPT0 == TRUE
    TC_START_DEV(0);
#endif
#if SAMV71_GPT_USE_GPT1 == TRUE
    TC_START_DEV(1);
#endif
#if SAMV71_GPT_USE_GPT2 == TRUE
    TC_START_DEV(2);
#endif
#if SAMV71_GPT_USE_GPT3 == TRUE
    TC_START_DEV(3);
#endif
#if SAMV71_GPT_USE_GPT4 == TRUE
    TC_START_DEV(4);
#endif
#if SAMV71_GPT_USE_GPT5 == TRUE
    TC_START_DEV(5);
#endif
#if SAMV71_GPT_USE_GPT6 == TRUE
    TC_START_DEV(6);
#endif
#if SAMV71_GPT_USE_GPT7 == TRUE
    TC_START_DEV(7);
#endif
#if SAMV71_GPT_USE_GPT8 == TRUE
    TC_START_DEV(8);
#endif
#if SAMV71_GPT_USE_GPT9 == TRUE
    TC_START_DEV(9);
#endif
#if SAMV71_GPT_USE_GPT10 == TRUE
    TC_START_DEV(10);
#endif
#if SAMV71_GPT_USE_GPT11 == TRUE
    TC_START_DEV(11);
#endif
  }
  /* Configures the peripheral.*/
  // Disable write protection
  gptp->driver->TC_WPMR = TC_WPMR_WPKEY_PASSWD;
  // Given the requested frequency and the different clock options, we should derive the best clock to use for the timer
  int selected_divisor = 0;
  const uint32_t freq[] = {ProgrammableClock[6],
                           GPT_MAIN_CLK / 2 / 2,
                           GPT_MAIN_CLK / 2 / 8,
                           GPT_MAIN_CLK / 2 / 32,
                           GPT_MAIN_CLK / 2 / 128};
  for (int i = 0; i < 4; i++)
  {
      // Select the current divisor
      selected_divisor = i;
      // Calculate boundaries of frequencies
      const uint32_t hf = freq[i];
      const uint32_t lf = hf / 65535;
      if ((gptp->config->frequency < hf) && (gptp->config->frequency > lf))
          break;
  }
  // Set CMR to zero first
  gptp->driver->TC_CHANNEL[gptp->channel].TC_CMR = 0;
  // Set the clock divisor (by index not by value)
  gptp->driver->TC_CHANNEL[gptp->channel].TC_CMR |= selected_divisor;
  // Set the timer to use reset the counter on RC compare
  gptp->driver->TC_CHANNEL[gptp->channel].TC_CMR |= TC_CMR_CPCTRG;

  // NOTE: IRQs are enabled when the timer is started

}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {

  if (gptp->state == GPT_READY) {
    /* Resets the peripheral.*/

    // Disable write protection
    gptp->driver->TC_WPMR = TC_WPMR_WPKEY_PASSWD;
    // Disable interrupts and reset the status register
    /*  Disable TC clock. */
    gptp->driver->TC_CHANNEL[gptp->channel].TC_CCR = TC_CCR_CLKDIS;
    /*  Disable interrupts. */
    gptp->driver->TC_CHANNEL[gptp->channel].TC_IDR = 0xFFFFFFFF;
    /*  Clear status register. */
    gptp->driver->TC_CHANNEL[gptp->channel].TC_SR;
    /* Disables the peripheral.*/

#define TC_STOP_DEV(num)                           \
    if (&GPTD##num == gptp) {                      \
        /* Disable the NVIC */                     \
        nvicDisableVector(TC##num##_NVIC_NUMBER);  \
        /* Disable the clock of the timer */       \
        pmc_disable_periph_clk(ID_TC##num);        \
    }
#if SAMV71_GPT_USE_GPT0 == TRUE
    TC_STOP_DEV(0);
#endif
#if SAMV71_GPT_USE_GPT1 == TRUE
    TC_STOP_DEV(1);
#endif
#if SAMV71_GPT_USE_GPT2 == TRUE
    TC_STOP_DEV(2);
#endif
#if SAMV71_GPT_USE_GPT3 == TRUE
    TC_STOP_DEV(3);
#endif
#if SAMV71_GPT_USE_GPT4 == TRUE
    TC_STOP_DEV(4);
#endif
#if SAMV71_GPT_USE_GPT5 == TRUE
    TC_STOP_DEV(5);
#endif
#if SAMV71_GPT_USE_GPT6 == TRUE
    TC_STOP_DEV(6);
#endif
#if SAMV71_GPT_USE_GPT7 == TRUE
    TC_STOP_DEV(7);
#endif
#if SAMV71_GPT_USE_GPT8 == TRUE
    TC_STOP_DEV(8);
#endif
#if SAMV71_GPT_USE_GPT9 == TRUE
    TC_STOP_DEV(9);
#endif
#if SAMV71_GPT_USE_GPT10 == TRUE
    TC_STOP_DEV(10);
#endif
#if SAMV71_GPT_USE_GPT11 == TRUE
    TC_STOP_DEV(11);
#endif
  }
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval) {
    // TODO: In the STM32 implementation this function does not check the state. Is this correct?
    // Change the interval
    gpt_lld_change_interval(gptp, interval);
    // Start the clock and the timer
    gptp->driver->TC_CHANNEL[gptp->channel].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
    /*  Clear status register. */
    gptp->driver->TC_CHANNEL[gptp->channel].TC_SR;
    // Enable RC compare IRQ
    gptp->driver->TC_CHANNEL[gptp->channel].TC_IER = TC_IER_CPCS;
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {
    // Stop the clock
    gptp->driver->TC_CHANNEL[gptp->channel].TC_CCR = TC_CCR_CLKDIS;
    /*  Clear status register. */
    gptp->driver->TC_CHANNEL[gptp->channel].TC_SR;
    // Disable RC compare IRQ (and all other enabled IRQs)
    gptp->driver->TC_CHANNEL[gptp->channel].TC_IDR = gptp->driver->TC_CHANNEL[gptp->channel].TC_IMR;
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {

  (void)gptp;
  (void)interval;

}

#endif /* HAL_USE_GPT == TRUE */

/** @} */
