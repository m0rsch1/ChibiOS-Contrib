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

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SAMV71_GPT_USE_GPT0
/**
 * @brief   TC0 IRQ Handler
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TC0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD0);

  OSAL_IRQ_EPILOGUE();
}
#endif
#if SAMV71_GPT_USE_GPT1
/**
 * @brief   TC1 IRQ Handler
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TC1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD1);

  OSAL_IRQ_EPILOGUE();
}
#endif
#if SAMV71_GPT_USE_GPT2
/**
 * @brief   TC2 IRQ Handler
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TC2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD2);

  OSAL_IRQ_EPILOGUE();
}
#endif
#if SAMV71_GPT_USE_GPT3
/**
 * @brief   TC3 IRQ Handler
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TC3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD3);

  OSAL_IRQ_EPILOGUE();
}
#endif
#if SAMV71_GPT_USE_GPT4
/**
 * @brief   TC4 IRQ Handler
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TC4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD4);

  OSAL_IRQ_EPILOGUE();
}
#endif

void gpt_lld_serve_interrupt(GPTDriver *gptp)
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
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {

#if SAMV71_GPT_USE_GPT0 == TRUE
  /* Driver initialization.*/
  gptObjectInit(&GPTD0);
  GPTD0.driver = TC0;
  gpt_lld_stop_timer(&GPTD0);
#endif
#if SAMV71_GPT_USE_GPT1 == TRUE
  /* Driver initialization.*/
  gptObjectInit(&GPTD1);
  GPTD1.driver = TC1;
  gpt_lld_stop_timer(&GPTD1);
#endif
#if SAMV71_GPT_USE_GPT2 == TRUE
  /* Driver initialization.*/
  gptObjectInit(&GPTD2);
  GPTD2.driver = TC2;
  gpt_lld_stop_timer(&GPTD2);
#endif
#if SAMV71_GPT_USE_GPT3 == TRUE
  /* Driver initialization.*/
  gptObjectInit(&GPTD3);
  GPTD3.driver = TC3;
  gpt_lld_stop_timer(&GPTD3);
#endif
#if SAMV71_GPT_USE_GPT4 == TRUE
  /* Driver initialization.*/
  gptObjectInit(&GPTD4);
  GPTD4.driver = TC4;
  gpt_lld_stop_timer(&GPTD4);
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
#if SAMV71_GPT_USE_GPT0 == TRUE
    if (&GPTD0 == gptp) {
        // First enable the clock of the timer
        pmc_enable_periph_clk(ID_TC0);
        // Enable the NVIC
        nvicEnableVector(TC0_NVIC_NUMBER, GPT_NVIC_PRIORITY);
    }
#endif
#if SAMV71_GPT_USE_GPT1 == TRUE
    if (&GPTD1 == gptp) {
        // First enable the clock of the timer
        pmc_enable_periph_clk(ID_TC1);
        // Enable the NVIC
        nvicEnableVector(TC1_NVIC_NUMBER, GPT_NVIC_PRIORITY);
    }
#endif
#if SAMV71_GPT_USE_GPT2 == TRUE
    if (&GPTD2 == gptp) {
        // First enable the clock of the timer
        pmc_enable_periph_clk(ID_TC2);
        // Enable the NVIC
        nvicEnableVector(TC2_NVIC_NUMBER, GPT_NVIC_PRIORITY);
    }
#endif
#if SAMV71_GPT_USE_GPT3 == TRUE
    if (&GPTD3 == gptp) {
        // First enable the clock of the timer
        pmc_enable_periph_clk(ID_TC3);
        // Enable the NVIC
        nvicEnableVector(TC3_NVIC_NUMBER, GPT_NVIC_PRIORITY);
    }
#endif
#if SAMV71_GPT_USE_GPT4 == TRUE
    if (&GPTD4 == gptp) {
        // First enable the clock of the timer
        pmc_enable_periph_clk(ID_TC4);
        // Enable the NVIC
        nvicEnableVector(TC4_NVIC_NUMBER, GPT_NVIC_PRIORITY);
    }
#endif
  }
  /* Configures the peripheral.*/
  // Disable write protection
  gptp->driver->TC_WPMR = TC_WPMR_WPKEY_PASSWD;
  // Given the requested frequency and the different clock options, we should derive the best clock to use for the timer
  int selected_divisor = 0;
  const uint32_t divisors[] = {2, 8, 32, 128};
  for (int i = 0; i < 4; i++)
  {
      // Select the current divisor
      selected_divisor = i;
      // Calculate boundaries of frequencies
      const uint32_t hf = GPT_MAIN_CLK / divisors[i];
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
#if SAMV71_GPT_USE_GPT0 == TRUE
    if (&GPTD0 == gptp) {
        // Disable the NVIC
        nvicDisableVector(TC0_NVIC_NUMBER);
        // Disable the clock of the timer
        pmc_disable_periph_clk(ID_TC0);
    }
#endif
#if SAMV71_GPT_USE_GPT1 == TRUE
    if (&GPTD1 == gptp) {
        // Disable the NVIC
        nvicDisableVector(TC1_NVIC_NUMBER);
        // Disable the clock of the timer
        pmc_disable_periph_clk(ID_TC1);
    }
#endif
#if SAMV71_GPT_USE_GPT2 == TRUE
    if (&GPTD2 == gptp) {
        // Disable the NVIC
        nvicDisableVector(TC2_NVIC_NUMBER);
        // Disable the clock of the timer
        pmc_disable_periph_clk(ID_TC2);
    }
#endif
#if SAMV71_GPT_USE_GPT3 == TRUE
    if (&GPTD3 == gptp) {
        // Disable the NVIC
        nvicDisableVector(TC3_NVIC_NUMBER);
        // Disable the clock of the timer
        pmc_disable_periph_clk(ID_TC3);
    }
#endif
#if SAMV71_GPT_USE_GPT4 == TRUE
    if (&GPTD4 == gptp) {
        // Disable the NVIC
        nvicDisableVector(TC4_NVIC_NUMBER);
        // Disable the clock of the timer
        pmc_disable_periph_clk(ID_TC4);
    }
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
