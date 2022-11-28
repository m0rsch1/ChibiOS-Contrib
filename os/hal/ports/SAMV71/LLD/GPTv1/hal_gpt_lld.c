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
#if (PLATFORM_GPT_USE_GPT0 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD0;
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

#if PLATFORM_GPT_USE_GPT0
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

#if PLATFORM_GPT_USE_GPT0 == TRUE
  /* Driver initialization.*/
  gptObjectInit(&GPTD0);
  GPTD0.driver = TC0;
  gpt_lld_stop_timer(&GPTD0);
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
#if PLATFORM_GPT_USE_GPT0 == TRUE
    if (&GPTD0 == gptp) {
        // First enable the clock of the timer
        pmc_enable_periph_clk(ID_TC0);
        // Enable the NVIC
        nvicEnableVector(TC0_NVIC_NUMBER, GPT_NVIC_PRIORITY);
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
#endif
  }
  /* Configures the peripheral.*/

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

    /* Disables the peripheral.*/
#if PLATFORM_GPT_USE_GPT0 == TRUE
    if (&GPTD0 == gptp) {
        // Disable write protection
        gptp->driver->TC_WPMR = TC_WPMR_WPKEY_PASSWD;
        // Disable interrupts and reset the status register
        /*  Disable TC clock. */
        gptp->driver->TC_CHANNEL[gptp->channel].TC_CCR = TC_CCR_CLKDIS;
        /*  Disable interrupts. */
        gptp->driver->TC_CHANNEL[gptp->channel].TC_IDR = 0xFFFFFFFF;
        /*  Clear status register. */
        gptp->driver->TC_CHANNEL[gptp->channel].TC_SR;
        // Disable the NVIC
        nvicDisableVector(TC0_NVIC_NUMBER);
        // Disable the clock of the timer
        pmc_disable_periph_clk(ID_TC0);
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
