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
 * @file    hal_pwm_lld.c
 * @brief   SAMV71 PWM subsystem low level driver source.
 *
 * @addtogroup PWM
 * @{
 */

#include "hal.h"

#if (HAL_USE_PWM == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PWMD0 synchronous channels 0-3 and asynchronous channel 0 driver identifier.
 * @note    The driver PWMD0 allocates the configured channels of PWM0 when enabled.
 */
#if (SAMV71_PWM_USE_PWM0 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD0;
#endif
/**
 * @brief   PWMD0 asynchronous channel 1 driver identifier.
 * @note    The driver PWMD0C1 allocates channel 1 of PWM0 when enabled.
 */
#if (SAMV71_PWM_USE_PWM0 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD0C1;
#endif
/**
 * @brief   PWMD0 asynchronous channel 2 driver identifier.
 * @note    The driver PWMD0C2 allocates channel 2 of PWM0 when enabled.
 */
#if (SAMV71_PWM_USE_PWM0 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD0C2;
#endif
/**
 * @brief   PWMD0 asynchronous channel 3 driver identifier.
 * @note    The driver PWMD0C3 allocates channel 3 of PWM0 when enabled.
 */
#if (SAMV71_PWM_USE_PWM0 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD0C3;
#endif
/**
 * @brief   PWMD1 synchronous channels 0-3 and asynchronous channel 0 driver identifier.
 * @note    The driver PWMD1 allocates the configured channels of PWM1 when enabled.
 */
#if (SAMV71_PWM_USE_PWM1 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD1;
#endif
/**
 * @brief   PWMD1 asynchronous channel 1 driver identifier.
 * @note    The driver PWMD1C1 allocates channel 1 ofPWM1 when enabled.
 */
#if (SAMV71_PWM_USE_PWM1 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD1C1;
#endif
/**
 * @brief   PWMD1 asynchronous channel 2 driver identifier.
 * @note    The driver PWMD1C2 allocates channel 2 ofPWM1 when enabled.
 */
#if (SAMV71_PWM_USE_PWM1 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD1C2;
#endif
/**
 * @brief   PWMD1 asynchronous channel 3 driver identifier.
 * @note    The driver PWMD1C3 allocates channel 3 of PWM1 when enabled.
 */
#if (SAMV71_PWM_USE_PWM1 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD1C3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void pwm_lld_serve_pwm_interrupt(PWMDriver *pwmp,
                                 PWMDriver *pwmpc1,
                                 PWMDriver *pwmpc2,
                                 PWMDriver *pwmpc3) {

  uint32_t isr1 = pwmp->device->PWM_ISR1;
  uint32_t isr2 = pwmp->device->PWM_ISR2;
  uint32_t imr1 = pwmp->device->PWM_IMR1;
  uint32_t imr2 = pwmp->device->PWM_IMR2;
  isr1 &= imr1;
  isr2 &= imr2;
  //These are channel updates, that is, counter resets
  if ((isr1 & PWM_ISR1_CHID0) != 0) {
    if (pwmp->config->callback)
      pwmp->config->callback(pwmp);
  }
  if ((isr1 & PWM_ISR1_CHID1) != 0) {
    if (pwmpc1->config->callback)
      pwmpc1->config->callback(pwmpc1);
  }
  if ((isr1 & PWM_ISR1_CHID2) != 0) {
    if (pwmpc2->config->callback)
      pwmpc2->config->callback(pwmpc2);
  }
  if ((isr1 & PWM_ISR1_CHID3) != 0) {
    if (pwmpc3->config->callback)
      pwmpc3->config->callback(pwmpc3);
  }
  for(int i = 0; i < 3; i++) {
    if ((isr2 & (PWM_ISR2_CMPM0 << i)) != 0) {
      if ((pwmp->enabled & (1 << i)) != 0 &&
          pwmp->config->channels[i].callback)
        pwmp->config->channels[i].callback(pwmp);
      if(i == 1 && (pwmpc1->channels & 1) != 0 &&
          pwmpc1->config->channels[0].callback)
        pwmpc1->config->channels[0].callback(pwmpc1);
      if(i == 2 && (pwmpc2->enabled & 1) != 0 &&
          pwmpc2->config->channels[0].callback)
        pwmpc2->config->channels[0].callback(pwmpc2);
      if(i == 3 && (pwmpc3->enabled & 1) != 0 &&
          pwmpc3->config->channels[0].callback)
        pwmpc3->config->channels[0].callback(pwmpc3);
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (SAMV71_PWM_USE_PWM0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PWM0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    pwm_lld_serve_pwm_interrupt(&PWMD0, &PWMD0C1, &PWMD0C2, &PWMD0C3);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_PWM_USE_PWM1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PWM1_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    pwm_lld_serve_pwm_interrupt(&PWMD1, &PWMD1C1, &PWMD1C2, &PWMD1C3);
    OSAL_IRQ_EPILOGUE();
}
#endif


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {

#if SAMV71_PWM_USE_PWM0 == TRUE
  /* Driver initialization.*/
  pwmObjectInit(&PWMD0);
  PWMD0.channels = 4;
  PWMD0.base_channel = 0;
  PWMD0.device = PWM0;
  pwmObjectInit(&PWMD0C1);
  PWMD0C1.channels = 1;
  PWMD0C1.base_channel = 1;
  PWMD0C1.device = PWM0;
  pwmObjectInit(&PWMD0C2);
  PWMD0C2.channels = 1;
  PWMD0C2.base_channel = 2;
  PWMD0C2.device = PWM0;
  pwmObjectInit(&PWMD0C3);
  PWMD0C3.channels = 1;
  PWMD0C3.base_channel = 3;
  PWMD0C3.device = PWM0;
  PWM0->PWM_CLK = PWM_CLK_DIVA_CLKA_POFF | PWM_CLK_DIVB_CLKB_POFF;
#endif
#if SAMV71_PWM_USE_PWM1 == TRUE
  /* Driver initialization.*/
  pwmObjectInit(&PWMD1);
  PWMD1.channels = 4;
  PWMD1.base_channel = 0;
  PWMD1.device = PWM1;
  pwmObjectInit(&PWMD1C1);
  PWMD1C1.channels = 1;
  PWMD1C1.base_channel = 1;
  PWMD1C1.device = PWM1;
  pwmObjectInit(&PWMD1C2);
  PWMD1C2.channels = 1;
  PWMD1C2.base_channel = 2;
  PWMD1C2.device = PWM1;
  pwmObjectInit(&PWMD1C3);
  PWMD1C3.channels = 1;
  PWMD1C3.base_channel = 3;
  PWMD1C3.device = PWM1;
  PWM1->PWM_CLK = PWM_CLK_DIVA_CLKA_POFF | PWM_CLK_DIVB_CLKB_POFF;
#endif
}

/**
 * @brief   Configures and activates the PWM peripheral.
 * @note    Starting a driver that is already in the @p PWM_READY state
 *          disables all the active channels.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_start(PWMDriver *pwmp) {

  if (pwmp->state == PWM_STOP) {
    /* Clock activation and timer reset.*/
    if(false) {
    }
#if SAMV71_PWM_USE_PWM0 == TRUE
    else if (&PWMD0 == pwmp) {
      osalDbgAssert((pwmp->config->synchronous_channel_mask & 2) == 0 ||
                    PWMD0C1.state != PWM_READY,
                    "Cannot set channel as synchronous that is already active asynchronous");
      osalDbgAssert((pwmp->config->synchronous_channel_mask & 4) == 0 ||
                    PWMD0C2.state != PWM_READY,
                    "Cannot set channel as synchronous that is already active asynchronous");
      osalDbgAssert((pwmp->config->synchronous_channel_mask & 8) == 0 ||
                    PWMD0C3.state != PWM_READY,
                    "Cannot set channel as synchronous that is already active asynchronous");
      /* Enable interrupt */
      nvicEnableVector(PWM0_NVIC_NUMBER, PWM_NVIC_PRIORITY);
      /* Enable clock source */
      pmc_enable_periph_clk(ID_PWM0);
    }
    else if (&PWMD0C1 == pwmp) {
      osalDbgAssert(PWMD0.state != PWM_READY ||
                    (PWMD0.config->synchronous_channel_mask & 2) == 0,
                    "Cannot set channel as synchronous that is already active asynchronous");
      /* Enable interrupt */
      nvicEnableVector(PWM0_NVIC_NUMBER, PWM_NVIC_PRIORITY);
      /* Enable clock source */
      pmc_enable_periph_clk(ID_PWM0);
    }
    else if (&PWMD0C2 == pwmp) {
      osalDbgAssert(PWMD0.state != PWM_READY ||
                    (PWMD0.config->synchronous_channel_mask & 4) == 0,
                    "Cannot set channel as synchronous that is already active asynchronous");
      /* Enable interrupt */
      nvicEnableVector(PWM0_NVIC_NUMBER, PWM_NVIC_PRIORITY);
      /* Enable clock source */
      pmc_enable_periph_clk(ID_PWM0);
    }
    else if (&PWMD0C3 == pwmp) {
      osalDbgAssert(PWMD0.state != PWM_READY ||
                    (PWMD0.config->synchronous_channel_mask & 8) == 0,
                    "Cannot set channel as synchronous that is already active asynchronous");
      /* Enable interrupt */
      nvicEnableVector(PWM0_NVIC_NUMBER, PWM_NVIC_PRIORITY);
      /* Enable clock source */
      pmc_enable_periph_clk(ID_PWM0);
    }
#endif
#if SAMV71_PWM_USE_PWM1 == TRUE
    else if (&PWMD1 == pwmp) {
      osalDbgAssert((pwmp->config->synchronous_channel_mask & 2) == 0 ||
                    PWMD1C1.state != PWM_READY,
                    "Cannot set channel as synchronous that is already active asynchronous");
      osalDbgAssert((pwmp->config->synchronous_channel_mask & 4) == 0 ||
                    PWMD1C2.state != PWM_READY,
                    "Cannot set channel as synchronous that is already active asynchronous");
      osalDbgAssert((pwmp->config->synchronous_channel_mask & 8) == 0 ||
                    PWMD1C3.state != PWM_READY,
                    "Cannot set channel as synchronous that is already active asynchronous");
      /* Enable interrupt */
      nvicEnableVector(PWM1_NVIC_NUMBER, PWM_NVIC_PRIORITY);
      /* Enable clock source */
      pmc_enable_periph_clk(ID_PWM1);
    }
    else if (&PWMD1C1 == pwmp) {
      osalDbgAssert(PWMD1.state != PWM_READY ||
                    (PWMD1.config->synchronous_channel_mask & 2) == 0,
                    "Cannot set channel as synchronous that is already active asynchronous");
      /* Enable interrupt */
      nvicEnableVector(PWM1_NVIC_NUMBER, PWM_NVIC_PRIORITY);
      /* Enable clock source */
      pmc_enable_periph_clk(ID_PWM1);
    }
    else if (&PWMD1C2 == pwmp) {
      osalDbgAssert(PWMD1.state != PWM_READY ||
                    (PWMD1.config->synchronous_channel_mask & 4) == 0,
                    "Cannot set channel as synchronous that is already active asynchronous");
      /* Enable interrupt */
      nvicEnableVector(PWM1_NVIC_NUMBER, PWM_NVIC_PRIORITY);
      /* Enable clock source */
      pmc_enable_periph_clk(ID_PWM1);
    }
    else if (&PWMD1C3 == pwmp) {
      osalDbgAssert(PWMD1.state != PWM_READY ||
                    (PWMD1.config->synchronous_channel_mask & 8) == 0,
                    "Cannot set channel as synchronous that is already active asynchronous");
      /* Enable interrupt */
      nvicEnableVector(PWM1_NVIC_NUMBER, PWM_NVIC_PRIORITY);
      /* Enable clock source */
      pmc_enable_periph_clk(ID_PWM1);
    }
#endif
    else {
      osalDbgAssert(false, "invalid PWM instance");
    }

    if(pwmp->base_channel == 0) {
      pwmp->device->PWM_SCM = pwmp->config->synchronous_channel_mask |
                              PWM_SCM_UPDM_MODE0;

      uint32_t oov = pwmp->device->PWM_OOV;
      oov &= ~((pwmp->config->synchronous_channel_mask << 16) |
               pwmp->config->synchronous_channel_mask |
               0x10001);
      pwmp->device->PWM_OOV = oov;
      pwmp->device->PWM_OSC = (pwmp->config->synchronous_channel_mask << 16) |
               pwmp->config->synchronous_channel_mask |
               0x10001;
    } else {
      uint32_t oov = pwmp->device->PWM_OOV;
      oov &= ~(0x10001 << pwmp->base_channel);
      pwmp->device->PWM_OOV = oov;
      pwmp->device->PWM_OSC = 0x10001 << pwmp->base_channel;
    }
    for (int i = 0; i < 4; i++) {
      if ((pwmp->config->synchronous_channel_mask & (1 << i)) != 0 || i == 0) {
        pwmp->device->PWM_CMP[pwmp->base_channel + i].PWM_CMPM = 0;
        pwmp->device->PWM_CH_NUM[pwmp->base_channel + i].PWM_DT = 0;
        uint32_t cmr = PWM_CMR_DTE;
        if (pwmp->config->channels[i].mode == PWM_OUTPUT_ACTIVE_LOW) {
          cmr |= PWM_CMR_CPOL;
        }
        pwmp->device->PWM_CH_NUM[pwmp->base_channel + i].PWM_CMR = cmr;
      }
    }

    //TODO this does not work. it only works on an already enabled channel.
    pwm_lld_change_period(pwmp, pwmp->config->period);
    //TODO this does set the channels period, at least. so, we may want to have
    //pwm_lld_change_period check for enabled, and set it directly otherwise.
    //same for the duty cycle.
    //pwmp->device->PWM_CH_NUM[pwmp->base_channel].PWM_CPRD = PWM_CPRD_CPRD(pwmp->config->period);

    //PWM_CLK can do CLKA/CLKB integer+shift prescalers
    //to make them maximally compatible, we need to select the smallest possible shift.
    //this assumes the user knows about the limitation and selects frequency accordingly.

    unsigned int divider = PWM_MAIN_CLOCK / pwmp->config->frequency;
    osalDbgAssert(PWM_MAIN_CLOCK == pwmp->config->frequency * divider,
                  "PWM frequency must be integer divider of the main clock");
    unsigned int shift = 0;
    while(divider > 255) {
      osalDbgAssert((divider & 1) == 0, "PWM frequency must be main clock shifted and divided by up to 255");
      divider /= 2;
      shift++;
    }
    while(divider > 1 && (divider & 1) == 0) {
      divider /= 2;
      shift++;
    }
    //now we maximized shift, minimized divider

    uint32_t cmr = pwmp->device->PWM_CH_NUM[pwmp->base_channel].PWM_CMR;
    cmr &= ~(PWM_CMR_CPRE_Msk);

    if(divider == 1 && shift <= 10) {
      //Just use a prescaler from main clock. no need to configure CLKA or CLKB.
      cmr |= PWM_CMR_CPRE(shift);
    } else {
      //need one of CLKA or CLKB, but maybe we can reuse one. first, make sure
      //shift is in range.
      if(shift > 10) {
        divider <<= shift-10;
        shift =  10;
        osalDbgAssert(divider <= 255, "Division from main clock can maximum be 261120");
      }
      uint32_t clk_setting = PWM_CLK_DIVA(divider) | PWM_CLK_PREA(shift);
      if ((pwmp->device->PWM_CLK & (PWM_CLK_DIVA_Msk | PWM_CLK_PREA_Msk)) == clk_setting) {
        cmr |= PWM_CMR_CPRE_CLKA;
      } else if ((pwmp->device->PWM_CLK & (PWM_CLK_DIVB_Msk | PWM_CLK_PREB_Msk)) == (clk_setting << 16)) {
        cmr |= PWM_CMR_CPRE_CLKB;
      } else if ((pwmp->device->PWM_CLK & PWM_CLK_DIVA_Msk) == PWM_CLK_DIVA_CLKA_POFF) {
        cmr |= PWM_CMR_CPRE_CLKA;
        uint32_t clk = pwmp->device->PWM_CLK;
        clk &= ~(PWM_CLK_DIVA_Msk | PWM_CLK_PREA_Msk);
        clk |= clk_setting;
        pwmp->device->PWM_CLK = clk;
      } else if ((pwmp->device->PWM_CLK & PWM_CLK_DIVB_Msk) == PWM_CLK_DIVB_CLKB_POFF) {
        cmr |= PWM_CMR_CPRE_CLKB;
        uint32_t clk = pwmp->device->PWM_CLK;
        clk &= ~(PWM_CLK_DIVB_Msk | PWM_CLK_PREB_Msk);
        clk |= clk_setting << 16;
        pwmp->device->PWM_CLK = clk;
      } else {
        osalDbgAssert(false, "Could not find free clock to satisfy shift/divider demand");
      }
    }

    pwmp->device->PWM_CH_NUM[pwmp->base_channel].PWM_CMR = cmr;


  }
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmp) {

  /* If in ready state then disables the PWM clock.*/
  if (pwmp->state == PWM_READY) {
    for(int i = 0; i < 4; i++) {
      if((pwmp->enabled & (1 << i)) != 0) {
        pwm_lld_disable_channel(pwmp, i);
      }
    }
    pwm_lld_disable_periodic_notification(pwmp);

    if(false) {
    }
#if SAMV71_PWM_USE_PWM0 == TRUE
    else if (&PWMD0 == pwmp) {
      if (PWMD0C1.state == PWM_STOP &&
          PWMD0C2.state == PWM_STOP &&
          PWMD0C3.state == PWM_STOP) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_PWM0);
        /* Disable interrupt */
        nvicDisableVector(PWM0_NVIC_NUMBER);
      }
    } else if (&PWMD0C1 == pwmp) {
      if (PWMD0.state == PWM_STOP &&
          PWMD0C2.state == PWM_STOP &&
          PWMD0C3.state == PWM_STOP) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_PWM0);
        /* Disable interrupt */
        nvicDisableVector(PWM0_NVIC_NUMBER);
      }
    } else if (&PWMD0C2 == pwmp) {
      if (PWMD0.state == PWM_STOP &&
          PWMD0C1.state == PWM_STOP &&
          PWMD0C3.state == PWM_STOP) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_PWM0);
        /* Disable interrupt */
        nvicDisableVector(PWM0_NVIC_NUMBER);
      }
    } else if (&PWMD0C3 == pwmp) {
      if (PWMD0.state == PWM_STOP &&
          PWMD0C1.state == PWM_STOP &&
          PWMD0C2.state == PWM_STOP) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_PWM0);
        /* Disable interrupt */
        nvicDisableVector(PWM0_NVIC_NUMBER);
      }
    }
#endif
#if SAMV71_PWM_USE_PWM1 == TRUE
    else if (&PWMD1 == pwmp) {
      if (PWMD1C1.state == PWM_STOP &&
          PWMD1C2.state == PWM_STOP &&
          PWMD1C3.state == PWM_STOP) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_PWM1);
        /* Disable interrupt */
        nvicDisableVector(PWM1_NVIC_NUMBER);
      }
    } else if (&PWMD1C1 == pwmp) {
      if (PWMD1.state == PWM_STOP &&
          PWMD1C2.state == PWM_STOP &&
          PWMD1C3.state == PWM_STOP) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_PWM1);
        /* Disable interrupt */
        nvicDisableVector(PWM1_NVIC_NUMBER);
      }
    } else if (&PWMD1C2 == pwmp) {
      if (PWMD1.state == PWM_STOP &&
          PWMD1C1.state == PWM_STOP &&
          PWMD1C3.state == PWM_STOP) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_PWM1);
        /* Disable interrupt */
        nvicDisableVector(PWM1_NVIC_NUMBER);
      }
    } else if (&PWMD1C3 == pwmp) {
      if (PWMD1.state == PWM_STOP &&
          PWMD1C1.state == PWM_STOP &&
          PWMD1C2.state == PWM_STOP) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_PWM1);
        /* Disable interrupt */
        nvicDisableVector(PWM1_NVIC_NUMBER);
      }
    }
#endif
    else {
      osalDbgAssert(false, "invalid PWM instance");
    }
  }
}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 * @note    The function has effect at the next cycle start.
 * @note    Channel notification is not enabled.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @notapi
 */
void pwm_lld_enable_channel(PWMDriver *pwmp,
                            pwmchannel_t channel,
                            pwmcnt_t width) {

  pwmChangeChannel(pwmp, channel, width);
  if (pwmp->config->channels[channel].mode != PWM_OUTPUT_DISABLED) {
    pwmp->device->PWM_ENA = PWM_ENA_CHID0 << (channel + pwmp->base_channel);
    if (!pwmp->config->manual_updates)
      pwmTriggerSynchUpdate(pwmp);
  }
}

/**
 * @brief   Disables a PWM channel and its notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel) {

  pwmp->device->PWM_DIS = PWM_DIS_CHID0 << (channel + pwmp->base_channel);
  pwm_lld_disable_channel_notification(pwmp, channel);
}

/**
 * @brief   Enables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_enable_periodic_notification(PWMDriver *pwmp) {

  pwmp->device->PWM_IER1 = PWM_IER1_CHID0 << pwmp->base_channel;
}

/**
 * @brief   Disables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_disable_periodic_notification(PWMDriver *pwmp) {

  pwmp->device->PWM_IDR1 = PWM_IDR1_CHID0 << pwmp->base_channel;
}

/**
 * @brief   Enables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                         pwmchannel_t channel) {

  pwmp->device->PWM_IER2 = PWM_IER2_CMPM0 << (channel + pwmp->base_channel);
  pwmp->device->PWM_CMP[channel + pwmp->base_channel].PWM_CMPM |= PWM_CMPM_CEN;
}

/**
 * @brief   Disables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel_notification(PWMDriver *pwmp,
                                          pwmchannel_t channel) {

  pwmp->device->PWM_CMP[channel + pwmp->base_channel].PWM_CMPM &= ~PWM_CMPM_CEN;
  pwmp->device->PWM_IDR2 = PWM_IDR2_CMPM0 << (channel + pwmp->base_channel);
}

/**
 * @brief   Sets the output drive forced state setting
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    The actual direction has effect immediately, the fact if it is
 *          overriden has effect at the next cycle start.
 *
 * @param[in] pwmp          pointer to a @p PWMDriver object
 * @param[in] channel       PWM channel identifier (0...channels-1)
 * @param[in] force_setting The force setting
 */
void pwmSetChannelForceMode(PWMDriver *pwmp,
                        pwmchannel_t channel,
                        pwmchannelforce_t force_setting) {
  osalDbgAssert(channel <= pwmp->channels, "Channel number out of range");
  channel += pwmp->base_channel;
  uint32_t os_set = (force_setting & (PWM_OS_OSH0 | PWM_OS_OSL0)) << channel;
  uint32_t os_clear = (~force_setting & (PWM_OS_OSH0 | PWM_OS_OSL0)) << channel;
  pwmp->os_reg |= os_set;
  pwmp->os_reg &= ~os_clear;
  os_set = pwmp->os_reg & ~pwmp->device->PWM_OS;
  os_clear = ~pwmp->os_reg & pwmp->device->PWM_OS;
  //avoid changing the output override value when it is not being used
  //this avoids toggling the value when it probably still is in use, but no
  //value is set from the force setting, while the selection only takes effect
  //later
  uint32_t oov_mask = os_set;
  uint32_t oov = pwmp->device->PWM_OOV;
  oov &= ~oov_mask;
  oov |= (force_setting >> (8-channel)) & oov_mask;
  pwmp->device->PWM_OOV = oov;
  if(pwmIsChannelEnabled(pwmp,channel)) {
    //It seems like the device does not like to have both OSSUPD and OSCUPD set in the same cycle
    if (os_set)
      pwmp->device->PWM_OSSUPD = os_set;
    if (os_clear && !os_set)
      pwmp->device->PWM_OSCUPD = os_clear;
  } else {
    pwmp->device->PWM_OSS = os_set;
    pwmp->device->PWM_OSC = os_clear;
  }
}

/**
 * @brief   Sets the output channel mode
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] mode      The new mode
 */
void pwmSetChannelMode(PWMDriver *pwmp,
                       pwmchannel_t channel,
                       pwmmode_t mode) {
  osalDbgCheck(channel < pwmp->channels);
  osalDbgCheck(channel < 4);
  //This should allow the compiler to optimize the register access to
  //offset+channel*0x20, since all relevant cases follow that pattern.
  if(channel >= 4)
    return;
  if (pwmIsChannelEnabled(pwmp,channel)) {
    volatile uint32_t *reg = NULL;
    switch (channel) {
      /*
      case 0:
        reg = &pwmp->device->PWM_CMUPD0;
        break;
      case 1:
        reg = &pwmp->device->PWM_CMUPD1;
        break;
      case 2:
        reg = &pwmp->device->PWM_CMUPD2;
        break;
      case 3:
        reg = &pwmp->device->PWM_CMUPD3;
        break;
        */
      default:
        reg = (&pwmp->device->PWM_CMUPD0) + channel*(0x20/sizeof(uint32_t));
        break;
    }
    if (mode == PWM_OUTPUT_ACTIVE_LOW) {
      *reg |= PWM_CMUPD0_CPOLUP;
    } else {
      *reg &= ~PWM_CMUPD0_CPOLUP;
    }
    if (!pwmp->config->manual_updates)
      pwmTriggerSynchUpdate(pwmp);
  } else {
    if (mode == PWM_OUTPUT_ACTIVE_LOW) {
      pwmp->device->PWM_CH_NUM[channel + pwmp->base_channel].PWM_CMR |=
        PWM_CMR_CPOL;
    } else {
      pwmp->device->PWM_CH_NUM[channel + pwmp->base_channel].PWM_CMR &=
        ~PWM_CMR_CPOL;
    }
  }
}



#endif /* HAL_USE_PWM == TRUE */

/** @} */
