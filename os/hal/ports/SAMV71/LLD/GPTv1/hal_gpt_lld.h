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
 * @file    hal_gpt_lld.h
 * @brief   SAMV71 GPT subsystem low level driver header.
 *
 * @addtogroup GPT
 * @{
 */

#ifndef HAL_GPT_LLD_H
#define HAL_GPT_LLD_H

#if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define GPT_MAIN_CLK (SystemCoreClock / 2)
//also uses PCK6, with ProgrammableClock[6], if closer to the target frequency

// NOTE: USART has CORTEX_MIN_KERNEL_PRIORITY-1, SYSTICK has CORTEX_MAX_KERNEL_PRIORITY+1
#define GPT_NVIC_PRIORITY CORTEX_MIN_KERNEL_PRIORITY-2
#define TC0_HANDLER Vector9C
#define TC0_NVIC_NUMBER TC0_IRQn
#define TC1_HANDLER VectorA0
#define TC1_NVIC_NUMBER TC1_IRQn
#define TC2_HANDLER VectorA4
#define TC2_NVIC_NUMBER TC2_IRQn
#define TC3_HANDLER VectorA8
#define TC3_NVIC_NUMBER TC3_IRQn
#define TC4_HANDLER VectorAC
#define TC4_NVIC_NUMBER TC4_IRQn
#define TC5_HANDLER VectorB0
#define TC5_NVIC_NUMBER TC5_IRQn
#define TC6_HANDLER VectorFC
#define TC6_NVIC_NUMBER TC6_IRQn
#define TC7_HANDLER Vector100
#define TC7_NVIC_NUMBER TC7_IRQn
#define TC8_HANDLER Vector104
#define TC8_NVIC_NUMBER TC8_IRQn
#define TC9_HANDLER Vector108
#define TC9_NVIC_NUMBER TC9_IRQn
#define TC10_HANDLER Vector10C
#define TC10_NVIC_NUMBER TC10_IRQn
#define TC11_HANDLER Vector110
#define TC11_NVIC_NUMBER TC11_IRQn

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SAMV71 configuration options
 * @{
 */
/**
 * @brief   GPTD0 driver enable switch.
 * @details If set to @p TRUE the support for GPTD0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT0) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT0               FALSE
#endif

/**
 * @brief   GPTD1 driver enable switch.
 * @details If set to @p TRUE the support for GPTD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT1) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT1               FALSE
#endif

/**
 * @brief   GPTD2 driver enable switch.
 * @details If set to @p TRUE the support for GPTD2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT2) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT2               FALSE
#endif

/**
 * @brief   GPTD3 driver enable switch.
 * @details If set to @p TRUE the support for GPTD3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT3) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT3               FALSE
#endif

/**
 * @brief   GPTD4 driver enable switch.
 * @details If set to @p TRUE the support for GPTD4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT4) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT4               FALSE
#endif
/** @} */

/**
 * @brief   GPTD5 driver enable switch.
 * @details If set to @p TRUE the support for GPTD5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT5) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT5               FALSE
#endif
/** @} */

/**
 * @brief   GPTD6 driver enable switch.
 * @details If set to @p TRUE the support for GPTD6 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT6) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT6               FALSE
#endif
/** @} */

/**
 * @brief   GPTD7 driver enable switch.
 * @details If set to @p TRUE the support for GPTD7 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT7) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT7               FALSE
#endif
/** @} */

/**
 * @brief   GPTD8 driver enable switch.
 * @details If set to @p TRUE the support for GPTD8 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT8) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT8               FALSE
#endif
/** @} */

/**
 * @brief   GPTD9 driver enable switch.
 * @details If set to @p TRUE the support for GPTD9 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT9) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT9               FALSE
#endif
/** @} */

/**
 * @brief   GPTD10 driver enable switch.
 * @details If set to @p TRUE the support for GPTD10 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT10) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT10               FALSE
#endif
/** @} */

/**
 * @brief   GPTD11 driver enable switch.
 * @details If set to @p TRUE the support for GPTD11 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_GPT_USE_GPT11) || defined(__DOXYGEN__)
#define SAMV71_GPT_USE_GPT11               FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !OSAL_IRQ_IS_VALID_PRIORITY(GPT_NVIC_PRIORITY)
#error "Invalid GPT interrupt priority"
/* The priority value is out of range or higher than the base priority used
 * for critical sections in the kernel(CORTEX_MAX_KERNEL_PRIORITY).
 */
#endif

#if SAMV71_GPT_USE_GPT0 && !defined(ID_TC0)
#error "GPT0 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT1 && !defined(ID_TC1)
#error "GPT1 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT2 && !defined(ID_TC2)
#error "GPT2 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT3 && !defined(ID_TC3)
#error "GPT3 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT4 && !defined(ID_TC4)
#error "GPT4 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT5 && !defined(ID_TC5)
#error "GPT5 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT6 && !defined(ID_TC6)
#error "GPT6 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT7 && !defined(ID_TC7)
#error "GPT7 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT8 && !defined(ID_TC8)
#error "GPT8 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT9 && !defined(ID_TC9)
#error "GPT9 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT10 && !defined(ID_TC10)
#error "GPT10 is not present on this device"
#endif
#if SAMV71_GPT_USE_GPT11 && !defined(ID_TC11)
#error "GPT11 is not present on this device"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   GPT frequency type.
 */
typedef uint32_t gptfreq_t;

/**
 * @brief   GPT counter type.
 */
typedef uint16_t gptcnt_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  gptfreq_t                 frequency;
  /**
   * @brief   Timer callback pointer.
   * @note    This callback is invoked on GPT counter events.
   */
  gptcallback_t             callback;
  /* End of the mandatory fields.*/
} GPTConfig;

/**
 * @brief   Structure representing a GPT driver.
 */
struct GPTDriver {
  /**
   * @brief Driver state.
   */
  gptstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const GPTConfig           *config;
#if defined(GPT_DRIVER_EXT_FIELDS)
  GPT_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /* Pointer to the driver instance */
  Tc*                       driver;
  /* Index to the channel used for this GPT */
  uint8_t                   channel;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Changes the interval of GPT peripheral.
 * @details This function changes the interval of a running GPT unit.
 * @pre     The GPT unit must have been activated using @p gptStart().
 * @pre     The GPT unit must have been running in continuous mode using
 *          @p gptStartContinuous().
 * @post    The GPT unit interval is changed to the new value.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @param[in] interval  new cycle time in timer ticks
 * @notapi
 */
#define gpt_lld_change_interval(gptp, interval) (gptp->driver->TC_CHANNEL[gptp->channel].TC_RC = (gptcnt_t)interval)

#define gpt_lld_get_interval(gptp) ((gptcnt_t)gptp->driver->TC_CHANNEL[gptp->channel].TC_RC)

#define gpt_lld_get_counter(gptp) ((gptcnt_t)gptp->driver->TC_CHANNEL[gptp->channel].TC_CV)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (SAMV71_GPT_USE_GPT0 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD0;
#endif
#if (SAMV71_GPT_USE_GPT1 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD1;
#endif
#if (SAMV71_GPT_USE_GPT2 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD2;
#endif
#if (SAMV71_GPT_USE_GPT3 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD3;
#endif
#if (SAMV71_GPT_USE_GPT4 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD4;
#endif
#if (SAMV71_GPT_USE_GPT5 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD5;
#endif
#if (SAMV71_GPT_USE_GPT6 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD6;
#endif
#if (SAMV71_GPT_USE_GPT7 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD7;
#endif
#if (SAMV71_GPT_USE_GPT8 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD8;
#endif
#if (SAMV71_GPT_USE_GPT9 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD9;
#endif
#if (SAMV71_GPT_USE_GPT10 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD10;
#endif
#if (SAMV71_GPT_USE_GPT11 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD11;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void gpt_lld_init(void);
  void gpt_lld_start(GPTDriver *gptp);
  void gpt_lld_stop(GPTDriver *gptp);
  void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval);
  void gpt_lld_stop_timer(GPTDriver *gptp);
  void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_GPT == TRUE */

#endif /* HAL_GPT_LLD_H */

/** @} */
