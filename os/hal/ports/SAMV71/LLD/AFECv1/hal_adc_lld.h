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
 * @file    hal_adc_lld.h
 * @brief   SAMV71 ADC subsystem low level driver header.
 *
 * @addtogroup ADC
 * @{
 */

#ifndef HAL_ADC_LLD_H
#define HAL_ADC_LLD_H

#if (HAL_USE_ADC == TRUE) || defined(__DOXYGEN__)

#include "samv71_xdmac.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Possible ADC errors mask bits.
 * @{
 */
#define ADC_ERR_DMAFAILURE      1U  /**< DMA operations failure.            */
#define ADC_ERR_OVERFLOW        2U  /**< ADC overflow condition.            */
#define ADC_ERR_AWD             4U  /**< Watchdog triggered.                */
/** @} */

/**
 * @name    Possible ADC configuration flag mask bits
 * @{
 */
#define ADC_FLAG_NUMERIC_CHANNEL_ORDER 1U  /**< Sample in numeric channel order, ignoring sequence */
#define ADC_FLAG_USE_DMA               2U  /**< Use DMA. Else use slow irq driven capture. */
/** @} */

#define AFEC_MAIN_CLK (SystemCoreClock / 2)

#define AFEC_NVIC_PRIORITY CORTEX_MIN_KERNEL_PRIORITY

#if defined(__SAMV71Q21B__)
#define AFEC0_NVIC_NUMBER AFEC0_IRQn
#define AFEC0_HANDLER VectorB4
#define AFEC1_NVIC_NUMBER AFEC1_IRQn
#define AFEC1_HANDLER VectorE0

#define ADC_TRIGGER_AFEC0_AFE0_ADTRG 0
#define ADC_TRIGGER_AFEC1_AFE1_ADTRG 0
#define ADC_TRIGGER_AFEC0_TC0_TIOA 2
#define ADC_TRIGGER_AFEC1_TC3_TIOA 2
#define ADC_TRIGGER_AFEC0_TC1_TIOA 4
#define ADC_TRIGGER_AFEC1_TC4_TIOA 4
#define ADC_TRIGGER_AFEC0_TC2_TIOA 6
#define ADC_TRIGGER_AFEC1_TC5_TIOA 6
#define ADC_TRIGGER_AFEC0_PWM0_EVL0 8
#define ADC_TRIGGER_AFEC1_PWM1_EVL0 8
#define ADC_TRIGGER_AFEC0_PWM0_EVL1 10
#define ADC_TRIGGER_AFEC1_PWM1_EVL1 10
#define ADC_TRIGGER_ANALOG_COMPARATOR 12
#define ADC_TRIGGER_SOFTWARE 15
#endif

#define AFEC_CONFIG_MODE_MASK (AFEC_MR_STARTUP_Msk | AFEC_MR_FREERUN | \
          AFEC_MR_FWUP | AFEC_MR_SLEEP)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SAMV71 configuration options
 * @{
 */
/**
 * @brief   ADC0 driver enable switch.
 * @details If set to @p TRUE the support for ADC0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_ADC_USE_ADC0) || defined(__DOXYGEN__)
#define SAMV71_ADC_USE_ADC0                  FALSE
#endif
/**
 * @brief   ADC1 driver enable switch.
 * @details If set to @p TRUE the support for ADC1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_ADC_USE_ADC1) || defined(__DOXYGEN__)
#define SAMV71_ADC_USE_ADC1                  FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ADC sample data type.
 */
typedef uint16_t adcsample_t;

/**
 * @brief   Channels number in a conversion group.
 */
typedef uint8_t adc_channels_num_t;

/**
 * @brief   Type of an ADC error mask.
 */
typedef uint32_t adcerror_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the ADC driver structure.
 */
#define adc_lld_driver_fields                                                 \
  Afec *device;                                                               \
  uint8_t current_channel;                                                    \
  uint8_t last_channel;                                                       \
  size_t current_pos;                                                         \
  const samv71_xdmac_channel_t* dma_channel;                                  \
  uint8_t dma_descriptors_buf[sizeof(samv71_xdmac_linked_list_view_0_t)*2+31];\
  samv71_xdmac_linked_list_view_0_t *dma_descriptors

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  uint8_t flags;                                                            \
  uint16_t channel_differential;                                            \
  uint16_t channel_sh_dual;                                                 \
  uint8_t channel_gain[12];                                                 \
  uint16_t channel_offset[12];                                              \
  uint32_t speed;                                                           \
  uint32_t mode

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_configuration_group_fields                                  \
  uint64_t channel_sequence;                                                \
  uint16_t channel_enabled;                                                 \
  uint8_t trigger_selection

#define ADC_CHANNEL_ENABLED(S0,S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11) \
  ((1 << (S0)) | (1 << (S1)) | (1 << (S2)) | (1 << (S3)) | \
      (1 << (S4)) | (1 << (S5)) | (1 << (S6)) | (1 << (S7)) | (1 << (S8)) | \
      (1 << (S9)) | (1 << (S10)) | (1 << (S11)))
#define ADC_CHANNEL_POS_UNUSED 0
#define ADC_CHANNEL_SEQUENCE(S0,S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11) \
  (((uint64_t)(S0) << 0ULL) | ((uint64_t)(S1) << 4ULL) | \
   ((uint64_t)(S2) << 8ULL) | ((uint64_t)(S3) << 12ULL) | \
   ((uint64_t)(S4) << 16ULL) | ((uint64_t)(S5) << 20ULL) | \
   ((uint64_t)(S6) << 24ULL) | ((uint64_t)(S7) << 28ULL) | \
   ((uint64_t)(S8) << 32ULL) | ((uint64_t)(S9) << 36ULL) | \
   ((uint64_t)(S10) << 40ULL) | ((uint64_t)(S11) << 44ULL))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (SAMV71_ADC_USE_ADC0 == TRUE) && !defined(__DOXYGEN__)
extern ADCDriver ADCD0;
#endif

#if (SAMV71_ADC_USE_ADC1 == TRUE) && !defined(__DOXYGEN__)
extern ADCDriver ADCD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void adc_lld_init(void);
  void adc_lld_start(ADCDriver *adcp);
  void adc_lld_stop(ADCDriver *adcp);
  void adc_lld_start_conversion(ADCDriver *adcp);
  void adc_lld_stop_conversion(ADCDriver *adcp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC == TRUE */

#endif /* HAL_ADC_LLD_H */

/** @} */
