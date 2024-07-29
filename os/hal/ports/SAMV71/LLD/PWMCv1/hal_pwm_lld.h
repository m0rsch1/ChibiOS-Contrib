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
 * @file    hal_pwm_lld.h
 * @brief   SAMV71 PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef HAL_PWM_LLD_H
#define HAL_PWM_LLD_H

#if (HAL_USE_PWM == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Number of PWM channels per PWM driver.
 */
#define PWM_CHANNELS                            4

#define PWM_MAIN_CLOCK (SystemCoreClock / 2)
#define PWM_NVIC_PRIORITY CORTEX_MIN_KERNEL_PRIORITY-1

#if defined(__SAMV71Q21B__)
#define PWM0_NVIC_NUMBER PWM0_IRQn
#define PWM0_HANDLER VectorBC
#define PWM1_NVIC_NUMBER PWM1_IRQn
#define PWM1_HANDLER Vector130
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SAMV71 configuration options
 * @{
 */
/**
 * @brief   PWMD0 driver enable switch.
 * @details If set to @p TRUE the support for PWM1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_PWM_USE_PWM0) || defined(__DOXYGEN__)
#define SAMV71_PWM_USE_PWM0                  FALSE
#endif
/**
 * @brief   PWMD1 driver enable switch.
 * @details If set to @p TRUE the support for PWM1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_PWM_USE_PWM1) || defined(__DOXYGEN__)
#define SAMV71_PWM_USE_PWM1                  FALSE
#endif
/** @} */

/*===========================================================================*/
/* Configuration checks.                                                     */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a PWM mode.
 */
typedef uint32_t pwmmode_t;

/**
 * @brief   Type of a PWM channel.
 */
typedef uint8_t pwmchannel_t;

/**
 * @brief   Type of a channels mask.
 */
typedef uint32_t pwmchnmsk_t;

/**
 * @brief   Type of a PWM counter.
 */
typedef uint32_t pwmcnt_t;

/**
 * @brief   Type of a PWM driver channel configuration structure.
 */
typedef struct {
  /**
   * @brief Channel active logic level.
   */
  pwmmode_t                 mode;
  /**
   * @brief Channel callback pointer.
   * @note  This callback is invoked on the channel compare event. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /* End of the mandatory fields.*/
} PWMChannelConfig;

/**
 * @brief   Type of a PWM driver configuration structure.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  uint32_t                  frequency;
  /**
   * @brief   PWM period in ticks.
   * @note    The low level can use assertions in order to catch invalid
   *          period specifications.
   */
  pwmcnt_t                  period;
  /**
   * @brief Periodic callback pointer.
   * @note  This callback is invoked on PWM counter reset. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /**
   * @brief Channels configurations.
   */
  PWMChannelConfig          channels[PWM_CHANNELS];
  /* End of the mandatory fields.*/
  /**
   * @brief Bit mask for selecting snychronous channels
   * @note  Bits 0-3 enable synchronous operation with channels 0-3.
   * @note  Only valid on PWMDx, not valid on PWMDxCx.
   * @note  Starting a channel that is already synchronous or asynchronous
   *        started elsewhere is not allowed.
   */
  uint32_t                  synchronous_channel_mask;
  /**
   * @brief Select whether value updates should be made automatically
   * @details When this is set, all functions that update at the next
   *          cycle will update only after pwmTriggerSynchUpdate has been
   *          called.
   */
  bool manual_updates;
} PWMConfig;

/**
 * @brief   Structure representing a PWM driver.
 */
struct PWMDriver {
  /**
   * @brief Driver state.
   */
  pwmstate_t                state;
  /**
   * @brief Current driver configuration data.
   */
  const PWMConfig           *config;
  /**
   * @brief   Current PWM period in ticks.
   */
  pwmcnt_t                  period;
  /**
   * @brief   Mask of the enabled channels.
   */
  pwmchnmsk_t               enabled;
  /**
   * @brief   Number of channels in this instance.
   */
  pwmchannel_t              channels;
#if defined(PWM_DRIVER_EXT_FIELDS)
  PWM_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief The base channel of this driver, used to determine
   *        the channel providing the base period and then channel
   *        number base. Only channel 0 can be used as period provider for other
   *        channels. If it actually used as synchronous base, and for what
   *        channels, is in the configuration.
   */
  pwmchannel_t              base_channel;
  Pwm                       *device;
  uint32_t                  os_reg;
};

/**
 * @brief   Channel output forced override options
 *
 * This encodes the bits from PWM_OS and PWM_OOV. PWM_OOV shifted left by 8
 */
typedef enum {
  PWM_FORCE_NONE = 0,        /**< Both drivers follow pwm drive */
  PWM_FORCE_LOW_OFF = PWM_OS_OSL0,     /**< The low side driver is forced off */
  PWM_FORCE_HIGH_OFF = PWM_OS_OSH0,    /**< The high side driver is forced off */
  /** The low side driver is forced off, the high side driver forced on */
  PWM_FORCE_HIGH_ON_LOW_OFF = PWM_FORCE_LOW_OFF | PWM_OS_OSH0 | (PWM_OOV_OOVH0 << 8),
  /** The high side driver is forced off, the low side driver forced on */
  PWM_FORCE_HIGH_OFF_LOW_ON = PWM_FORCE_HIGH_OFF | PWM_OS_OSL0 | (PWM_OOV_OOVL0 << 8),
  /** Both drivers are forced off */
  PWM_FORCE_HIGH_OFF_LOW_OFF = PWM_FORCE_HIGH_OFF | PWM_FORCE_LOW_OFF,
} pwmchannelforce_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Checks if a channel is enabled
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @return    true if the channel is enabled
 */
#define pwmIsChannelEnabled(pwmp, channel) \
  (((pwmp)->device->PWM_SR &                                              \
      (PWM_SR_CHID0 << ((channel) + (pwmp)->base_channel))) != 0)

/**
 * @brief   Triggers a synchronous control value update
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    Makes the functions, that have effect at the next cycle start,
 *          actually have effect at the next cycle start, if manual
 *          updates have been enabled.
 * @note    Does nothing when manual updates are disabled
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 */
#define pwmTriggerSynchUpdate(pwmp)               \
  (pwmp)->device->PWM_SCUC = PWM_SCUC_UPDULOCK

/**
 * @brief   Changes the period of the PWM peripheral.
 * @details This function changes the period of a PWM unit that has already
 *          been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit period is changed to the new value.
 * @note    The function has effect at the next cycle start.
 * @note    If a period is specified that is shorter than the pulse width
 *          programmed in one of the channels then the behavior is not
 *          guaranteed.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] period    new cycle time in ticks
 *
 * @notapi
 */
#define pwm_lld_change_period(pwmp, period) do {                              \
    if(pwmIsChannelEnabled(pwmp,0)) {                         \
      (pwmp)->device->PWM_CH_NUM[(pwmp)->base_channel].PWM_CPRDUPD =          \
        PWM_CPRDUPD_CPRDUPD(period);                                          \
      if(!(pwmp)->config->manual_updates)                                     \
        pwmTriggerSynchUpdate(pwmp);                                          \
    } else {                                                                  \
      (pwmp)->device->PWM_CH_NUM[(pwmp)->base_channel].PWM_CPRD =             \
        PWM_CPRD_CPRD(period);                                                \
    }                                                                         \
  } while (0)

  /* extra user API */

/**
 * @brief   Sets the output channel width
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] width     new PWM width for the channel in ticks
 */
#define pwmChangeChannel(pwmp, channel, width) do {                            \
    if(pwmIsChannelEnabled(pwmp,channel)) {            \
      (pwmp)->device->PWM_CH_NUM[(channel)+(pwmp)->base_channel].PWM_CDTYUPD = \
        PWM_CDTYUPD_CDTYUPD(width);                                            \
      (pwmp)->device->PWM_CMP[(channel)+(pwmp)->base_channel].PWM_CMPVUPD =    \
        PWM_CMPVUPD_CVUPD(width);                                              \
      if(!(pwmp)->config->manual_updates)                                      \
        pwmTriggerSynchUpdate(pwmp);                                           \
    } else {                                                                   \
      (pwmp)->device->PWM_CH_NUM[(channel)+(pwmp)->base_channel].PWM_CDTY =    \
        PWM_CDTY_CDTY(width);                                                  \
      (pwmp)->device->PWM_CMP[(channel)+(pwmp)->base_channel].PWM_CMPV =       \
        PWM_CMPV_CV(width);                                                    \
    }                                                                          \
  } while(0)

/**
 * @brief   Sets the output channel dead time
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] l_to_h    new deadtime for transition from low to high side drive,
 *                      in ticks
 * @param[in] h_to_l    new deadtime for transition from high to low side drive,
 *                      in ticks
 */
#define pwmSetDeadtime(pwmp, channel, l_to_h, h_to_l) do {                    \
    if(pwmIsChannelEnabled(pwmp,channel)) {           \
      (pwmp)->device->PWM_CH_NUM[(channel)+(pwmp)->base_channel].PWM_DTUPD =  \
        PWM_DTUPD_DTHUPD(l_to_h) | PWM_DTUPD_DTLUPD(h_to_l);                  \
      if(!(pwmp)->config->manual_updates)                                     \
        pwmTriggerSynchUpdate(pwmp);                                          \
    } else {                                                                  \
      (pwmp)->device->PWM_CH_NUM[(channel)+(pwmp)->base_channel].PWM_DT =     \
        PWM_DT_DTH(l_to_h) | PWM_DT_DTL(h_to_l);                              \
    }                                                                         \
  } while(0)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (SAMV71_PWM_USE_PWM0 == TRUE) && !defined(__DOXYGEN__)
extern PWMDriver PWMD0;
extern PWMDriver PWMD0C1;
extern PWMDriver PWMD0C2;
extern PWMDriver PWMD0C3;
#endif
#if (SAMV71_PWM_USE_PWM1 == TRUE) && !defined(__DOXYGEN__)
extern PWMDriver PWMD1;
extern PWMDriver PWMD1C1;
extern PWMDriver PWMD1C2;
extern PWMDriver PWMD1C3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void pwm_lld_init(void);
  void pwm_lld_start(PWMDriver *pwmp);
  void pwm_lld_stop(PWMDriver *pwmp);
  void pwm_lld_enable_channel(PWMDriver *pwmp,
                              pwmchannel_t channel,
                              pwmcnt_t width);
  void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel);
  void pwm_lld_enable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_disable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                           pwmchannel_t channel);
  void pwm_lld_disable_channel_notification(PWMDriver *pwmp,
                                            pwmchannel_t channel);

  /* extra user API */
  void pwmSetChannelForceMode(PWMDriver *pwmp,
                         pwmchannel_t channel,
                         pwmchannelforce_t force_setting);
  void pwmSetChannelMode(PWMDriver *pwmp,
                         pwmchannel_t channel,
                         pwmmode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PWM == TRUE */

#endif /* HAL_PWM_LLD_H */

/** @} */
