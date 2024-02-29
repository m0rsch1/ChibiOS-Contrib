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
 * @file    hal_serial_lld.h
 * @brief   SAMV71 serial subsystem low level driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

#include "samv71.h"
#include "hal_pmc_lld.h"
#include "hal_st_lld.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define USART_MAIN_CLOCK (SystemCoreClock / 2)
#define USART_PCLK_CLOCK (ProgrammableClock[4] / 2)
#define USART_NVIC_PRIORITY CORTEX_MIN_KERNEL_PRIORITY-1

/* The CD value scope programmed in MR register. */
#define MIN_CD_VALUE                  0x01
#define MAX_USART_CD_VALUE            US_BRGR_CD_Msk
#define MAX_UART_CD_VALUE            UART_BRGR_CD_Msk

/* The receiver sampling divide of baudrate clock. */
#define HIGH_FRQ_SAMPLE_DIV           16
#define LOW_FRQ_SAMPLE_DIV            8

#if defined(__SAMV71Q21B__)
#define USART0_NVIC_NUMBER USART0_IRQn
#define USART0_HANDLER Vector74
#define USART1_NVIC_NUMBER USART1_IRQn
#define USART1_HANDLER Vector78
#define USART2_NVIC_NUMBER USART2_IRQn
#define USART2_HANDLER Vector7C

#define UART0_NVIC_NUMBER UART0_IRQn
#define UART0_HANDLER Vector5C
#define UART1_NVIC_NUMBER UART1_IRQn
#define UART1_HANDLER Vector60
#define UART2_NVIC_NUMBER UART2_IRQn
#define UART2_HANDLER VectorF0
#define UART3_NVIC_NUMBER UART3_IRQn
#define UART3_HANDLER VectorF4
#define UART4_NVIC_NUMBER UART4_IRQn
#define UART4_HANDLER VectorF8
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SAMV71 configuration options
 * @{
 */
/**
 * @brief   USART0 driver enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_SERIAL_USE_USART0) || defined(__DOXYGEN__)
#define SAMV71_SERIAL_USE_USART0             FALSE
#endif
#if !defined(SAMV71_SERIAL_USE_USART1) || defined(__DOXYGEN__)
#define SAMV71_SERIAL_USE_USART1             FALSE
#endif
#if !defined(SAMV71_SERIAL_USE_USART2) || defined(__DOXYGEN__)
#define SAMV71_SERIAL_USE_USART2             FALSE
#endif
#if !defined(SAMV71_SERIAL_USE_UART0) || defined(__DOXYGEN__)
#define SAMV71_SERIAL_USE_UART0             FALSE
#endif
#if !defined(SAMV71_SERIAL_USE_UART1) || defined(__DOXYGEN__)
#define SAMV71_SERIAL_USE_UART1             FALSE
#endif
#if !defined(SAMV71_SERIAL_USE_UART2) || defined(__DOXYGEN__)
#define SAMV71_SERIAL_USE_UART2             FALSE
#endif
#if !defined(SAMV71_SERIAL_USE_UART3) || defined(__DOXYGEN__)
#define SAMV71_SERIAL_USE_UART3             FALSE
#endif
#if !defined(SAMV71_SERIAL_USE_UART4) || defined(__DOXYGEN__)
#define SAMV71_SERIAL_USE_UART4             FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (USART_NVIC_PRIORITY < ST_NVIC_PRIORITY)
#warning "Setting USART prio higher than systick prio might cause problems"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(USART_NVIC_PRIORITY)
#error "Invalid USART interrupt priority"
#endif

#if SAMV71_SERIAL_USE_USART0 && !defined(ID_USART0)
#error "USART0 is not present on this device"
#endif
#if SAMV71_SERIAL_USE_USART1 && !defined(ID_USART1)
#error "USART1 is not present on this device"
#endif
#if SAMV71_SERIAL_USE_USART2 && !defined(ID_USART2)
#error "USART2 is not present on this device"
#endif
#if SAMV71_SERIAL_USE_UART0 && !defined(ID_UART0)
#error "UART0 is not present on this device"
#endif
#if SAMV71_SERIAL_USE_UART1 && !defined(ID_UART1)
#error "UART1 is not present on this device"
#endif
#if SAMV71_SERIAL_USE_UART2 && !defined(ID_UART2)
#error "UART2 is not present on this device"
#endif
#if SAMV71_SERIAL_USE_UART3 && !defined(ID_UART3)
#error "UART3 is not present on this device"
#endif
#if SAMV71_SERIAL_USE_UART4 && !defined(ID_UART4)
#error "UART4 is not present on this device"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   SAMV71 Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 * @note    This structure content is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct hal_serial_config {
    /**
    * @brief Bit rate.
    */
    uint32_t                  speed;
    /* End of the mandatory fields.*/

    /*
	 * Number of bits, which should be one of the following: US_MR_CHRL_5_BIT,
	 * US_MR_CHRL_6_BIT, US_MR_CHRL_7_BIT, US_MR_CHRL_8_BIT or
	 * US_MR_MODE9.
	 */
	uint32_t char_length;

	/*
	 * Parity type, which should be one of the following: US_MR_PAR_EVEN,
	 * US_MR_PAR_ODD, US_MR_PAR_SPACE, US_MR_PAR_MARK, US_MR_PAR_NO
	 * or US_MR_PAR_MULTIDROP.
	 */
	uint32_t parity_type;

	/*
	 * Number of stop bits between two characters: US_MR_NBSTOP_1_BIT,
	 * US_MR_NBSTOP_1_5_BIT, US_MR_NBSTOP_2_BIT.
	 * \note US_MR_NBSTOP_1_5_BIT is supported in asynchronous modes only.
	 */
	uint32_t stop_bits;

	/*
	 * Run the channel in test mode, which should be one of following:
	 * US_MR_CHMODE_NORMAL, US_MR_CHMODE_AUTOMATIC,
	 * US_MR_CHMODE_LOCAL_LOOPBACK, US_MR_CHMODE_REMOTE_LOOPBACK.
	 */
	uint32_t channel_mode;

	/* Filter of IrDA mode, useless in other modes. */
	uint32_t irda_filter;
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  input_queue_t             iqueue;                                         \
  /* Output queue.*/                                                        \
  output_queue_t            oqueue;                                         \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                         \
  /* Device specific fields */                                              \
  Usart *                   usart;                                          \
  Uart *                    uart;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (SAMV71_SERIAL_USE_USART0 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD0;
#endif
#if (SAMV71_SERIAL_USE_USART1 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if (SAMV71_SERIAL_USE_USART2 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif
#if (SAMV71_SERIAL_USE_UART0 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif
#if (SAMV71_SERIAL_USE_UART1 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD4;
#endif
#if (SAMV71_SERIAL_USE_UART2 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD5;
#endif
#if (SAMV71_SERIAL_USE_UART3 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD6;
#endif
#if (SAMV71_SERIAL_USE_UART4 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD7;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL == TRUE */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
