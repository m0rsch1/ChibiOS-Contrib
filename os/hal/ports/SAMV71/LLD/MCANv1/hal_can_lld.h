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
 * @file    hal_can_lld.h
 * @brief   SAMV71 CAN subsystem low level driver header.
 *
 * @addtogroup CAN
 * @{
 */

#ifndef HAL_CAN_LLD_H
#define HAL_CAN_LLD_H

#if (HAL_USE_CAN == TRUE) || defined(__DOXYGEN__)

#include "samv71.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Use enhanced API
 */
#define CAN_LLD_ENHANCED_API TRUE

/**
 * @brief   Number of transmit mailboxes.
 *
 * Used for input validation; only mailbox numbers below this and
 * CAN_ANY_MAILBOX are going to be passed into the lld driver
 */
//we have (up to) one tx queue and up to 32 tx buffers
//we do osalDbgAssert if the mailbox is not configured.
#define CAN_TX_MAILBOXES            33

#define CAN_TX_MAILBOX_FIFO 1
#define CAN_TX_MAILBOX_BUFFER0 2
#define CAN_TX_MAILBOX_BUFFER63 (2+31)

/**
 * @brief   Number of receive mailboxes.
 *
 * Used for input validation; only mailbox numbers below this and
 * CAN_ANY_MAILBOX are going to be passed into the lld driver
 */
//we have (up to) two rx queues and up to 64 rx buffers, and one more
//rx mailbox for the timestamped transmissions.
//we do osalDbgAssert if the mailbox is not configured.
//a future filter API will make use of the mailboxes/queues above 0
#define CAN_RX_MAILBOXES            67

#define CAN_RX_MAILBOX_FIFO0 1
#define CAN_RX_MAILBOX_FIFO1 2
#define CAN_RX_MAILBOX_BUFFER0 3
#define CAN_RX_MAILBOX_BUFFER63 (3+63)
#define CAN_RX_MAILBOX_TXEVENT 67

#define MCAN_NVIC_PRIORITY CORTEX_MIN_KERNEL_PRIORITY-1

#if defined(__SAMV71Q21B__)
#define MCAN0_INT0_NVIC_NUMBER MCAN0_INT0_IRQn
#define MCAN0_INT0_HANDLER VectorCC
#define MCAN0_INT1_NVIC_NUMBER MCAN0_INT1_IRQn
#define MCAN0_INT1_HANDLER VectorD0
#define MCAN1_INT0_NVIC_NUMBER MCAN1_INT0_IRQn
#define MCAN1_INT0_HANDLER VectorD4
#define MCAN1_INT1_NVIC_NUMBER MCAN1_INT1_IRQn
#define MCAN1_INT1_HANDLER VectorD8
#endif

#define MCAN_CONFIG_CCCR_MASK (MCAN_CCCR_NISO | MCAN_CCCR_TXP | MCAN_CCCR_EFBI | MCAN_CCCR_BRSE | MCAN_CCCR_FDOE | MCAN_CCCR_DAR | MCAN_CCCR_MON | MCAN_CCCR_TEST)

#define CAN_DLC_8 0x8
#define CAN_DLC_12 0x9
#define CAN_DLC_16 0xa
#define CAN_DLC_20 0xb
#define CAN_DLC_24 0xc
#define CAN_DLC_32 0xd
#define CAN_DLC_48 0xe
#define CAN_DLC_64 0xf

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SAMV71 configuration options
 * @{
 */
/**
 * @brief   CAN0 driver enable switch.
 * @details If set to @p TRUE the support for CAN1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_CAN_USE_CAN0) || defined(__DOXYGEN__)
#define SAMV71_CAN_USE_CAN0                  FALSE
#endif
/**
 * @brief   CAN1 driver enable switch.
 * @details If set to @p TRUE the support for CAN1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_CAN_USE_CAN1) || defined(__DOXYGEN__)
#define SAMV71_CAN_USE_CAN1                  FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !OSAL_IRQ_IS_VALID_PRIORITY(MCAN_NVIC_PRIORITY)
#error "Invalid MCAN interrupt priority"
#endif

#define CAN_PERIPH_CLOCK (SystemCoreClock / 2)
#define CAN_CORE_CLOCK (ProgrammableClock[5])

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing a CAN driver.
 */
typedef struct hal_can_driver CANDriver;

/**
 * @brief   Type of a transmission mailbox index.
 */
typedef uint32_t canmbx_t;

#if defined(CAN_ENFORCE_USE_CALLBACKS) || defined(__DOXYGEN__)
/**
 * @brief   Type of a CAN notification callback.
 *
 * @param[in] canp      pointer to the @p CANDriver object triggering the
 *                      callback
 * @param[in] flags     flags associated to the mailbox callback
 */
typedef void (*can_callback_t)(CANDriver *canp, uint32_t flags);
#endif

/**
 * @brief   CAN transmission frame.
 * @note    Accessing the frame data as word16 or word32 is not portable because
 *          machine data endianness, it can be still useful for a quick filling.
 */
typedef struct {
  /*lint -save -e46 [6.1] Standard types are fine too.*/
  uint8_t                   MM;             /**< @brief Message marker. (for identifying TxEvents) */
  uint8_t                   DLC:4;          /**< @brief Data length.        */
  uint8_t                   RTR:1;          /**< @brief Frame type.         */
  uint8_t                   IDE:1;          /**< @brief Identifier type.    */
  uint8_t                   FDF:1;          /**< @brief FD format.          */
  uint8_t                   BRS:1;          /**< @brief Bit rate select.    */
  uint8_t                   EFC_:1;         /**< @brief Event FIFO control. */
  union {
    uint32_t                SID:11;         /**< @brief Standard identifier.*/
    uint32_t                EID:29;         /**< @brief Extended identifier.*/
    uint32_t                _align1;
  };
  /*lint -restore*/
  union {
    uint8_t                 data8[64];      /**< @brief Frame data.         */
    uint16_t                data16[32];     /**< @brief Frame data.         */
    uint32_t                data32[16];     /**< @brief Frame data.         */
  };
} CANTxFrame;

/**
 * @brief   CAN received frame.
 * @note    Accessing the frame data as word16 or word32 is not portable because
 *          machine data endianness, it can be still useful for a quick filling.
 */
typedef struct {
  /*lint -save -e46 [6.1] Standard types are fine too.*/
  uint8_t                   FMI;            /**< @brief Filter id or Message marker */
  uint16_t                  TIME;           /**< @brief Time stamp.         */
  uint8_t                   DLC:4;          /**< @brief Data length.        */
  uint8_t                   RTR:1;          /**< @brief Frame type.         */
  uint8_t                   IDE:1;          /**< @brief Identifier type.    */
  uint8_t                   FDF:1;          /**< @brief FD format.          */
  uint8_t                   BRS:1;          /**< @brief Bit rate select.    */
  union {
    uint32_t                SID:11;         /**< @brief Standard identifier.*/
    uint32_t                EID:29;         /**< @brief Extended identifier.*/
    uint32_t                _align1;
  };
  /*lint -restore*/
  union {
    uint8_t                 data8[64];      /**< @brief Frame data.         */
    uint16_t                data16[32];     /**< @brief Frame data.         */
    uint32_t                data32[16];     /**< @brief Frame data.         */
  };
  uint32_t                  timeroverflowcount; /**< @brief Number of times the TIME source overflowed */
} CANRxFrame;

/**
 * @brief   Type of a CAN configuration structure.
 */
typedef struct hal_can_config {
  /* End of the mandatory fields.*/
  /** CCCR register contents
   *
   * Filtered for MCAN_CONFIG_CCCR_MASK
   */
  uint32_t cccr;
  /** TDCR register contents, plus DBTP_TDC_ENABLED
   *
   * transmitter delay componsation settings and enable bit
   */
  uint32_t tdcr;
  /** TEST register contents
   *
   * Loopback and forced TX state
   */
  uint8_t test;
  /** TSCC register contents
   *
   * According to Datasheet 49.6.9, for FD, an "external timer" has to be used.
   * The internal timestamp and timeout counters are driven by the CAN Core
   * sample point signal that changes frequency between arbitration and data
   * phases(Datasheet 49.5.3).
   *
   * Datasheet 49.4.5:
   * Timestamping uses the value of CV in the TC Counter Value 0 register
   * (TC_CV0) at address 0x4000C010. TC0 can use the programmable clocks PCK6
   * or PCK7 as input. Refer to the section “Timer Counter (TC)” for more
   * details. The selection between PCK6 and PCK7 is done in the Matrix
   * Peripheral Clock Configuration Register (CCFG_PCCR), using the bit TC0CC.
   * Refer to this register in the section “Bus Matrix (MATRIX)” for more
   * details. These clocks can be programmed in the the registers PMC
   * Programmable Clock Registers PMC_PCK6 and PMC_PCK7, respectively. Refer
   * to these registers in the section “Power Management Controller (PMC)” for
   * more details.
   *
   * Using (MCAN_TSCC_TSS_TCP_INC | MCAN_TSCC_TCP(0)) results in a timestamp
   * that increments for every bit time.
   *
   * External clock sources should call canTimerOverflowI().
   */
  uint32_t tscc;
  /** GFC register contents
   *
   * Global filter settings; default message handling and similar
   */
  uint32_t gfc;
  /** XIDAM register contents
   *
   * Optional mask for extended can id range filters.
   */
  uint32_t xidam;

  uint32_t nominal_bitrate;/**< Bitrate in Baud during arbitration phase, standard transfers and fd without bit rate switching */
  uint32_t fd_data_bitrate;/**< Bitrate in Baud during data phase using fd with bit rate switching */

  uint8_t std_flt_count; /**< Number of standard id filters; 0-128 */
  uint8_t ext_flt_count; /**< Number of extended id filters; 0-64 */
  uint8_t rxfifo0_count; /**< Number of elements in rx fifo 0; 0-64 */
  uint8_t rxfifo0_data_size; /**< Data size for rx fifo 0 elements; 8, 12, 16, 20, 24, 32, 48, 64 */
  uint8_t rxfifo1_count; /**< Number of elements in rx fifo 1; 0-64 */
  uint8_t rxfifo1_data_size; /**< Data size for rx fifo 1 elements; 8, 12, 16, 20, 24, 32, 48, 64 */
  uint8_t rxbuffer_count; /**< Number of rx buffers; 0-64 */
  uint8_t rxbuffer_data_size; /**< Data size for rx buffer elements; 8, 12, 16, 20, 24, 32, 48, 64 */
  uint8_t txfifo_count; /**< Number of elements in the tx fifo; sum of this and txbuffer_count 0-32 */
  uint8_t txbuffer_count; /**< Number of elements in the tx fifo; sum of this and txfifo_count 0-32 */
  uint8_t txelem_data_size; /**< Data size for tx fifo and buffer elements; 8, 12, 16, 20, 24, 32, 48, 64 */
  uint8_t txevent_count; /**< Number of elements in the tx event fifo; 0-32 */
  /** Memory for DMA.
   *
   * Memory must be of sufficient size to accomodate the configured fifos and
   * buffers. Use one of the MCAN_MEMORY_ALLOCATION_SIZE_* macros to
   * calculate the size needed. The memory is not allowed to cross a 64kByte
   * boundary, except for
   * 4*(rxfifo0_count+rxfifo1_count+rxbuffer_count+txevent_count) bytes that
   * can be shuffled to either end of the memory arena.
   *
   * If in doubt, allocate twice the needed amount, or check
   * where the allocation ends up after linking. The memory position is also
   * checked at canStart and will result in an HAL_RET_CONFIG_ERROR.
   */
  void *memory;
  /** Size of memory pointed to by memory */
  size_t memory_size;
} CANConfig;


/**
 * @brief   Structures for the memory structures used by the device
 */

//align_size must be a power of 2
#define MCAN_MEMORY_PAD_TO(size, align_size) \
  (((size)+((align_size)-1)) & ~((align_size)-1))

#define MCAN_MEMORY_ALLOCATION_SIZE_ALIGNED_32_(stdflt, extflt, rxf0_cnt, rxf0_dsz, rxf1_cnt, rxf1_dsz, rxb_cnt, rxb_dsz, txf_cnt, txb_cnt, txfb_dsz, txef_cnt) \
  (MCAN_MEMORY_PAD_TO( /* these are cpu write/device read; pad to multiples of 32 bytes */ \
     4*(stdflt)+ \
     8*(extflt)+ \
     MCAN_MEMORY_PAD_TO(8+(txfb_dsz),4)*(txf_cnt + txb_cnt), 32) + \
   MCAN_MEMORY_PAD_TO( /* these are cpu read/device write; at the end of the block. also pad to multiples of 32 bytes so we do not mess with the next allocation */ \
     MCAN_MEMORY_PAD_TO(8+(rxf0_dsz),4)*(rxf0_cnt) + \
     MCAN_MEMORY_PAD_TO(8+(rxf1_dsz),4)*(rxf1_cnt) + \
     MCAN_MEMORY_PAD_TO(8+(rxb_dsz),4)*(rxb_cnt) + \
     8*(txef_cnt), 32)) + \
     (4 * ((rxf0_cnt)+(rxf1_cnt)+(rxb_cnt)+(txef_cnt)))

#define MCAN_MEMORY_ALLOCATION_SIZE_UNALIGNED_(stdflt, extflt, rxf0_cnt, rxf0_dsz, rxf1_cnt, rxf1_dsz, rxb_cnt, rxb_dsz, txf_cnt, txb_cnt, txfb_dsz, txef_cnt) \
  (31 + /*up to 31 bytes to align first block on cache line */ \
   MCAN_MEMORY_ALLOCATION_SIZE_ALIGNED_32_(stdflt, extflt, rxf0_cnt, rxf0_dsz, rxf1_cnt, rxf1_dsz, rxb_cnt, rxb_dsz, txf_cnt, txb_cnt, txfb_dsz, txef_cnt))

#define MCAN_MEMORY_SIZE_DEFINITIONS_DOT_SYNTAX_(stdflt, extflt, rxf0_cnt, rxf0_dsz, rxf1_cnt, rxf1_dsz, rxb_cnt, rxb_dsz, txf_cnt, txb_cnt, txfb_dsz, txef_cnt) \
  .std_flt_count = stdflt,       \
  .ext_flt_count = extflt,       \
  .rxfifo0_count = rxf0_cnt,     \
  .rxfifo0_data_size = rxf0_dsz, \
  .rxfifo1_count = rxf1_cnt,     \
  .rxfifo1_data_size = rxf1_dsz, \
  .rxbuffer_count = rxb_cnt,     \
  .rxbuffer_data_size = rxb_dsz, \
  .txfifo_count = txf_cnt,       \
  .txbuffer_count = txb_cnt,     \
  .txelem_data_size = txfb_dsz,  \
  .txevent_count = txef_cnt

#define MCAN_MEMORY_SIZE_DEFINITIONS(stdflt, extflt, rxf0_cnt, rxf0_dsz, rxf1_cnt, rxf1_dsz, rxb_cnt, rxb_dsz, txf_cnt, txb_cnt, txfb_dsz, txef_cnt) \
  stdflt, extflt, rxf0_cnt, rxf0_dsz, rxf1_cnt, rxf1_dsz, rxb_cnt, rxb_dsz, txf_cnt, txb_cnt, txfb_dsz, txef_cnt

#define MCAN_MEMORY_ALLOCATION_SIZE_UNALIGNED(defs) MCAN_MEMORY_ALLOCATION_SIZE_UNALIGNED_(defs)
#define MCAN_MEMORY_ALLOCATION_SIZE_ALIGNED_32(defs) MCAN_MEMORY_ALLOCATION_SIZE_ALIGNED_32_(defs)
#define MCAN_MEMORY_SIZE_DEFINITIONS_DOT_SYNTAX(defs) MCAN_MEMORY_SIZE_DEFINITIONS_DOT_SYNTAX_(defs)
#define MCAN_MEMORY_SIZE_DEFINITIONS_ARRAY_SYNTAX(defs) defs

/**
 * @brief   Structure representing a CAN driver.
 */
struct hal_can_driver {
  /**
   * @brief   Driver state.
   */
  canstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const CANConfig           *config;
  /**
   * @brief   Transmission threads queue.
   */
  threads_queue_t           txqueue;
  /**
   * @brief   Receive threads queue.
   */
  threads_queue_t           rxqueue;
#if (CAN_ENFORCE_USE_CALLBACKS == FALSE) || defined (__DOXYGEN__)
  /**
   * @brief   One or more frames become available.
   * @note    After broadcasting this event it will not be broadcasted again
   *          until the received frames queue has been completely emptied. It
   *          is <b>not</b> broadcasted for each received frame. It is
   *          responsibility of the application to empty the queue by
   *          repeatedly invoking @p chReceive() when listening to this event.
   *          This behavior minimizes the interrupt served by the system
   *          because CAN traffic.
   * @note    The flags associated to the listeners will indicate which
   *          receive mailboxes become non-empty.
   */
  event_source_t            rxfull_event;
  /**
   * @brief   One or more transmission mailbox become available.
   * @note    The flags associated to the listeners will indicate which
   *          transmit mailboxes become empty.
   */
  event_source_t            txempty_event;
  /**
   * @brief   A CAN bus error happened.
   * @note    The flags associated to the listeners will indicate the
   *          error(s) that have occurred.
   */
  event_source_t            error_event;
#if (CAN_USE_SLEEP_MODE == TRUE) || defined (__DOXYGEN__)
  /**
   * @brief   Entering sleep state event.
   */
  event_source_t            sleep_event;
  /**
   * @brief   Exiting sleep state event.
   */
  event_source_t            wakeup_event;
#endif
#else /* CAN_ENFORCE_USE_CALLBACKS == TRUE */
  /**
   * @brief   One or more frames become available.
   * @note    After calling this function it will not be called again
   *          until the received frames queue has been completely emptied. It
   *          is <b>not</b> called for each received frame. It is
   *          responsibility of the application to empty the queue by
   *          repeatedly invoking @p chTryReceiveI().
   *          This behavior minimizes the interrupt served by the system
   *          because CAN traffic.
   */
  can_callback_t            rxfull_cb;
  /**
   * @brief   One or more transmission mailbox become available.
   * @note    The flags associated to the callback will indicate which
   *          transmit mailboxes become empty.
   */
  can_callback_t            txempty_cb;
  /**
   * @brief   A CAN bus error happened.
   */
  can_callback_t            error_cb;
#if (CAN_USE_SLEEP_MODE == TRUE) || defined (__DOXYGEN__)
  /**
   * @brief   Exiting sleep state.
   */
  can_callback_t            wakeup_cb;
#endif
#endif
  /* End of the mandatory fields.*/
  Mcan *device;
  void *std_flt;
  void *ext_flt;
  void *rxfifo0;
  void *rxfifo1;
  void *rxbuffers;
  void *txeventfifo;
  void *txbuffers;

  uint32_t *rxfifo0_timeroverflowcount;
  uint32_t *rxfifo1_timeroverflowcount;
  uint32_t *rxbuffers_timeroverflowcount;
  uint32_t *txeventfifo_timeroverflowcount;

  //bits set to 1 where we reported an empty buffer so we don't report twice. set to 0 after can_lld_receive.
  uint64_t rxbuffer_status;
  //bits set to 1 where a message is being transmitted; bit 31 is used for
  //the fifo, if configured.
  //(fifo and buffers share the 32 available memory locations)
  uint32_t txbuffer_status;

  uint32_t timerOverflowCounter;
  uint8_t rxfifo0_lastPutPos;
  uint8_t rxfifo1_lastPutPos;
  uint8_t txeventfifo_lastPutPos;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (SAMV71_CAN_USE_CAN0 == TRUE) && !defined(__DOXYGEN__)
extern CANDriver CAND0;
#endif
#if (SAMV71_CAN_USE_CAN1 == TRUE) && !defined(__DOXYGEN__)
extern CANDriver CAND1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void can_lld_init(void);
  msg_t can_lld_start(CANDriver *canp);
  void can_lld_stop(CANDriver *canp);
  bool can_lld_is_tx_empty(CANDriver *canp, canmbx_t mailbox);
  void can_lld_transmit(CANDriver *canp,
                        canmbx_t mailbox,
                        const CANTxFrame *ctfp);
  bool can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox);
  void can_lld_receive(CANDriver *canp,
                       canmbx_t mailbox,
                       CANRxFrame *crfp);
  void can_lld_abort(CANDriver *canp,
                     canmbx_t mailbox);
#if CAN_USE_SLEEP_MODE == TRUE
  void can_lld_sleep(CANDriver *canp);
  void can_lld_wakeup(CANDriver *canp);
#endif

  /* if the timer counter is configured to be external, this must be
   * called whenever the 16 bit counter overflows.
   */
  void canTimerOverflowI(CANDriver *canp);

/* extra user facing functions */
void canFilterStdDisableX ( CANDriver *canp,  uint8_t filter );
void canFilterExtDisableX ( CANDriver *canp, uint8_t filter );
void canFilterStdSetFifoRangeX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint16_t SID1, uint16_t SID2, bool HPM );
void canFilterExtSetFifoRangeX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint32_t EID1, uint32_t EID2, bool HPM, bool applyGlobalMasking );
void canFilterStdSetFifoIDPairX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint16_t SID1, uint16_t SID2, bool HPM );
void canFilterExtSetFifoIDPairX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint32_t EID1, uint32_t EID2, bool HPM );
void canFilterStdSetFifoIDMaskX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint16_t SID, uint16_t SMASK, bool HPM );
void canFilterExtSetFifoIDMaskX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint32_t EID, uint32_t EMASK, bool HPM );
void canFilterStdSetRejectRangeX ( CANDriver *canp, uint8_t filter, uint16_t SID1, uint16_t SID2 );
void canFilterExtSetRejectRangeX ( CANDriver *canp, uint8_t filter, uint32_t EID1, uint32_t EID2, bool applyGlobalMasking );
void canFilterStdSetRejectIDPairX ( CANDriver *canp, uint8_t filter, uint16_t SID1, uint16_t SID2 );
void canFilterExtSetRejectIDPairX ( CANDriver *canp, uint8_t filter, uint32_t EID1, uint32_t EID2 );
void canFilterStdSetRejectIDMaskX ( CANDriver *canp, uint8_t filter, uint16_t SID, uint16_t SMASK );
void canFilterExtSetRejectIDMaskX ( CANDriver *canp, uint8_t filter, uint32_t EID, uint32_t EMASK );
void canFilterStdSetMarkHPMRangeX ( CANDriver *canp, uint8_t filter, uint16_t SID1, uint16_t SID2 );
void canFilterExtSetMarkHPMRangeX ( CANDriver *canp, uint8_t filter, uint32_t EID1, uint32_t EID2, bool applyGlobalMasking );
void canFilterStdSetMarkHPMIDPairX ( CANDriver *canp, uint8_t filter, uint16_t SID1, uint16_t SID2 );
void canFilterExtSetMarkHPMIDPairX ( CANDriver *canp, uint8_t filter, uint32_t EID1, uint32_t EID2 );
void canFilterStdSetMarkHPMIDMaskX ( CANDriver *canp, uint8_t filter, uint16_t SID, uint16_t SMASK );
void canFilterExtSetMarkHPMIDMaskX ( CANDriver *canp, uint8_t filter, uint32_t EID, uint32_t EMASK );
void canFilterStdSetBufferFilterX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint16_t SID );
void canFilterExtSetBufferFilterX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint32_t EID );

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CAN == TRUE */

#endif /* HAL_CAN_LLD_H */

/** @} */
