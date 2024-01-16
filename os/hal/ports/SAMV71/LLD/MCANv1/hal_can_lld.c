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
 * @file    hal_can_lld.c
 * @brief   PLATFORM CAN subsystem low level driver source.
 *
 * @addtogroup CAN
 * @{
 */

#include "hal.h"
#include "hal_matrix_lld.h"
#include "string.h"

#if (HAL_USE_CAN == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CAN0 driver identifier.
 */
#if (SAMV71_CAN_USE_CAN0 == TRUE) || defined(__DOXYGEN__)
CANDriver CAND0;
#endif

/**
 * @brief   CAN1 driver identifier.
 */
#if (SAMV71_CAN_USE_CAN1 == TRUE) || defined(__DOXYGEN__)
CANDriver CAND1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

typedef struct {
        uint32_t id:29;
        uint8_t rtr:1;
        uint8_t xtd:1;
        uint8_t esi:1;
        uint16_t rxts:16;
        uint8_t dlc:4;
        uint8_t brs:1;
        uint8_t fdf:1;
        uint8_t unused_54:2;
        uint8_t fidx:7;
        uint8_t anmf:1;
} MCAN_RX_Element_Header;

typedef struct {
        uint32_t id:29;
        uint8_t rtr:1;
        uint8_t xtd:1;
        uint8_t esi:1;
        uint16_t unused_32:16;
        uint8_t dlc:4;
        uint8_t brs:1;
        uint8_t fdf:1;
        uint8_t unused_54:1;
        uint8_t efc:1;
        uint8_t mm:8;
} MCAN_TX_Element_Header;

typedef struct {
        uint32_t id:29;
        uint8_t rtr:1;
        uint8_t xtd:1;
        uint8_t esi:1;
        uint16_t txts:16;
        uint8_t dlc:4;
        uint8_t brs:1;
        uint8_t fdf:1;
        uint8_t et:2;
        uint8_t mm:8;
} MCAN_TX_Event_Element_Header;

typedef struct {
    uint16_t sfid2:11;
    uint8_t unused_12:5;
    uint16_t sfid1:11;
    uint8_t sfec:3;
    uint8_t sft:2;
} MCAN_Std_Filter_Element;

typedef struct {
    uint32_t efid1:29;
    uint8_t efec:3;
    uint32_t efid2:29;
    uint8_t unused_61:1;
    uint8_t eft:2;
} MCAN_Ext_Filter_Element;

static uint8_t const DLC_to_size[16] = {
    0, 1, 2, 3, 4, 5, 6, 7,
    8, 12, 16, 20, 24, 32, 48, 64
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// converts a byte size to a fifo/buffer data size code
static int convert_data_size(uint8_t size) {
    switch(size) {
    case 8:
        return 0;
    case 12:
        return 1;
    case 16:
        return 2;
    case 20:
        return 3;
    case 24:
        return 4;
    case 32:
        return 5;
    case 48:
        return 6;
    case 64:
        return 7;
    default:
        return -1;
    }
}

static void fill_tx_fifo_elem_for_packet(char *elem_base,
                                         const CANTxFrame *ctfp,
                                         size_t max_data) {
    MCAN_TX_Element_Header *elem_head =
        ( MCAN_TX_Element_Header* ) elem_base;
    char *elem_data = elem_base+sizeof ( MCAN_TX_Element_Header );
    elem_head->esi = 0; // Error state indicator
    elem_head->xtd = ctfp->IDE;
    elem_head->rtr = ctfp->RTR;
    elem_head->id = ctfp->IDE?ctfp->EID:(ctfp->SID << 18);
    elem_head->mm = ctfp->MM;
    elem_head->efc = ctfp->EFC_;
    elem_head->fdf = ctfp->FDF;
    elem_head->brs = ctfp->BRS;
    elem_head->dlc = ctfp->DLC;
    size_t len = DLC_to_size[ctfp->DLC];
    if(len > max_data)
        len = max_data;
    memcpy ( elem_data, ctfp->data8, len );
    if((SCB->CCR & SCB_CCR_DC_Msk) != 0) {
        //D-Cache enabled. need to take care of cleaning/invalidating
        SCB_CleanDCache_by_Addr ( ( uint32_t* ) elem_base, 8 + max_data );
    }
}

static void parse_rx_fifo_elem_to_packet(CANRxFrame *crfp,
                                         char const *elem_base,
                                         size_t max_data) {
    if ( ( SCB->CCR & SCB_CCR_DC_Msk ) != 0 ) {
        //D-Cache enabled. need to take care of cleaning/invalidating
        SCB_InvalidateDCache_by_Addr ( ( uint32_t* ) elem_base, 8 + max_data );
    }
    MCAN_RX_Element_Header const *elem_head =
        ( MCAN_RX_Element_Header* ) elem_base;
    char const *elem_data = elem_base+sizeof ( MCAN_RX_Element_Header );

    //elem_head->esi;  // error state indicator

    crfp->TIME = elem_head->rxts;
    crfp->DLC = elem_head->dlc;
    crfp->RTR = elem_head->rtr;
    crfp->FDF = elem_head->fdf;
    crfp->BRS = elem_head->brs;

    if(elem_head->anmf != 0) {
        // accepted non-matching frame
        crfp->FMI = ~0;
    } else {
        crfp->FMI = elem_head->fidx;
    }

    if ( elem_head->xtd ) {
        crfp->IDE = 1;
        crfp->EID = elem_head->id;
    } else {
        crfp->IDE = 0;
        crfp->SID = elem_head->id >> 18;
    }

    size_t len = DLC_to_size[crfp->DLC];
    if(len > max_data)
        len = max_data;
    memcpy( crfp->data8, elem_data, len);
}

static void parse_txevent_fifo_elem_to_packet(CANRxFrame *crfp,
                                         char const *elem_base) {
    if ( ( SCB->CCR & SCB_CCR_DC_Msk ) != 0 ) {
        //D-Cache enabled. need to take care of cleaning/invalidating
        SCB_InvalidateDCache_by_Addr ( ( uint32_t* ) elem_base, 8 );
    }
    MCAN_TX_Event_Element_Header const *elem_head =
        ( MCAN_TX_Event_Element_Header* ) elem_base;

    //elem_head->esi;  // error state indicator

    crfp->TIME = elem_head->txts;
    crfp->DLC = elem_head->dlc;
    crfp->RTR = elem_head->rtr;
    crfp->FDF = elem_head->fdf;
    crfp->BRS = elem_head->brs;

    crfp->FMI = elem_head->mm;

    if ( elem_head->xtd ) {
        crfp->IDE = 1;
        crfp->EID = elem_head->id;
    } else {
        crfp->IDE = 0;
        crfp->SID = elem_head->id >> 18;
    }
}

uint32_t can_irq0_bits_seen = 0;
uint32_t can_irq1_bits_seen = 0;
uint32_t can_irq0_bits_cleared = 0;
uint32_t can_irq1_bits_cleared = 0;

static void can_lld_serve_interrupt0 ( CANDriver *canp )
{
    //the interrupt gets recalled if still active, no need to loop here.
    uint32_t ir = canp->device->MCAN_IR;
    uint32_t ie = canp->device->MCAN_IE;
    uint32_t ils = canp->device->MCAN_ILS;

    //clear interrupt status bit handled by us
    canp->device->MCAN_IR = ir & ie & ~ils;
    can_irq0_bits_seen |= ir;
    can_irq0_bits_cleared |= ir & ie & ~ils;

    //select only enabled interrupts that are handled by us
    ir &= ie & ~ils;

    /* we are interested in
     *
     * the rx interrupts:
     * DRX: Message stored to dedicated read buffer _can_rx_full_isr(mailbox)
     * RF1N: New message written to receive fifo 1 _can_rx_full_isr(2)
     * RF0N: New message written to receive fifo 0 _can_rx_full_isr(1)
     */

    if((ir & MCAN_IR_TSW) != 0) {
        osalSysLockFromISR();
        canTimerOverflowI(canp);
        osalSysUnlockFromISR();
    }

    if((ir & MCAN_IR_RF1N) != 0) {
        //rx fifo put index points to the next element to be written
        while ( canp->rxfifo1_lastPutPos !=
                ( canp->device->MCAN_RXF1S & MCAN_RXF1S_F1PI_Msk ) >> MCAN_RXF1S_F1PI_Pos ) {
            char *fifo_elem_base = canp->rxfifo1 +
                                   ( sizeof ( MCAN_RX_Element_Header ) +
                                     canp->config->rxfifo1_data_size ) * canp->rxfifo1_lastPutPos;
            MCAN_RX_Element_Header const *elem_head =
                ( MCAN_RX_Element_Header const * ) fifo_elem_base;
            osalSysLockFromISR();
            if ( elem_head->rxts < canp->device->MCAN_TSCV ) {
                canp->rxfifo1_timeroverflowcount[canp->rxfifo1_lastPutPos] = canp->timerOverflowCounter;
            } else {
                canp->rxfifo1_timeroverflowcount[canp->rxfifo1_lastPutPos] = canp->timerOverflowCounter-1;
            }
            osalSysUnlockFromISR();
            canp->rxfifo1_lastPutPos++;
            if ( canp->rxfifo1_lastPutPos >= canp->config->rxfifo1_count )
                canp->rxfifo1_lastPutPos = 0;
        }

        _can_rx_full_isr(canp, CAN_RX_MAILBOX_FIFO1);
    }
    if((ir & MCAN_IR_RF0N) != 0) {
        while ( canp->rxfifo0_lastPutPos !=
                ( canp->device->MCAN_RXF0S & MCAN_RXF0S_F0PI_Msk ) >> MCAN_RXF0S_F0PI_Pos ) {
            char *fifo_elem_base = canp->rxfifo0 +
                                   ( sizeof ( MCAN_RX_Element_Header ) +
                                     canp->config->rxfifo0_data_size ) * canp->rxfifo0_lastPutPos;
            MCAN_RX_Element_Header const *elem_head =
                ( MCAN_RX_Element_Header const * ) fifo_elem_base;
            osalSysLockFromISR();
            if ( elem_head->rxts < canp->device->MCAN_TSCV ) {
                canp->rxfifo0_timeroverflowcount[canp->rxfifo0_lastPutPos] = canp->timerOverflowCounter;
            } else {
                canp->rxfifo0_timeroverflowcount[canp->rxfifo0_lastPutPos] = canp->timerOverflowCounter-1;
            }
            osalSysUnlockFromISR();
            canp->rxfifo0_lastPutPos++;
            if ( canp->rxfifo0_lastPutPos >= canp->config->rxfifo0_count )
                canp->rxfifo0_lastPutPos = 0;
        }
        _can_rx_full_isr(canp, CAN_RX_MAILBOX_FIFO0);
    }
    if((ir & MCAN_IR_TEFN) != 0) {
        while ( canp->txeventfifo_lastPutPos !=
                ( canp->device->MCAN_TXEFS & MCAN_TXEFS_EFPI_Msk ) >> MCAN_TXEFS_EFPI_Pos ) {
            char *fifo_elem_base = canp->txeventfifo +
                                   ( sizeof ( MCAN_TX_Event_Element_Header ) ) * canp->txeventfifo_lastPutPos;
            MCAN_TX_Event_Element_Header const *elem_head =
                ( MCAN_TX_Event_Element_Header const * ) fifo_elem_base;
            osalSysLockFromISR();
            if ( elem_head->txts < canp->device->MCAN_TSCV ) {
                canp->txeventfifo_timeroverflowcount[canp->txeventfifo_lastPutPos] = canp->timerOverflowCounter;
            } else {
                canp->txeventfifo_timeroverflowcount[canp->txeventfifo_lastPutPos] = canp->timerOverflowCounter-1;
            }
            osalSysUnlockFromISR();
            canp->txeventfifo_lastPutPos++;
            if ( canp->txeventfifo_lastPutPos >= canp->config->txevent_count )
                canp->txeventfifo_lastPutPos = 0;
        }
        _can_rx_full_isr(canp, CAN_RX_MAILBOX_TXEVENT);
    }
    if((ir & MCAN_IR_DRX) != 0) {

        osalSysLockFromISR();
        uint64_t actual_status = 0;
        actual_status |= canp->device->MCAN_NDAT1;
        actual_status |= ( ( uint64_t ) canp->device->MCAN_NDAT2 ) << 32;
        canp->device->MCAN_NDAT1 = actual_status;
        canp->device->MCAN_NDAT2 = actual_status >> 32;

        //calculate buffers that have been written without having been read
        uint64_t overflow = actual_status & canp->rxbuffer_status;
        //calculate buffers that have been written after having been read
        uint64_t newly_set = actual_status & ~canp->rxbuffer_status;
        canp->rxbuffer_status |= actual_status;
        osalSysUnlockFromISR();

        //check which mailbox has updated
        for ( int i = 0; i < canp->config->rxbuffer_count; i++ ) {
            if ( ( actual_status & ( 1ULL << i ) ) != 0 ) {
                char *buffer_elem_base = canp->rxbuffers +
                                         ( sizeof ( MCAN_RX_Element_Header ) +
                                           canp->config->rxbuffer_data_size ) * i;
                MCAN_RX_Element_Header const *elem_head =
                    ( MCAN_RX_Element_Header const * ) buffer_elem_base;
                osalSysLockFromISR();
                if ( elem_head->rxts < canp->device->MCAN_TSCV ) {
                    canp->rxbuffers_timeroverflowcount[i] = canp->timerOverflowCounter;
                } else {
                    canp->rxbuffers_timeroverflowcount[i] = canp->timerOverflowCounter-1;
                }
                osalSysUnlockFromISR();
            }
        }

        //if any overflow happened, report that.
        if(overflow != 0ULL)
            _can_error_isr ( canp, CAN_OVERFLOW_ERROR );

        if ( newly_set != 0ULL ) {
            //check which mailbox has changed
            for ( int i = 0; i < canp->config->rxbuffer_count; i++ ) {
                if ( ( newly_set & ( 1ULL << i ) ) != 0 ) {
                    _can_rx_full_isr ( canp, i+CAN_RX_MAILBOX_BUFFER0 );
                }
            }
        }
    }
}

static void can_lld_serve_interrupt1 ( CANDriver *canp )
{
    uint32_t ir = canp->device->MCAN_IR;
    uint32_t ie = canp->device->MCAN_IE;
    uint32_t ils = canp->device->MCAN_ILS;

    //clear interrupt status bit handled by us
    canp->device->MCAN_IR = ir & ie & ils;
    can_irq1_bits_seen |= ir;
    can_irq1_bits_cleared |= ir & ie & ~ils;

    //select only enabled interrupts that are handled by us
    ir &= ie & ils;

    //handle errors and tx side, as configured

    /* we are interested in any of the error interrupt sources, specifically
     *
     * PED: Protocol Error in Data Phase  _can_error_isr(CAN_FRAMING_ERROR)
     * PEA: Protocol Error in Arbitration Phase  _can_error_isr(CAN_FRAMING_ERROR)
     * BO: Bus off status  _can_error_isr(CAN_BUS_OFF_ERROR)
     * EW: Warning status  _can_error_isr(CAN_LIMIT_WARNING)
     * RF1L: Receive Fifo 1 message lost  _can_error_isr(CAN_OVERFLOW_ERROR)
     * RF0L: Receive Fifo 0 message lost   _can_error_isr(CAN_OVERFLOW_ERROR)
     *
     */
    eventflags_t error_flags = 0;
    if((ir & (MCAN_IR_PED | MCAN_IR_PEA)) != 0) {
        error_flags |= CAN_FRAMING_ERROR;
    }
    if((ir & MCAN_IR_BO) != 0) {
        error_flags |= CAN_BUS_OFF_ERROR;
    }
    if((ir & MCAN_IR_EW) != 0) {
        error_flags |= CAN_LIMIT_WARNING;
    }
    if((ir & (MCAN_IR_RF1L | MCAN_IR_RF0L)) != 0) {
        error_flags |= CAN_OVERFLOW_ERROR;
    }
    if(error_flags) {
        _can_error_isr(canp, error_flags);
    }

    /* we are interested in the tx complete interrupt
     * TC: Transmission completed _can_tx_empty_isr(mailbox)
     *
     */
    if((ir & MCAN_IR_TC) != 0) {
        osalSysLockFromISR();
        uint32_t actual_status = 0;//1 bit set where the buffer is actually full

        //queue not full?
        if((canp->device->MCAN_TXFQS & MCAN_TXFQS_TFQF) == 0 && canp->config->txfifo_count != 0)
            actual_status |= 0x80000000U;
        actual_status |= canp->device->MCAN_TXBRP & ((1U << canp->config->txbuffer_count)-1);

        uint32_t newly_cleared = ~actual_status & canp->txbuffer_status;
        canp->txbuffer_status &= ~newly_cleared;
        osalSysUnlockFromISR();

        if((newly_cleared & 0x80000000U) != 0) {
            _can_tx_empty_isr ( canp, CAN_TX_MAILBOX_FIFO );
        }

        if((newly_cleared & 0x7fffffffU) != 0) {
            for(int i = 0; i < canp->config->txbuffer_count; i++) {
                if((newly_cleared & (1U << i)) != 0) {
                    _can_tx_empty_isr(canp, i+CAN_TX_MAILBOX_BUFFER0);
                }
            }
        }
    }
}

static void can_lld_set_std_filter ( CANDriver *canp,  uint8_t filter, MCAN_Std_Filter_Element value )
{
    osalDbgAssert ( filter < canp->config->std_flt_count, "Filter entry does not exist" );

    MCAN_Std_Filter_Element *elem =
        ( MCAN_Std_Filter_Element * ) ( ( ( char* ) canp->std_flt ) + filter*4 );

    if ( elem->sfec != 0 ) {
        //first disable the filter. this will prevent this filter from being
        //used with partially set data.
        //for reject filters, the effect is to now allow more messages!
        elem->sfec = 0;
        if ( ( SCB->CCR & SCB_CCR_DC_Msk ) != 0 ) {
            //D-Cache enabled. need to take care of cleaning/invalidating
            SCB_CleanDCache_by_Addr ( ( uint32_t* ) elem, 4 );
        }
    }
    if ( value.sfec != 0 ) {
        elem->sfid1 = value.sfid1;
        elem->sfid2 = value.sfid2;
        elem->sft = value.sft;
        elem->unused_12 = value.unused_12;
        if ( ( SCB->CCR & SCB_CCR_DC_Msk ) != 0 ) {
            //D-Cache enabled. need to take care of cleaning/invalidating
            SCB_CleanDCache_by_Addr ( ( uint32_t* ) elem, 4 );
        }
        //setting sfec reenables the filter. we do that after we are
        //sure our changes have been written to memory.
        elem->sfec = value.sfec;
        if ( ( SCB->CCR & SCB_CCR_DC_Msk ) != 0 ) {
            //D-Cache enabled. need to take care of cleaning/invalidating
            SCB_CleanDCache_by_Addr ( ( uint32_t* ) elem, 4 );
        }
    }
}

static void can_lld_set_ext_filter ( CANDriver *canp,  uint8_t filter, MCAN_Ext_Filter_Element value )
{
    osalDbgAssert ( filter < canp->config->ext_flt_count, "Filter entry does not exist" );

    MCAN_Ext_Filter_Element *elem =
        ( MCAN_Ext_Filter_Element * ) ( ( ( char* ) canp->ext_flt ) + filter*8 );

    if ( elem->efec != 0 ) {
        //first disable the filter. this will prevent this filter from being
        //used with partially set data.
        //for reject filters, the effect is to now allow more messages!
        elem->efec = 0;
        if ( ( SCB->CCR & SCB_CCR_DC_Msk ) != 0 ) {
            //D-Cache enabled. need to take care of cleaning/invalidating
            SCB_CleanDCache_by_Addr ( ( uint32_t* ) elem, 8 );
        }
    }
    if ( value.efec != 0 ) {
        elem->efid1 = value.efid1;
        elem->efid2 = value.efid2;
        elem->eft = value.eft;
        elem->unused_61 = value.unused_61;
        if ( ( SCB->CCR & SCB_CCR_DC_Msk ) != 0 ) {
            //D-Cache enabled. need to take care of cleaning/invalidating
            SCB_CleanDCache_by_Addr ( ( uint32_t* ) elem, 8 );
        }
        //setting efec reenables the filter. we do that after we are
        //sure our changes have been written to memory.
        elem->efec = value.efec;
        if ( ( SCB->CCR & SCB_CCR_DC_Msk ) != 0 ) {
            //D-Cache enabled. need to take care of cleaning/invalidating
            SCB_CleanDCache_by_Addr ( ( uint32_t* ) elem, 8 );
        }
    }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (SAMV71_CAN_USE_CAN0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(MCAN0_INT0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    can_lld_serve_interrupt0(&CAND0);
    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(MCAN0_INT1_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    can_lld_serve_interrupt1(&CAND0);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_CAN_USE_CAN1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(MCAN1_INT0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    can_lld_serve_interrupt0(&CAND1);
    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(MCAN1_INT1_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    can_lld_serve_interrupt1(&CAND1);
    OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level CAN driver initialization.
 *
 * @notapi
 */
void can_lld_init(void) {

#if SAMV71_CAN_USE_CAN0 == TRUE
  /* Driver initialization.*/
  canObjectInit(&CAND0);
  CAND0.device = MCAN0;
#endif
#if SAMV71_CAN_USE_CAN1 == TRUE
  /* Driver initialization.*/
  canObjectInit(&CAND1);
  CAND1.device = MCAN1;
#endif
}

/**
 * @brief   Configures and activates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
msg_t can_lld_start ( CANDriver *canp )
{
    /* canp->state == CAN_STARTING */

    /* run memory allocation, bail if the region is too small or crosses
     * a 64kByte boundary */

    //first, we run the DMA bits. Later, we try to fit the timestamp overflow counter arrays.
    char *mem_dma_base = NULL;
    char *mem_dma_end = NULL;
    {
        char *memp = canp->config->memory;
        memp = ( char * ) ( ( ( ( uintptr_t ) memp )+31U ) & ~31U );
        mem_dma_base = memp;
        canp->std_flt = memp;
        memp += 4 * canp->config->std_flt_count;
        canp->ext_flt = memp;
        memp += 8 * canp->config->ext_flt_count;
        canp->txbuffers = memp;
        memp += MCAN_MEMORY_PAD_TO ( 8+canp->config->txelem_data_size, 4 ) *
                ( canp->config->txbuffer_count + canp->config->txfifo_count );
        memp = ( char * ) ( ( ( ( uintptr_t ) memp )+31U ) & ~31U );
        canp->rxfifo0 = memp;
        memp += MCAN_MEMORY_PAD_TO ( 8+canp->config->rxfifo0_data_size, 4 ) *
                canp->config->rxfifo0_count;
        canp->rxfifo1 = memp;
        memp += MCAN_MEMORY_PAD_TO ( 8+canp->config->rxfifo1_data_size, 4 ) *
                canp->config->rxfifo1_count;
        canp->rxbuffers = memp;
        memp += MCAN_MEMORY_PAD_TO ( 8+canp->config->rxbuffer_data_size, 4 ) *
                canp->config->rxbuffer_count;
        canp->txeventfifo = memp;
        memp += 8*canp->config->txevent_count;
        memp = ( char * ) ( ( ( ( uintptr_t ) memp )+31U ) & ~31U );
        mem_dma_end = memp;
    }

    // check that the last address is in the same 64kByte block as the first
    // if not, move everything so mem_base is at the start of the next 64kByte
    // block. The size check afterwards catches the case where there was not
    // enough memory.
    char *mem_end = mem_dma_end;
    if ( ( ( ( ( ( uintptr_t ) mem_dma_end )-1 ) ^ ( ( uintptr_t ) mem_dma_base ) ) & ~0xffffU ) != 0 ) {
        //since the timer overflow counters cannot be larger than 896 bytes, place them at the begin.
        //this may already get the dma arena into the same 64k block.
        uint32_t offset = 0x10000 - ( ( ( uintptr_t ) mem_dma_base ) & 0xffff );
        char *new_dma_base = ( ( char* ) canp->config->memory ) +
                             4* ( canp->config->rxfifo0_count+canp->config->rxfifo1_count+
                                  canp->config->rxbuffer_count + canp->config->txevent_count );
        if ( mem_dma_base + offset < new_dma_base ) {
            offset = new_dma_base - mem_dma_base;
            offset = (offset + 31) & ~31;
        }
        mem_dma_base += offset;
        canp->std_flt += offset;
        canp->ext_flt += offset;
        canp->txbuffers += offset;
        canp->rxfifo0 += offset;
        canp->rxfifo1 += offset;
        canp->rxbuffers += offset;
        canp->txeventfifo += offset;
        mem_dma_end += offset;

        char *memp = canp->config->memory;
        canp->rxfifo0_timeroverflowcount = (uint32_t*)memp;
        memp += 4*canp->config->rxfifo0_count;
        canp->rxfifo1_timeroverflowcount = (uint32_t*)memp;
        memp += 4*canp->config->rxfifo1_count;
        canp->rxbuffers_timeroverflowcount = (uint32_t*)memp;
        memp += 4*canp->config->rxbuffer_count;
        canp->txeventfifo_timeroverflowcount = (uint32_t*)memp;
        memp += 4*canp->config->txevent_count;

        mem_end = mem_dma_end;
    } else {
        char *memp = mem_end;
        canp->rxfifo0_timeroverflowcount = (uint32_t*)memp;
        memp += 4*canp->config->rxfifo0_count;
        canp->rxfifo1_timeroverflowcount = (uint32_t*)memp;
        memp += 4*canp->config->rxfifo1_count;
        canp->rxbuffers_timeroverflowcount = (uint32_t*)memp;
        memp += 4*canp->config->rxbuffer_count;
        canp->txeventfifo_timeroverflowcount = (uint32_t*)memp;
        memp += 4*canp->config->txevent_count;
        mem_end = memp;
    }

    //these are soft errors; the caller may be able to try a different memory region.
    //at the same time, these must always be errors, otherwise memory will be
    //corrupted.
    if ( ( size_t ) ( mem_end - ( ( char* ) canp->config->memory ) ) > canp->config->memory_size ) {
        return HAL_RET_CONFIG_ERROR;
    }

    // check config: hardware limits
    osalDbgCheck ( canp->config->std_flt_count <= 128 );
    osalDbgCheck ( canp->config->ext_flt_count <= 64 );
    osalDbgCheck ( canp->config->rxfifo0_count <= 64 );
    osalDbgCheck ( canp->config->rxfifo1_count <= 64 );
    osalDbgCheck ( canp->config->rxbuffer_count <= 64 );
    osalDbgCheck ( canp->config->txfifo_count + canp->config->txbuffer_count <= 32 );
    osalDbgCheck ( canp->config->txevent_count <= 32 );
    // check config: invalid data sizes
    osalDbgCheck ( convert_data_size ( canp->config->rxfifo0_data_size ) != -1 ||
                   canp->config->rxfifo0_count == 0 );
    osalDbgCheck ( convert_data_size ( canp->config->rxfifo1_data_size ) != -1  ||
                   canp->config->rxfifo1_count == 0 );
    osalDbgCheck ( convert_data_size ( canp->config->rxbuffer_data_size ) != -1 ||
                   canp->config->rxbuffer_count == 0 );
    osalDbgCheck ( convert_data_size ( canp->config->txelem_data_size ) != -1 ||
                   canp->config->txfifo_count + canp->config->txbuffer_count == 0 );

    osalDbgAssert ( canp->config->rxfifo0_count != 0 ||
                    ( ( canp->config->gfc & MCAN_GFC_ANFS_Msk ) != MCAN_GFC_ANFS_RX_FIFO_0 &&
                      ( canp->config->gfc & MCAN_GFC_ANFE_Msk ) != MCAN_GFC_ANFE_RX_FIFO_0 ),
                    "The catch all fifos configured in GFC must have entries" );
    osalDbgAssert ( canp->config->rxfifo1_count != 0 ||
                    ( ( canp->config->gfc & MCAN_GFC_ANFS_Msk ) != MCAN_GFC_ANFS_RX_FIFO_1 &&
                      ( canp->config->gfc & MCAN_GFC_ANFE_Msk ) != MCAN_GFC_ANFE_RX_FIFO_1 ),
                    "The catch all fifos configured in GFC must have entries" );

    memset ( canp->config->memory, 0, canp->config->memory_size );
    if ( ( SCB->CCR & SCB_CCR_DC_Msk ) != 0 ) {
        //D-Cache enabled. need to take care of cleaning/invalidating
        SCB_CleanDCache_by_Addr (
            ( uint32_t * ) mem_dma_base,
            mem_dma_end - mem_dma_base );
    }

    canp->rxbuffer_status = 0ULL;
    canp->txbuffer_status = 0;
    canp->timerOverflowCounter = 0;
    canp->rxfifo0_lastPutPos = 0;
    canp->rxfifo1_lastPutPos = 0;
    canp->txeventfifo_lastPutPos = 0;

    /* Enables the peripheral.*/
#if SAMV71_CAN_USE_CAN0 == TRUE
    if ( &CAND0 == canp ) {
        // Enable interrupts
        nvicEnableVector ( MCAN0_INT0_NVIC_NUMBER, MCAN_NVIC_PRIORITY );
        nvicEnableVector ( MCAN0_INT1_NVIC_NUMBER, MCAN_NVIC_PRIORITY );
        // Enable clock source
        pmc_enable_periph_clk ( ID_MCAN0 );

        // Set the DMA offset(high 16 bits)
        matrix_set_periph_master_addr ( MATRIX_MASTER_ID_CAN0, mem_dma_base );
    }
#endif
#if SAMV71_CAN_USE_CAN1 == TRUE
    if ( &CAND1 == canp ) {
        // Enable interrupt
        nvicEnableVector ( MCAN1_INT0_NVIC_NUMBER, MCAN_NVIC_PRIORITY );
        nvicEnableVector ( MCAN1_INT1_NVIC_NUMBER, MCAN_NVIC_PRIORITY );
        // Enable clock source
        pmc_enable_periph_clk ( ID_MCAN1 );

        // Set the DMA offset(high 16 bits)
        matrix_set_periph_master_addr ( MATRIX_MASTER_ID_CAN1, mem_dma_base );
    }
#endif

    /* Configures the peripheral.*/
    //CCCR_INIT and CCCR_CCE must be set one after the other.
    uint32_t cccr = canp->device->MCAN_CCCR;
    cccr |= MCAN_CCCR_INIT;
    canp->device->MCAN_CCCR = cccr;
    cccr |= MCAN_CCCR_CCE;
    canp->device->MCAN_CCCR = cccr;

    //configure registers

    //for FD mode, the docs make it important to setup the transmitter delay; there is an automatic measurement method for that

    int prescaler = (CAN_CORE_CLOCK + canp->config->fd_data_bitrate * 49) /
                    ( canp->config->fd_data_bitrate * 49 );
    //in both cases, 1 clock for sync
    // sample point is recommended to be at 87.5% 7/8th
    int seg1 = CAN_CORE_CLOCK*7/prescaler/canp->config->fd_data_bitrate/8 - 1;
    int seg2 = CAN_CORE_CLOCK/prescaler/canp->config->fd_data_bitrate - seg1 - 1;
    osalDbgAssert( prescaler <= 32 && seg1 <= 32 && seg2 <= 16, "Invalid fd data phase timing");
    //maximum number of clocks that can be removed from phase_seg2/added during
    //synch; used during arbitration to compensate for different senders at
    //different distances, else only to adjust for clock differences.
    //This setting here(for the data phase) is never used for arbitration.
    int sjw = seg1;
    if(sjw > seg2)
        sjw = seg2;
    if(sjw > 8)
        sjw = 8;
    canp->device->MCAN_DBTP = (canp->config->tdcr & MCAN_DBTP_TDC_ENABLED) |
                              MCAN_DBTP_DBRP ( prescaler-1 ) | // prescaler from CAN_CORE_CLOCK; 0..31 meaning /1 .. /32
                              MCAN_DBTP_DTSEG1 ( seg1-1 ) | // duration up to sample point; 1..31 meaning 2..32 clock times; sum of Prop_Seg and Phase_Seg1
                              MCAN_DBTP_DTSEG2 ( seg2-1 ) | // duration after sample point; 0..15 meaning 1..16 clock times; Phase_Seg2
                              MCAN_DBTP_DSJW ( sjw-1 ); // sync width; 0..7 meaning 1..8 clock times

    // disable message ram watchdog
    canp->device->MCAN_RWD = MCAN_RWD_WDC ( 0 );

    cccr &= ~MCAN_CONFIG_CCCR_MASK;
    cccr |= canp->config->cccr & MCAN_CONFIG_CCCR_MASK;
    canp->device->MCAN_CCCR = cccr;

    prescaler = (CAN_CORE_CLOCK + canp->config->nominal_bitrate * 385) /
                    ( canp->config->nominal_bitrate * 385 );
    //in both cases, 1 clock for sync
    // sample point is recommended to be at 87.5% 7/8th
    seg1 = CAN_CORE_CLOCK*7/prescaler/canp->config->nominal_bitrate/8 - 1;
    seg2 = CAN_CORE_CLOCK/prescaler/canp->config->nominal_bitrate - seg1 - 1;
    osalDbgAssert( prescaler <= 128 && seg1 <= 256 && seg2 <= 128, "Invalid nominal timing" );
    //maximum number of clocks that can be removed from phase_seg2/added during
    //synch; used during arbitration to compensate for different senders at
    //different distances, else only to adjust for clock differences.
    sjw = seg1;
    if(sjw > seg2)
        sjw = seg2;
    if(sjw > 128)
        sjw = 128;
    canp->device->MCAN_NBTP = MCAN_NBTP_NSJW( sjw-1 ) | // sync width; 0..127 meaning 1..128 clock times
                              MCAN_NBTP_NBRP ( prescaler-1 ) | // prescaler from CAN_CORE_CLOCK; 0..511 meaning /1 .. /512
                              MCAN_NBTP_NTSEG1 ( seg1-1 ) | // duration up to sample point; 1..255 meaning 2..256 clock times sum of Prop_Seg and Phase_Seg1
                              MCAN_NBTP_NTSEG2 ( seg2-1 ); // duration after sample point; 1..127 meaning 2..128 clock times; Phase_Seg2

    canp->device->MCAN_TDCR = canp->config->tdcr & ~MCAN_DBTP_TDC_ENABLED;

    /* we are interested in any of the error interrupt sources, specifically
     *
     * PED: Protocol Error in Data Phase  _can_error_isr(CAN_FRAMING_ERROR)
     * PEA: Protocol Error in Arbitration Phase  _can_error_isr(CAN_FRAMING_ERROR)
     * BO: Bus off status  _can_error_isr(CAN_BUS_OFF_ERROR)
     * EW: Warning status  _can_error_isr(CAN_LIMIT_WARNING)
     * RF1L: Receive Fifo 1 message lost  _can_error_isr(CAN_OVERFLOW_ERROR)
     * RF0L: Receive Fifo 0 message lost   _can_error_isr(CAN_OVERFLOW_ERROR)
     *
     * the rx interrupts:
     * DRX: Message stored to dedicated read buffer _can_rx_full_isr(mailbox)
     * RF1N: New message written to receive fifo 1 _can_rx_full_isr(2)
     * RF0N: New message written to receive fifo 0 _can_rx_full_isr(1)
     *
     * the tx complete interrupts
     * TC: Transmission completed (to figure out if one of the tx buffers finished) _can_tx_empty_isr(mailbox)
     *
     */

    //the device is not active, yet, so no special care needs to be taken
    //ordering the register writes to the interrupt registers
    canp->device->MCAN_IR = ~0U; // clear all interrupt sources
    uint32_t ie = MCAN_IE_PEDE | MCAN_IE_PEAE | MCAN_IE_BOE |
                  MCAN_IE_EWE | MCAN_IE_TCE;
    if ( canp->config->rxbuffer_count > 0 ) {
        ie |= MCAN_IE_DRXE;
    }
    if ( canp->config->rxfifo0_count > 0 ) {
        ie |= MCAN_IE_RF0LE | MCAN_IE_RF0NE;
    }
    if ( canp->config->rxfifo1_count > 0 ) {
        ie |= MCAN_IE_RF1LE | MCAN_IE_RF1NE;
    }
    if ( canp->config->txevent_count > 0 ) {
        ie |= MCAN_IE_TEFNE;
    }
    if((canp->config->tscc & MCAN_TSCC_TSS_Msk) == MCAN_TSCC_TSS_TCP_INC) {
        ie |= MCAN_IE_TSWE;
    }

    canp->device->MCAN_IE = ie;
    canp->device->MCAN_ILS = MCAN_ILS_PEDL | MCAN_ILS_PEAL | MCAN_ILS_BOL |
                            MCAN_ILS_EWL | MCAN_ILS_RF1LL | MCAN_ILS_RF0LL |
                            MCAN_ILS_TCL | MCAN_ILS_TEFLL;
    canp->device->MCAN_ILE = MCAN_ILE_EINT0 | MCAN_ILE_EINT1;


    canp->device->MCAN_SIDFC = MCAN_SIDFC_FLSSA ( (( ( uintptr_t ) canp->std_flt ) & 0xfffc ) >> 2 ) |
                               MCAN_SIDFC_FLSSA ( canp->config->std_flt_count );
    canp->device->MCAN_XIDFC = MCAN_XIDFC_FLESA ( (( ( uintptr_t ) canp->ext_flt ) & 0xfffc ) >> 2 ) |
                               MCAN_XIDFC_LSE ( canp->config->ext_flt_count );

    canp->device->MCAN_RXF0C = MCAN_RXF0C_F0SA ( ( ( ( uintptr_t ) canp->rxfifo0 ) & 0xfffc ) >> 2 ) |
    /* MCAN_RXF0C_F0OM | */ /* MCAN_RXF0C_F0WM(?) | */ MCAN_RXF0C_F0S(canp->config->rxfifo0_count);
    canp->device->MCAN_RXF1C = MCAN_RXF1C_F1SA ( ( ( ( uintptr_t ) canp->rxfifo1 ) & 0xfffc ) >> 2 ) |
    /* MCAN_RXF1C_F1OM | */ /* MCAN_RXF1C_F1WM(?) | */ MCAN_RXF1C_F1S(canp->config->rxfifo1_count);
    canp->device->MCAN_RXBC = MCAN_RXBC_RBSA ( ( ( ( uintptr_t ) canp->rxbuffers ) & 0xfffc ) >> 2 );
    canp->device->MCAN_RXESC =
        MCAN_RXESC_F0DS ( convert_data_size ( canp->config->rxfifo0_data_size ) & 7 ) |
        MCAN_RXESC_F1DS ( convert_data_size ( canp->config->rxfifo1_data_size ) & 7 ) |
        MCAN_RXESC_RBDS ( convert_data_size ( canp->config->rxbuffer_data_size ) & 7 );
    canp->device->MCAN_TXBC = MCAN_TXBC_TBSA ( ( ( ( uintptr_t ) canp->txbuffers ) & 0xfffc ) >> 2 ) |
                              /* MCAN_TXBC_TFQM | */ MCAN_TXBC_TFQS ( canp->config->txfifo_count ) |
                              MCAN_TXBC_NDTB ( canp->config->txbuffer_count );
    canp->device->MCAN_TXESC =
        MCAN_TXESC_TBDS ( convert_data_size ( canp->config->txelem_data_size ) & 7 );
    canp->device->MCAN_TXEFC = MCAN_TXEFC_EFSA ( ( ( ( uintptr_t ) canp->txeventfifo ) & 0xfffc ) >> 2 ) |
    /* MCAN_TXEFC_EFWM(?) | */ MCAN_TXEFC_EFS(canp->config->txevent_count);

    canp->device->MCAN_TXBTIE = (1U << (canp->config->txbuffer_count + canp->config->txfifo_count)) -1;

    //the number of rxbuffers used is determined by the setup of the filters;
    //if a dedicated rx buffer filter is setup, that rxbuffer element will be used.
    //this driver will have to make sure that no more rxbuffers than allocated
    //are used.

    canp->device->MCAN_TEST = canp->config->test;
    canp->device->MCAN_TSCC = canp->config->tscc;
    canp->device->MCAN_GFC = canp->config->gfc;
    canp->device->MCAN_XIDAM = canp->config->xidam;

    //this needs to be a separate write so the changes have already been committed.
    cccr &= ~MCAN_CCCR_INIT;
    canp->device->MCAN_CCCR = cccr;

    return HAL_RET_SUCCESS;
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_stop ( CANDriver *canp )
{

  if (canp->state == CAN_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if SAMV71_CAN_USE_CAN0 == TRUE
    if (&CAND0 == canp) {
        // Disable clock source
        pmc_disable_periph_clk(ID_MCAN0);
        // Disable interrupts
        nvicDisableVector(MCAN0_INT0_NVIC_NUMBER);
        nvicDisableVector(MCAN0_INT1_NVIC_NUMBER);
    }
#endif
#if SAMV71_CAN_USE_CAN1 == TRUE
    if (&CAND1 == canp) {
        // Disable clock source
        pmc_disable_periph_clk(ID_MCAN1);
        // Disable interrupts
        nvicDisableVector(MCAN1_INT0_NVIC_NUMBER);
        nvicDisableVector(MCAN1_INT1_NVIC_NUMBER);
    }
#endif
  }
}

/**
 * @brief   Determines whether a frame can be transmitted.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval false        no space in the transmit queue.
 * @retval true         transmit slot available.
 *
 * @notapi
 */
bool can_lld_is_tx_empty ( CANDriver *canp, canmbx_t mailbox )
{
    osalDbgAssert ( mailbox != CAN_TX_MAILBOX_FIFO || canp->config->txfifo_count > 0,
                    "Queued mailbox not configured" );
    osalDbgAssert ( mailbox < CAN_TX_MAILBOX_BUFFER0 || mailbox == CAN_ANY_MAILBOX ||
                    mailbox < (canmbx_t)canp->config->txbuffer_count + CAN_TX_MAILBOX_BUFFER0,
                    "Mailbox not configured" );

    if ( mailbox == CAN_ANY_MAILBOX ) {
        if ( canp->config->txfifo_count > 0 &&
                can_lld_is_tx_empty ( canp, CAN_TX_MAILBOX_FIFO ) )
            return true;
        for ( canmbx_t i = CAN_TX_MAILBOX_BUFFER0; i < (canmbx_t)canp->config->txbuffer_count+CAN_TX_MAILBOX_BUFFER0; i++ ) {
            if ( can_lld_is_tx_empty ( canp, i ) ) {
                return true;
            }
        }
        return false;
    } else if (mailbox == CAN_TX_MAILBOX_FIFO) {
        //this is the txfifo. it "is empty" if there is at least one free fifo element
        return (canp->device->MCAN_TXFQS & MCAN_TXFQS_TFQF) == 0;
    } else {
        //the buffers, essentially true mailboxes with single items;
        //these are output using their sid/eid as priority
        int buffer_index = mailbox - CAN_TX_MAILBOX_BUFFER0;
        return (canp->device->MCAN_TXBRP & (MCAN_TXBRP_TRP0 << buffer_index )) == 0;
    }
}

/**
 * @brief   Inserts a frame into the transmit queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @param[in] mailbox   mailbox number,  @p CAN_ANY_MAILBOX for any mailbox
 *
 * @notapi
 */
void can_lld_transmit(CANDriver *canp,
                      canmbx_t mailbox,
                      const CANTxFrame *ctfp) {
    osalDbgAssert ( mailbox != CAN_TX_MAILBOX_FIFO || canp->config->txfifo_count > 0,
                    "Queued mailbox not configured" );
    osalDbgAssert ( mailbox < CAN_TX_MAILBOX_BUFFER0 || mailbox == CAN_ANY_MAILBOX ||
                    mailbox < ((canmbx_t)canp->config->txbuffer_count) + CAN_TX_MAILBOX_BUFFER0,
                    "Mailbox not configured" );
    osalDbgAssert ( DLC_to_size[ctfp->DLC] <= canp->config->txelem_data_size,
                    "Message data too big" );

    if ( mailbox == CAN_ANY_MAILBOX ) {
        if ( canp->config->txfifo_count > 0 && can_lld_is_tx_empty ( canp, CAN_TX_MAILBOX_FIFO ) )
            return can_lld_transmit ( canp, CAN_TX_MAILBOX_FIFO, ctfp );
        for ( canmbx_t i = CAN_TX_MAILBOX_BUFFER0; i < (canmbx_t)canp->config->txbuffer_count+CAN_TX_MAILBOX_BUFFER0; i++ ) {
            if ( can_lld_is_tx_empty ( canp, i ) ) {
                return can_lld_transmit ( canp, i, ctfp );
            }
        }
    } else if ( mailbox == CAN_TX_MAILBOX_FIFO ) {
        //this is the txfifo
        osalDbgAssert ( can_lld_is_tx_empty ( canp, mailbox ), "Queued tx mailbox full" );
        int buffer_index = ( canp->device->MCAN_TXFQS & MCAN_TXFQS_TFQPI_Msk ) >> MCAN_TXFQS_TFQPI_Pos;
        //the index is based on the start of the whole array, not just the fifo part.
        char *fifo_elem_base = ( ( char* ) canp->txbuffers ) +
                               ( sizeof ( MCAN_TX_Element_Header ) +
                                 canp->config->txelem_data_size ) * buffer_index;
        fill_tx_fifo_elem_for_packet ( fifo_elem_base, ctfp, canp->config->txelem_data_size );
        canp->txbuffer_status |= 0x80000000U;
        canp->device->MCAN_TXBAR = MCAN_TXBAR_AR0 << buffer_index;
    } else {
        //the buffers, essentially true mailboxes with single items;
        //these are output using their sid/eid as priority
        osalDbgAssert ( can_lld_is_tx_empty ( canp, mailbox ), "Tx mailbox not empty" );
        int buffer_index = mailbox - CAN_TX_MAILBOX_BUFFER0;
        char *fifo_elem_base = ( ( char* ) canp->txbuffers ) +
                               ( sizeof ( MCAN_TX_Element_Header ) +
                                 canp->config->txelem_data_size ) * buffer_index;
        fill_tx_fifo_elem_for_packet ( fifo_elem_base, ctfp, canp->config->txelem_data_size );
        canp->txbuffer_status |= 1 << buffer_index;
        canp->device->MCAN_TXBAR = MCAN_TXBAR_AR0 << buffer_index;
    }
}

/**
 * @brief   Determines whether a frame has been received.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval false        no space in the transmit queue.
 * @retval true         transmit slot available.
 *
 * @notapi
 */
bool can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox) {
    osalDbgAssert ( mailbox != CAN_RX_MAILBOX_FIFO0 || canp->config->rxfifo0_count > 0,
                    "Queued mailbox not configured" );
    osalDbgAssert ( mailbox != CAN_RX_MAILBOX_FIFO1 || canp->config->rxfifo1_count > 0,
                    "Queued mailbox not configured" );
    osalDbgAssert ( mailbox < CAN_RX_MAILBOX_BUFFER0 || mailbox == CAN_ANY_MAILBOX ||
                    mailbox == CAN_RX_MAILBOX_TXEVENT ||
                    mailbox < ((canmbx_t)canp->config->rxbuffer_count) + CAN_RX_MAILBOX_BUFFER0,
                    "Mailbox not configured" );

    if(mailbox == CAN_ANY_MAILBOX) {
        if ( canp->config->rxfifo0_count > 0 ) {
            if ( can_lld_is_rx_nonempty ( canp, CAN_RX_MAILBOX_FIFO0 ) )
                return true;
        }
        if ( canp->config->rxfifo1_count > 0 ) {
            if ( can_lld_is_rx_nonempty ( canp, CAN_RX_MAILBOX_FIFO1 ) )
                return true;
        }
        for ( canmbx_t i = CAN_RX_MAILBOX_BUFFER0;
                i < ( canmbx_t ) canp->config->rxbuffer_count+CAN_RX_MAILBOX_BUFFER0;
                i++ ) {
            if ( can_lld_is_rx_nonempty ( canp, i ) )
                return true;
        }
        return false;
    } else if (mailbox == CAN_RX_MAILBOX_FIFO0 || mailbox == CAN_RX_MAILBOX_FIFO1) {
        //one of the two fifos
        uint32_t const volatile *RXFxS;
        //one of the two fifos
        if(mailbox == CAN_RX_MAILBOX_FIFO0) {
            RXFxS = &canp->device->MCAN_RXF0S;
        } else if ( mailbox == CAN_RX_MAILBOX_FIFO1 ) {
            RXFxS = &canp->device->MCAN_RXF1S;
        } else {
            return false;
        }
        return ((*RXFxS & MCAN_RXF0S_F0FL_Msk) >> MCAN_RXF0S_F0FL_Pos) != 0;
    } else if ( mailbox == CAN_RX_MAILBOX_TXEVENT ) {
        //the tx event fifo
        return ((canp->device->MCAN_TXEFS & MCAN_TXEFS_EFFL_Msk) >> MCAN_TXEFS_EFFL_Pos) != 0;
    } else {
        //one of the buffers
        int buffer_index = mailbox - CAN_RX_MAILBOX_BUFFER0;
        if ( buffer_index < 32 ) {
            return ( ( canp->device->MCAN_NDAT1 & ( 1 << buffer_index ) ) != 0 ) ||
                   ( ( canp->rxbuffer_status & ( 1ULL << buffer_index ) ) != 0 );
        } else {
            return ( ( canp->device->MCAN_NDAT2 & ( 1 << ( buffer_index-32 ) ) ) != 0 ) ||
                   ( ( canp->rxbuffer_status & ( 1ULL << buffer_index ) ) != 0 );
        }
    }
    return false;
}

/**
 * @brief   Receives a frame from the input queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 *
 * @notapi
 */
void can_lld_receive(CANDriver *canp,
                     canmbx_t mailbox,
                     CANRxFrame *crfp) {
    osalDbgAssert ( mailbox != CAN_RX_MAILBOX_FIFO0 || canp->config->rxfifo0_count > 0,
                    "Queued mailbox not configured" );
    osalDbgAssert ( mailbox != CAN_RX_MAILBOX_FIFO1 || canp->config->rxfifo1_count > 0,
                    "Queued mailbox not configured" );
    osalDbgAssert ( mailbox < CAN_RX_MAILBOX_BUFFER0 || mailbox == CAN_ANY_MAILBOX ||
                    mailbox == CAN_RX_MAILBOX_TXEVENT ||
                    mailbox < ((canmbx_t)canp->config->rxbuffer_count) + CAN_RX_MAILBOX_BUFFER0,
                    "Mailbox not configured" );

    if(mailbox == CAN_ANY_MAILBOX) {
        if ( canp->config->rxfifo0_count > 0 ) {
            if ( can_lld_is_rx_nonempty ( canp, CAN_RX_MAILBOX_FIFO0 ) ) {
                can_lld_receive ( canp, CAN_RX_MAILBOX_FIFO0, crfp );
                return;
            }
        }
        if ( canp->config->rxfifo1_count > 0 ) {
            if ( can_lld_is_rx_nonempty ( canp, CAN_RX_MAILBOX_FIFO1 ) ) {
                can_lld_receive ( canp, CAN_RX_MAILBOX_FIFO1, crfp );
                return;
            }
        }
        for ( canmbx_t i = CAN_RX_MAILBOX_BUFFER0;
                i < ( canmbx_t ) canp->config->rxbuffer_count+CAN_RX_MAILBOX_BUFFER0;
                i++ ) {
            if ( can_lld_is_rx_nonempty ( canp, i ) ) {
                can_lld_receive ( canp, i, crfp );
                return;
            }
        }
    } else if (mailbox == CAN_RX_MAILBOX_FIFO0 || mailbox == CAN_RX_MAILBOX_FIFO1) {
        osalDbgAssert ( can_lld_is_rx_nonempty ( canp, mailbox ), "Queued rx mailbox empty" );

        uint32_t const volatile *RXFxS;
        uint32_t volatile *RXFxA;
        void *mem;
        uint8_t rxelem_data_size;
        //one of the two fifos
        if(mailbox == CAN_RX_MAILBOX_FIFO0) {
            RXFxS = &canp->device->MCAN_RXF0S;
            RXFxA = &canp->device->MCAN_RXF0A;
            mem = canp->rxfifo0;
            rxelem_data_size = canp->config->rxfifo0_data_size;
        } else if ( mailbox == CAN_RX_MAILBOX_FIFO1 ) {
            RXFxS = &canp->device->MCAN_RXF1S;
            RXFxA = &canp->device->MCAN_RXF1A;
            mem = canp->rxfifo1;
            rxelem_data_size = canp->config->rxfifo1_data_size;
        } else {
            return;
        }
        int buffer_index = (*RXFxS & MCAN_RXF0S_F0GI_Msk) >> MCAN_RXF0S_F0GI_Pos;

        char *fifo_elem_base = mem +
                               ( sizeof ( MCAN_RX_Element_Header ) +
                                 rxelem_data_size ) * buffer_index;

        parse_rx_fifo_elem_to_packet(crfp, fifo_elem_base, rxelem_data_size);

        *RXFxA = MCAN_RXF0A_F0AI(buffer_index);
    } else if ( mailbox == CAN_RX_MAILBOX_TXEVENT ) {
        osalDbgAssert ( can_lld_is_rx_nonempty ( canp, mailbox ), "Queued rx mailbox empty" );

        int buffer_index = (canp->device->MCAN_TXEFS & MCAN_RXF0S_F0GI_Msk) >> MCAN_RXF0S_F0GI_Pos;

        char *fifo_elem_base = canp->txeventfifo +
                               sizeof ( MCAN_TX_Event_Element_Header ) * buffer_index;

        parse_txevent_fifo_elem_to_packet(crfp, fifo_elem_base);

        canp->device->MCAN_TXEFA = MCAN_RXF0A_F0AI(buffer_index);
    } else {
        //one of the buffers
        osalDbgAssert ( can_lld_is_rx_nonempty ( canp, mailbox ), "Rx mailbox empty" );
        int buffer_index = mailbox - CAN_RX_MAILBOX_BUFFER0;
        char *fifo_elem_base = ( ( char* ) canp->rxbuffers ) +
                               ( sizeof ( MCAN_RX_Element_Header ) +
                                 canp->config->rxbuffer_data_size ) * buffer_index;

        parse_rx_fifo_elem_to_packet ( crfp, fifo_elem_base, canp->config->rxbuffer_data_size );

        canp->rxbuffer_status &= ~ ( 1ULL << buffer_index );
        if ( buffer_index < 32 ) {
            canp->device->MCAN_NDAT1 = ( 1 << buffer_index );
        } else {
            canp->device->MCAN_NDAT2 = ( 1 << ( buffer_index - 32 ) );
        }
    }
}

/**
 * @brief   Tries to abort an ongoing transmission.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number
 *
 * @notapi
 */
void can_lld_abort(CANDriver *canp,
                   canmbx_t mailbox) {
    osalDbgAssert ( mailbox != CAN_TX_MAILBOX_FIFO || canp->config->txfifo_count > 0,
                    "Queued mailbox not configured" );
    osalDbgAssert ( mailbox < CAN_TX_MAILBOX_BUFFER0 || mailbox == CAN_ANY_MAILBOX ||
                    mailbox < ((canmbx_t)canp->config->txbuffer_count) + CAN_TX_MAILBOX_BUFFER0,
                    "Mailbox not configured" );
    osalDbgCheck ( mailbox != CAN_ANY_MAILBOX );
    osalDbgAssert ( !can_lld_is_tx_empty ( canp, mailbox ), "Mailbox empty" );

    if ( mailbox <= CAN_TX_MAILBOX_FIFO ) {
        //entries in the fifo can be aborted the same way as buffers.
        //question is, which element to abort: the oldest or the newest written?
        //also has some interesting effects on the read/write position of the fifo.
        return;
    } else {
        int buffer_index = mailbox - CAN_TX_MAILBOX_BUFFER0;
        canp->device->MCAN_TXBCR = MCAN_TXBCR_CR0 << buffer_index;
        //we could now wait for the cancellation to be finished using TXBCF
    }
}

#if (CAN_USE_SLEEP_MODE == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_sleep(CANDriver *canp) {

  (void)canp;

}

/**
 * @brief   Enforces leaving the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_wakeup(CANDriver *canp) {

  (void)canp;

}
#endif /* CAN_USE_SLEEP_MOD == TRUEE */

void canTimerOverflowI ( CANDriver* canp )
{
    osalDbgCheckClassI();
    canp->timerOverflowCounter++;
}


void canFilterStdDisableX ( CANDriver *canp,  uint8_t filter )
{
    MCAN_Std_Filter_Element elem = {0};

    elem.sfec = 0;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtDisableX ( CANDriver *canp, uint8_t filter )
{
    MCAN_Ext_Filter_Element elem = {0};

    elem.efec = 0;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetFifoRangeX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint16_t SID1, uint16_t SID2, bool HPM )
{
    osalDbgAssert ( mailbox >= 1 && mailbox <= 2, "Must use a fifo mailbox" );
    osalDbgAssert ( SID1 <= SID2, "SID1 must be less than or equal SID2" );
    MCAN_Std_Filter_Element elem = {0};

    if ( HPM ) {
        elem.sfec = 5+mailbox-1;
    } else {
        elem.sfec = 1+mailbox-1;
    }
    elem.sft = 0;
    elem.sfid1 = SID1;
    elem.sfid2 = SID2;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetFifoRangeX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint32_t EID1, uint32_t EID2, bool HPM, bool applyGlobalMasking )
{
    osalDbgAssert ( mailbox >= 1 && mailbox <= 2, "Must use a fifo mailbox" );
    osalDbgAssert ( EID1 <= EID2, "EID1 must be less than or equal EID2" );
    MCAN_Ext_Filter_Element elem = {0};

    if ( HPM ) {
        elem.efec = 5+mailbox-1;
    } else {
        elem.efec = 1+mailbox-1;
    }
    if ( applyGlobalMasking ) {
        elem.eft = 0;
    } else {
        elem.eft = 3;
    }
    elem.efid1 = EID1;
    elem.efid2 = EID2;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetFifoIDPairX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint16_t SID1, uint16_t SID2, bool HPM )
{
    osalDbgAssert ( mailbox >= 1 && mailbox <= 2, "Must use a fifo mailbox" );
    MCAN_Std_Filter_Element elem = {0};

    if ( HPM ) {
        elem.sfec = 5+mailbox-1;
    } else {
        elem.sfec = 1+mailbox-1;
    }
    elem.sft = 1;
    elem.sfid1 = SID1;
    elem.sfid2 = SID2;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetFifoIDPairX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint32_t EID1, uint32_t EID2, bool HPM )
{
    osalDbgAssert ( mailbox >= 1 && mailbox <= 2, "Must use a fifo mailbox" );
    MCAN_Ext_Filter_Element elem = {0};

    if ( HPM ) {
        elem.efec = 5+mailbox-1;
    } else {
        elem.efec = 1+mailbox-1;
    }
    elem.eft = 1;
    elem.efid1 = EID1;
    elem.efid2 = EID2;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetFifoIDMaskX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint16_t SID, uint16_t SMASK, bool HPM )
{
    osalDbgAssert ( mailbox >= 1 && mailbox <= 2, "Must use a fifo mailbox" );
    MCAN_Std_Filter_Element elem = {0};

    if ( HPM ) {
        elem.sfec = 5+mailbox-1;
    } else {
        elem.sfec = 1+mailbox-1;
    }
    elem.sft = 2;
    elem.sfid1 = SID & SMASK;
    elem.sfid2 = SMASK;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetFifoIDMaskX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint32_t EID, uint32_t EMASK, bool HPM )
{
    osalDbgAssert ( mailbox >= 1 && mailbox <= 2, "Must use a fifo mailbox" );
    MCAN_Ext_Filter_Element elem = {0};

    if ( HPM ) {
        elem.efec = 5+mailbox-1;
    } else {
        elem.efec = 1+mailbox-1;
    }
    elem.eft = 2;
    elem.efid1 = EID & EMASK;
    elem.efid2 = EMASK;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetRejectRangeX ( CANDriver *canp, uint8_t filter, uint16_t SID1, uint16_t SID2 )
{
    osalDbgAssert ( SID1 <= SID2, "SID1 must be less than or equal SID2" );
    MCAN_Std_Filter_Element elem = {0};

    elem.sfec = 3;
    elem.sft = 0;
    elem.sfid1 = SID1;
    elem.sfid2 = SID2;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetRejectRangeX ( CANDriver *canp, uint8_t filter, uint32_t EID1, uint32_t EID2, bool applyGlobalMasking )
{
    osalDbgAssert ( EID1 <= EID2, "EID1 must be less than or equal EID2" );
    MCAN_Ext_Filter_Element elem = {0};

    elem.efec = 3;
    if ( applyGlobalMasking ) {
        elem.eft = 0;
    } else {
        elem.eft = 3;
    }
    elem.efid1 = EID1;
    elem.efid2 = EID2;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetRejectIDPairX ( CANDriver *canp, uint8_t filter, uint16_t SID1, uint16_t SID2 )
{
    MCAN_Std_Filter_Element elem = {0};

    elem.sfec = 3;
    elem.sft = 1;
    elem.sfid1 = SID1;
    elem.sfid2 = SID2;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetRejectIDPairX ( CANDriver *canp, uint8_t filter, uint32_t EID1, uint32_t EID2 )
{
    MCAN_Ext_Filter_Element elem = {0};

    elem.efec = 3;
    elem.eft = 1;
    elem.efid1 = EID1;
    elem.efid2 = EID2;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetRejectIDMaskX ( CANDriver *canp, uint8_t filter, uint16_t SID, uint16_t SMASK )
{
    MCAN_Std_Filter_Element elem = {0};

    elem.sfec = 3;
    elem.sft = 2;
    elem.sfid1 = SID & SMASK;
    elem.sfid2 = SMASK;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetRejectIDMaskX ( CANDriver *canp, uint8_t filter, uint32_t EID, uint32_t EMASK )
{
    MCAN_Ext_Filter_Element elem = {0};

    elem.efec = 3;
    elem.eft = 2;
    elem.efid1 = EID & EMASK;
    elem.efid2 = EMASK;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetMarkHPMRangeX ( CANDriver *canp, uint8_t filter, uint16_t SID1, uint16_t SID2 )
{
    osalDbgAssert ( SID1 <= SID2, "SID1 must be less than or equal SID2" );
    MCAN_Std_Filter_Element elem = {0};

    elem.sfec = 4;
    elem.sft = 0;
    elem.sfid1 = SID1;
    elem.sfid2 = SID2;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetMarkHPMRangeX ( CANDriver *canp, uint8_t filter, uint32_t EID1, uint32_t EID2, bool applyGlobalMasking )
{
    osalDbgAssert ( EID1 <= EID2, "EID1 must be less than or equal EID2" );
    MCAN_Ext_Filter_Element elem = {0};

    elem.efec = 4;
    if ( applyGlobalMasking ) {
        elem.eft = 0;
    } else {
        elem.eft = 3;
    }
    elem.efid1 = EID1;
    elem.efid2 = EID2;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetMarkHPMIDPairX ( CANDriver *canp, uint8_t filter, uint16_t SID1, uint16_t SID2 )
{
    MCAN_Std_Filter_Element elem = {0};

    elem.sfec = 4;
    elem.sft = 1;
    elem.sfid1 = SID1;
    elem.sfid2 = SID2;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetMarkHPMIDPairX ( CANDriver *canp, uint8_t filter, uint32_t EID1, uint32_t EID2 )
{
    MCAN_Ext_Filter_Element elem = {0};

    elem.efec = 4;
    elem.eft = 1;
    elem.efid1 = EID1;
    elem.efid2 = EID2;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetMarkHPMIDMaskX ( CANDriver *canp, uint8_t filter, uint16_t SID, uint16_t SMASK )
{
    MCAN_Std_Filter_Element elem = {0};

    elem.sfec = 4;
    elem.sft = 2;
    elem.sfid1 = SID & SMASK;
    elem.sfid2 = SMASK;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetMarkHPMIDMaskX ( CANDriver *canp, uint8_t filter, uint32_t EID, uint32_t EMASK )
{
    MCAN_Ext_Filter_Element elem = {0};

    elem.efec = 4;
    elem.eft = 2;
    elem.efid1 = EID & EMASK;
    elem.efid2 = EMASK;
    can_lld_set_ext_filter ( canp, filter, elem );
}

void canFilterStdSetBufferFilterX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint16_t SID )
{
    osalDbgAssert ( mailbox >= 3 && mailbox < (unsigned)canp->config->rxbuffer_count+3, "Must use a buffer mailbox" );
    MCAN_Std_Filter_Element elem = {0};

    elem.sfec = 7;
    elem.sfid1 = SID;
    elem.sfid2 = mailbox - CAN_RX_MAILBOX_BUFFER0;
    can_lld_set_std_filter ( canp, filter, elem );
}

void canFilterExtSetBufferFilterX ( CANDriver *canp, uint8_t filter, canmbx_t mailbox, uint32_t EID )
{
    osalDbgAssert ( mailbox >= 3 && mailbox < (unsigned)canp->config->rxbuffer_count+3, "Must use a buffer mailbox" );
    MCAN_Ext_Filter_Element elem = {0};

    elem.efec = 7;
    elem.eft = 1;
    elem.efid1 = EID;
    elem.efid2 = mailbox - CAN_RX_MAILBOX_BUFFER0;
    can_lld_set_ext_filter ( canp, filter, elem );
}

#endif /* HAL_USE_CAN == TRUE */

/** @} */
