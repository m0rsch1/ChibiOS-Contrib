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
 * @file    hal_spi_lld.c
 * @brief   SAMV71 spi subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 spi driver identifier.*/
#if (SAMV71_SPI_USE_SPI0 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID0;
#endif
#if (SAMV71_SPI_USE_SPI1 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif
#if (SAMV71_SPI_USE_QSPI == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/* not marked const so it appears in RAM instead of FLASH */
static uint32_t spi_lld_constant_output_value = 0xffffffffU;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static int spi_lld_isSpi(SPIDriver *spip) {
  (void)spip;
#if SAMV71_SPI_USE_SPI0 == TRUE
  if(spip == &SPID0) {
    return TRUE;
  }
#endif
#if SAMV71_SPI_USE_SPI1 == TRUE
  if(spip == &SPID1) {
    return TRUE;
  }
#endif
  return FALSE;
}

static int spi_lld_isQspi(SPIDriver *spip) {
  (void)spip;
#if SAMV71_SPI_USE_QSPI == TRUE
  if(spip == &SPID2) {
    return TRUE;
  }
#endif
  return FALSE;
}

static Spi *spi_lld_getSpiDevice(SPIDriver *spip) {
  (void)spip;
#if SAMV71_SPI_USE_SPI0 == TRUE
  if(spip == &SPID0) {
    return SPI0;
  }
#endif
#if SAMV71_SPI_USE_SPI1 == TRUE
  if(spip == &SPID1) {
    return SPI1;
  }
#endif
  return NULL;
}

static Qspi *spi_lld_getQspiDevice(SPIDriver *spip) {
  (void)spip;
#if SAMV71_SPI_USE_QSPI == TRUE
  if(spip == &SPID2) {
    return QSPI;
  }
#endif
  return NULL;
}

static void spi_lld_setup_circular_buffer_recv(SPIDriver *spip,
    size_t block1_count, size_t block2_count, void *rxbuf, uint32_t dwidth) {
  uint32_t *recv_addr = NULL;
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    recv_addr = (uint32_t *) & (device->SPI_RDR);
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    recv_addr = (uint32_t *) & (device->QSPI_RDR);
  }

  spip->dma_recv_descriptors[0].XDMAC_MBR_NDA =
    (samv71_xdmac_linked_list_base_t *)&spip->dma_recv_descriptors[1];
  spip->dma_recv_descriptors[0].XDMAC_MBR_UBC = XDMAC_MBR_UBC_NDE |
      XDMAC_MBR_UBC_NDEN | XDMAC_MBR_UBC_NVIEW_NDV0 |
      XDMAC_MBR_UBC_UBLEN(block1_count);
  spip->dma_recv_descriptors[0].XDMAC_MBR_TA = rxbuf;

  spip->dma_recv_descriptors[1].XDMAC_MBR_NDA =
    (samv71_xdmac_linked_list_base_t *)&spip->dma_recv_descriptors[0];
  spip->dma_recv_descriptors[1].XDMAC_MBR_UBC =
    XDMAC_MBR_UBC_NDEN | XDMAC_MBR_UBC_NVIEW_NDV0 |
    XDMAC_MBR_UBC_UBLEN(block2_count);
  spip->dma_recv_descriptors[1].XDMAC_MBR_TA = rxbuf + block1_count;
  spip->dma_recv_descriptors[1].XDMAC_MBR_UBC |= XDMAC_MBR_UBC_NDE;

  DCACHE_WRITE_BACK_ALIGNED(spip->dma_recv_descriptors, sizeof(*spip->dma_recv_descriptors) * 2);

  xdmacChannelSetSource(spip->dma_recv_channel, recv_addr);
  xdmacChannelSetDestination(spip->dma_recv_channel, rxbuf);
  xdmacChannelSetMicroblockLength(spip->dma_recv_channel, 0);

  xdmacChannelSetMode(spip->dma_recv_channel,
                      XDMAC_CC_TYPE_PER_TRAN |
                      XDMAC_CC_MBSIZE_SIXTEEN |
                      XDMAC_CC_SWREQ_HWR_CONNECTED |
                      XDMAC_CC_SAM_FIXED_AM |
                      XDMAC_CC_DAM_INCREMENTED_AM |
                      XDMAC_CC_DSYNC_PER2MEM |
                      XDMAC_CC_CSIZE_CHK_1 |
                      dwidth |
                      xdmacAutomaticInterfaceBits_CCReg(
                        recv_addr, rxbuf) |
                      XDMAC_CC_PERID(spip->dma_recv_hwid));
  xdmacChannelSetBlockLength(spip->dma_recv_channel, 0);
  xdmacChannelSetNextDescriptorMode(spip->dma_recv_channel, 0);
  xdmacChannelSetStride(spip->dma_recv_channel, 0);
  xdmacChannelSetSourceMicroblockStride(spip->dma_recv_channel, 0);
  xdmacChannelSetDestinationMicroblockStride(spip->dma_recv_channel, 0);
  xdmacChannelSetInterruptCauses(spip->dma_recv_channel, XDMAC_CIE_BIE |
                                 XDMAC_CIE_LIE | XDMAC_CIE_RBIE |
                                 XDMAC_CIE_WBIE | XDMAC_CIE_ROIE);
  xdmacChannelStartMasterTransfer(spip->dma_recv_channel,
                                  XDMAC_CNDC_NDVIEW_NDV0 |
                                  XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED,
                                  (block1_count == 0) ?
                                  (samv71_xdmac_linked_list_base_t *)&spip->dma_recv_descriptors[1] :
                                  (samv71_xdmac_linked_list_base_t *)&spip->dma_recv_descriptors[0],
                                  0);
}

static void spi_lld_setup_linear_buffer_recv(SPIDriver *spip, size_t n,
    void *rxbuf, uint32_t dwidth) {
  uint32_t *recv_addr = NULL;
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    recv_addr = (uint32_t *) & (device->SPI_RDR);
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    recv_addr = (uint32_t *) & (device->QSPI_RDR);
  }

  xdmacChannelSetInterruptCauses(spip->dma_recv_channel,
                                 XDMAC_CIE_LIE | XDMAC_CIE_RBIE |
                                 XDMAC_CIE_WBIE | XDMAC_CIE_ROIE);
  xdmacChannelStartSingleMicroblock(spip->dma_recv_channel,
                                    XDMAC_CC_TYPE_PER_TRAN |
                                    XDMAC_CC_MBSIZE_SIXTEEN |
                                    XDMAC_CC_SWREQ_HWR_CONNECTED |
                                    XDMAC_CC_SAM_FIXED_AM |
                                    XDMAC_CC_DAM_INCREMENTED_AM |
                                    XDMAC_CC_DSYNC_PER2MEM |
                                    XDMAC_CC_CSIZE_CHK_1 |
                                    dwidth |
                                    xdmacAutomaticInterfaceBits_CCReg(
                                      recv_addr, rxbuf) |
                                    XDMAC_CC_PERID(spip->dma_recv_hwid),
                                    rxbuf,
                                    recv_addr,
                                    n);
}

static void spi_lld_setup_circular_buffer_send(SPIDriver *spip,
    size_t block1_count, size_t block2_count, const void *txbuf,
    uint32_t dwidth, bool send_only) {
  uint32_t *send_addr = NULL;
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    send_addr = (uint32_t *) & (device->SPI_TDR);
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    send_addr = (uint32_t *) & (device->QSPI_TDR);
  }

  spip->dma_send_descriptors[0].XDMAC_MBR_NDA =
    (samv71_xdmac_linked_list_base_t *)&spip->dma_send_descriptors[1];
  spip->dma_send_descriptors[0].XDMAC_MBR_UBC = XDMAC_MBR_UBC_NDE |
      XDMAC_MBR_UBC_NSEN | XDMAC_MBR_UBC_NVIEW_NDV0 |
      XDMAC_MBR_UBC_UBLEN(block1_count);
  spip->dma_send_descriptors[0].XDMAC_MBR_TA = txbuf;

  spip->dma_send_descriptors[1].XDMAC_MBR_NDA =
    (samv71_xdmac_linked_list_base_t *)&spip->dma_send_descriptors[0];
  spip->dma_send_descriptors[1].XDMAC_MBR_UBC =
    XDMAC_MBR_UBC_NSEN | XDMAC_MBR_UBC_NVIEW_NDV0 |
    XDMAC_MBR_UBC_UBLEN(block2_count);
  spip->dma_send_descriptors[1].XDMAC_MBR_TA = txbuf + block1_count;
  spip->dma_send_descriptors[1].XDMAC_MBR_UBC |= XDMAC_MBR_UBC_NDE;

  DCACHE_WRITE_BACK_ALIGNED(spip->dma_send_descriptors, sizeof(*spip->dma_send_descriptors) * 2);

  xdmacChannelSetSource(spip->dma_send_channel, txbuf);
  xdmacChannelSetDestination(spip->dma_send_channel, send_addr);
  xdmacChannelSetMicroblockLength(spip->dma_send_channel, 0);

  xdmacChannelSetMode(spip->dma_send_channel,
                      XDMAC_CC_TYPE_PER_TRAN |
                      XDMAC_CC_MBSIZE_SIXTEEN |
                      XDMAC_CC_SWREQ_HWR_CONNECTED |
                      XDMAC_CC_SAM_INCREMENTED_AM |
                      XDMAC_CC_DAM_FIXED_AM |
                      XDMAC_CC_DSYNC_MEM2PER |
                      XDMAC_CC_CSIZE_CHK_1 |
                      dwidth |
                      xdmacAutomaticInterfaceBits_CCReg(
                        txbuf, send_addr) |
                      XDMAC_CC_PERID(spip->dma_send_hwid));
  xdmacChannelSetBlockLength(spip->dma_send_channel, 0);
  xdmacChannelSetNextDescriptorMode(spip->dma_send_channel, 0);
  xdmacChannelSetStride(spip->dma_send_channel, 0);
  xdmacChannelSetSourceMicroblockStride(spip->dma_send_channel, 0);
  xdmacChannelSetDestinationMicroblockStride(spip->dma_send_channel, 0);
  xdmacChannelSetInterruptCauses(spip->dma_send_channel,
                                 (send_only ? (XDMAC_CIE_BIE | XDMAC_CIE_LIE) : 0) |
                                 XDMAC_CIE_RBIE |
                                 XDMAC_CIE_WBIE | XDMAC_CIE_ROIE);
  xdmacChannelStartMasterTransfer(spip->dma_send_channel,
                                  XDMAC_CNDC_NDVIEW_NDV0 |
                                  XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED,
                                  (block1_count == 0) ?
                                  (samv71_xdmac_linked_list_base_t *)&spip->dma_send_descriptors[1] :
                                  (samv71_xdmac_linked_list_base_t *)&spip->dma_send_descriptors[0],
                                  0);
}

static void spi_lld_setup_circular_constant_send(SPIDriver *spip,
    size_t block1_count, size_t block2_count, uint32_t dwidth, bool send_only) {
  uint32_t *send_addr = NULL;
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    send_addr = (uint32_t *) & (device->SPI_TDR);
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    send_addr = (uint32_t *) & (device->QSPI_TDR);
  }

  const void *tx_ptr = &spi_lld_constant_output_value;

  spip->dma_send_descriptors[0].XDMAC_MBR_NDA =
    (samv71_xdmac_linked_list_base_t *)&spip->dma_send_descriptors[1];
  spip->dma_send_descriptors[0].XDMAC_MBR_UBC = XDMAC_MBR_UBC_NDE |
      XDMAC_MBR_UBC_NSEN | XDMAC_MBR_UBC_NVIEW_NDV0 |
      XDMAC_MBR_UBC_UBLEN(block1_count);
  spip->dma_send_descriptors[0].XDMAC_MBR_TA = tx_ptr;

  spip->dma_send_descriptors[1].XDMAC_MBR_NDA =
    (samv71_xdmac_linked_list_base_t *)&spip->dma_send_descriptors[0];
  spip->dma_send_descriptors[1].XDMAC_MBR_UBC = XDMAC_MBR_UBC_NDE |
      XDMAC_MBR_UBC_NSEN | XDMAC_MBR_UBC_NVIEW_NDV0 |
      XDMAC_MBR_UBC_UBLEN(block2_count);
  spip->dma_send_descriptors[1].XDMAC_MBR_TA = tx_ptr;

  DCACHE_WRITE_BACK_ALIGNED(spip->dma_send_descriptors, sizeof(*spip->dma_send_descriptors) * 2);

  xdmacChannelSetSource(spip->dma_send_channel, tx_ptr);
  xdmacChannelSetDestination(spip->dma_send_channel, send_addr);
  xdmacChannelSetMicroblockLength(spip->dma_send_channel, 0);

  xdmacChannelSetMode(spip->dma_send_channel,
                      XDMAC_CC_TYPE_PER_TRAN |
                      XDMAC_CC_MBSIZE_SIXTEEN |
                      XDMAC_CC_SWREQ_HWR_CONNECTED |
                      XDMAC_CC_SAM_FIXED_AM |
                      XDMAC_CC_DAM_FIXED_AM |
                      XDMAC_CC_DSYNC_MEM2PER |
                      XDMAC_CC_CSIZE_CHK_1 |
                      dwidth |
                      xdmacAutomaticInterfaceBits_CCReg(
                        send_addr, send_addr) |
                      XDMAC_CC_PERID(spip->dma_send_hwid));
  xdmacChannelSetBlockLength(spip->dma_send_channel, 0);
  xdmacChannelSetNextDescriptorMode(spip->dma_send_channel, 0);
  xdmacChannelSetSourceMicroblockStride(spip->dma_send_channel, 0);
  xdmacChannelSetDestinationMicroblockStride(spip->dma_send_channel, 0);
  xdmacChannelSetInterruptCauses(spip->dma_send_channel,
                                 (send_only ? (XDMAC_CIE_BIE | XDMAC_CIE_LIE) : 0) |
                                 XDMAC_CIE_RBIE |
                                 XDMAC_CIE_WBIE | XDMAC_CIE_ROIE);
  xdmacChannelStartMasterTransfer(spip->dma_send_channel,
                                  XDMAC_CNDC_NDVIEW_NDV0 |
                                  XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED,
                                  (block1_count == 0) ?
                                  (samv71_xdmac_linked_list_base_t *)&spip->dma_send_descriptors[1] :
                                  (samv71_xdmac_linked_list_base_t *)&spip->dma_send_descriptors[0],
                                  0);
}

static void spi_lld_setup_linear_buffer_send(SPIDriver *spip, size_t n,
    const void *txbuf, uint32_t dwidth, bool send_only) {
  uint32_t *send_addr = NULL;
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    send_addr = (uint32_t *) & (device->SPI_TDR);
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    send_addr = (uint32_t *) & (device->QSPI_TDR);
  }

  xdmacChannelSetInterruptCauses(spip->dma_send_channel,
                                 (send_only ? XDMAC_CIE_LIE : 0) | XDMAC_CIE_RBIE |
                                 XDMAC_CIE_WBIE | XDMAC_CIE_ROIE);
  xdmacChannelStartSingleMicroblock(spip->dma_send_channel,
                                    XDMAC_CC_TYPE_PER_TRAN |
                                    XDMAC_CC_MBSIZE_SIXTEEN |
                                    XDMAC_CC_SWREQ_HWR_CONNECTED |
                                    XDMAC_CC_SAM_INCREMENTED_AM |
                                    XDMAC_CC_DAM_FIXED_AM |
                                    XDMAC_CC_DSYNC_MEM2PER |
                                    XDMAC_CC_CSIZE_CHK_1 |
                                    dwidth |
                                    xdmacAutomaticInterfaceBits_CCReg(
                                      txbuf, send_addr) |
                                    XDMAC_CC_PERID(spip->dma_send_hwid),
                                    send_addr,
                                    txbuf,
                                    n);
}

static void spi_lld_setup_linear_constant_send(SPIDriver *spip, size_t n,
    uint32_t dwidth, bool send_only) {
  uint32_t *send_addr = NULL;
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    send_addr = (uint32_t *) & (device->SPI_TDR);
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    send_addr = (uint32_t *) & (device->QSPI_TDR);
  }

  const void *tx_ptr = &spi_lld_constant_output_value;

  xdmacChannelSetInterruptCauses(spip->dma_send_channel,
                                 (send_only ? XDMAC_CIE_LIE : 0) | XDMAC_CIE_RBIE |
                                 XDMAC_CIE_WBIE | XDMAC_CIE_ROIE);
  xdmacChannelStartSingleMicroblock(spip->dma_send_channel,
                                    XDMAC_CC_TYPE_PER_TRAN |
                                    XDMAC_CC_MBSIZE_SIXTEEN |
                                    XDMAC_CC_SWREQ_HWR_CONNECTED |
                                    XDMAC_CC_SAM_FIXED_AM |
                                    XDMAC_CC_DAM_FIXED_AM |
                                    XDMAC_CC_DSYNC_MEM2PER |
                                    XDMAC_CC_CSIZE_CHK_1 |
                                    dwidth |
                                    xdmacAutomaticInterfaceBits_CCReg(
                                      tx_ptr, send_addr) |
                                    XDMAC_CC_PERID(spip->dma_send_hwid),
                                    send_addr,
                                    tx_ptr,
                                    n);
}

static void spi_lld_stop_abort(SPIDriver *spip) {
  xdmacChannelDisable(spip->dma_send_channel);
  xdmacChannelDisable(spip->dma_recv_channel);
}

/**
 * @brief Common IRQ handler for spi drivers
 */
static void spi_lld_serve_spi_interrupt(SPIDriver *spip)
{
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    uint32_t sr = device->SPI_SR;
    uint32_t imr = device->SPI_IMR;
    sr &= imr;

    if ((sr & (SPI_SR_UNDES | SPI_SR_OVRES | SPI_SR_MODF)) != 0 &&
        spip->state == SPI_ACTIVE) {
      /* Aborting the transfer.*/
      (void) spi_lld_stop_abort(spip);

      /* Reporting the failure.*/
      __spi_isr_error_code(spip, HAL_RET_HW_FAILURE);
    }
    if ((sr & SPI_SR_TXEMPTY) != 0) {
      device->SPI_IDR = SPI_IDR_TXEMPTY;
      if (spip->state == SPI_ACTIVE) {
        if (spip->config->circular) {
          /* this does not matter in circular buffer mode.
          * the callbacks are driven from the recv side.
          */
        } else {
          __spi_isr_complete_code(spip)
        }
      }
    }
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    uint32_t sr = device->QSPI_SR;
    uint32_t imr = device->QSPI_IMR;
    sr &= imr;

    if ((sr & (QSPI_SR_OVRES)) != 0 &&
        spip->state == SPI_ACTIVE) {
      /* Aborting the transfer.*/
      (void) spi_lld_stop_abort(spip);

      /* Reporting the failure.*/
      __spi_isr_error_code(spip, HAL_RET_HW_FAILURE);
    }
    if ((sr & QSPI_SR_TXEMPTY) != 0) {
      device->QSPI_IDR = QSPI_IDR_TXEMPTY;
      if (spip->state == SPI_ACTIVE) {
        if (spip->config->circular) {
          /* this does not matter in circular buffer mode.
          * the callbacks are driven from the recv side.
          */
        } else {
          __spi_isr_complete_code(spip)
        }
      }
    }
  }
}

/**
 * @brief Send DMA callback for spi drivers
 */
static void spi_lld_send_dma_func(void *param, uint32_t flags)
{
  SPIDriver *spip = (SPIDriver *)param;
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);

    if ((flags & (XDMAC_CIS_RBEIS | XDMAC_CIS_WBEIS | XDMAC_CIS_ROIS)) != 0 &&
        spip->state == SPI_ACTIVE) {
      /* Aborting the transfer.*/
      (void) spi_lld_stop_abort(spip);

      /* Reporting the failure.*/
      __spi_isr_error_code(spip, HAL_RET_HW_FAILURE);
    }
    if ((flags & (XDMAC_CIS_BIS | XDMAC_CIS_LIS)) != 0 &&
        spip->state == SPI_ACTIVE) {
      device->SPI_IER = SPI_IER_TXEMPTY;
    }
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);

    if ((flags & (XDMAC_CIS_RBEIS | XDMAC_CIS_WBEIS | XDMAC_CIS_ROIS)) != 0 &&
        spip->state == SPI_ACTIVE) {
      /* Aborting the transfer.*/
      (void) spi_lld_stop_abort(spip);

      /* Reporting the failure.*/
      __spi_isr_error_code(spip, HAL_RET_HW_FAILURE);
    }
    if ((flags & (XDMAC_CIS_BIS | XDMAC_CIS_LIS)) != 0 &&
        spip->state == SPI_ACTIVE) {
      device->QSPI_IER = QSPI_IER_TXEMPTY;
    }
  }
}

/**
  * @brief Receive DMA callback for spi drivers
  */
static void spi_lld_recv_dma_func(void *param, uint32_t flags)
{
  SPIDriver *spip = (SPIDriver *)param;

  if ((flags & (XDMAC_CIS_RBEIS | XDMAC_CIS_WBEIS | XDMAC_CIS_ROIS)) != 0 &&
      spip->state == SPI_ACTIVE) {
    /* Aborting the transfer.*/
    (void) spi_lld_stop_abort(spip);

    /* Reporting the failure.*/
    __spi_isr_error_code(spip, HAL_RET_HW_FAILURE);
  }
  if ((flags & (XDMAC_CIS_BIS | XDMAC_CIS_LIS)) != 0 &&
      spip->state == SPI_ACTIVE) {
    if (spip->config->circular) {
      //figure out which half we are in. if the next descriptor is #1,
      //we are in the first and vice versa.
      if (xdmacChannelGetNextDescriptor(spip->dma_recv_channel) ==
          (samv71_xdmac_linked_list_base_t *) & (spip->dma_recv_descriptors[1])) {
        //the second half just completed, working on the first half again.
        __spi_isr_full_code(spip);
      } else {
        //the first half just completed, working on the second half again.
        __spi_isr_half_code(spip);
      }
    } else {
      __spi_isr_complete_code(spip)
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (SAMV71_SPI_USE_SPI0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(SPI0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    spi_lld_serve_spi_interrupt(&SPID0);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SPI_USE_SPI1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(SPI1_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    spi_lld_serve_spi_interrupt(&SPID1);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (SAMV71_SPI_USE_QSPI == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(SPI_QSPI_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    spi_lld_serve_spi_interrupt(&SPID2);
    OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level spi driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {
#if SAMV71_SPI_USE_SPI0 == TRUE
  /* Driver initialization.*/
  spiObjectInit(&SPID0);
  SPID0.dma_recv_hwid = SAMV71_XDMAC_HWREQ_SPI0_RECV;
  SPID0.dma_send_hwid = SAMV71_XDMAC_HWREQ_SPI0_XMIT;
  SPID0.dma_recv_descriptors = (samv71_xdmac_linked_list_view_0_t*)CACHE_ALIGN_AFTER(SPID0.dma_recv_descriptors_buf);
  SPID0.dma_send_descriptors = (samv71_xdmac_linked_list_view_0_t*)CACHE_ALIGN_AFTER(SPID0.dma_send_descriptors_buf);
#endif
#if SAMV71_SPI_USE_SPI1 == TRUE
  /* Driver initialization.*/
  spiObjectInit(&SPID1);
  SPID1.dma_recv_hwid = SAMV71_XDMAC_HWREQ_SPI1_RECV;
  SPID1.dma_send_hwid = SAMV71_XDMAC_HWREQ_SPI1_XMIT;
  SPID1.dma_recv_descriptors = (samv71_xdmac_linked_list_view_0_t*)CACHE_ALIGN_AFTER(SPID1.dma_recv_descriptors_buf);
  SPID1.dma_send_descriptors = (samv71_xdmac_linked_list_view_0_t*)CACHE_ALIGN_AFTER(SPID1.dma_send_descriptors_buf);
#endif
#if SAMV71_SPI_USE_QSPI == TRUE
  /* Driver initialization.*/
  spiObjectInit(&SPID2);
  SPID2.dma_recv_hwid = SAMV71_XDMAC_HWREQ_QSPI_RECV;
  SPID2.dma_send_hwid = SAMV71_XDMAC_HWREQ_QSPI_XMIT;
  SPID2.dma_recv_descriptors = (samv71_xdmac_linked_list_view_0_t*)CACHE_ALIGN_AFTER(SPID2.dma_recv_descriptors_buf);
  SPID2.dma_send_descriptors = (samv71_xdmac_linked_list_view_0_t*)CACHE_ALIGN_AFTER(SPID2.dma_send_descriptors_buf);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_start(SPIDriver *spip) {

  if (spip->state == SPI_STOP) {

    samv71_xdmacprio_t recv_dma_prio = 23;
    samv71_xdmacprio_t send_dma_prio = 23;
    /* Enables the peripheral.*/
    if (false) {
    }

#if SAMV71_SPI_USE_SPI0 == TRUE
    else if (&SPID0 == spip) {
        /* Enable interrupt */
        nvicEnableVector(SPI0_NVIC_NUMBER, SPI_NVIC_PRIORITY);
        /* Enable clock source */
        pmc_enable_periph_clk(ID_SPI0);
        recv_dma_prio = SAMV71_SPI0_RECV_DMA_PRIO;
        send_dma_prio = SAMV71_SPI0_SEND_DMA_PRIO;
    }
#endif
#if SAMV71_SPI_USE_SPI1 == TRUE
    else if (&SPID1 == spip) {
        /* Enable interrupt */
        nvicEnableVector(SPI1_NVIC_NUMBER, SPI_NVIC_PRIORITY);
        /* Enable clock source */
        pmc_enable_periph_clk(ID_SPI1);
        recv_dma_prio = SAMV71_SPI1_RECV_DMA_PRIO;
        send_dma_prio = SAMV71_SPI1_SEND_DMA_PRIO;
    }
#endif
#if SAMV71_SPI_USE_QSPI == TRUE
    else if (&SPID2 == spip) {
        /* Enable interrupt */
        nvicEnableVector(SPI_QSPI_NVIC_NUMBER, SPI_NVIC_PRIORITY);
        /* Enable clock source */
        pmc_enable_periph_clk(ID_QSPI);
        recv_dma_prio = SAMV71_SPI_QSPI_RECV_DMA_PRIO;
        send_dma_prio = SAMV71_SPI_QSPI_SEND_DMA_PRIO;
    }
#endif

    else {
      osalDbgAssert(false, "invalid SPI instance");
    }

    spip->dma_recv_channel = xdmacChannelAllocI(spi_lld_recv_dma_func, spip, recv_dma_prio);
    spip->dma_send_channel = xdmacChannelAllocI(spi_lld_send_dma_func, spip, send_dma_prio);

    if (spi_lld_isSpi(spip)) {
      Spi *device = spi_lld_getSpiDevice(spip);

      device->SPI_CR = SPI_CR_SWRST;
      device->SPI_MR =
        (spip->config->mr & (SPI_MR_DLYBCS_Msk | SPI_MR_LLB | SPI_MR_WDRBT |
                             SPI_MR_MODFDIS | SPI_MR_PCSDEC)) |
        (spip->config->slave ? 0 : SPI_MR_MSTR);

      for (int i = 0; i < SPI_SUPPORTED_SLAVES; i++) {
        uint32_t csr = spip->config->slave_configs[i].csr &
                       (SPI_CSR_BITS_Msk | SPI_CSR_CPOL | SPI_CSR_CSAAT |
                        SPI_CSR_CSNAAT | SPI_CSR_DLYBCT_Msk | SPI_CSR_DLYBS_Msk |
                        SPI_CSR_NCPHA);
        //ceil(clk/baudrate)=(clk+baudrate-1)/baudrate=
        //(clk-1)/baudrate+baudrate/baudrate=(clk-1)/baudrate+1
        int divider = ((SPI_MAIN_CLOCK - 1) /
                       spip->config->slave_configs[i].baudrate) + 1;
        if (divider < 1) {
          divider = 1;
        }
        if (divider > 255) {
          divider = 255;
        }
        csr |= SPI_CSR_SCBR(divider);
        device->SPI_CSR[i] = csr;
      }

      spip->active_slave_config = 0;

      device->SPI_IER = SPI_IER_MODF | SPI_IER_OVRES | SPI_IER_UNDES;

      device->SPI_CR = SPI_CR_SPIEN;
    }
    if (spi_lld_isQspi(spip)) {
      osalDbgAssert(!spip->config->slave, "QSPI cannot do slave mode");

      Qspi *device = spi_lld_getQspiDevice(spip);

      device->QSPI_CR = QSPI_CR_SWRST;
      device->QSPI_MR =
        (spip->config->mr & (QSPI_MR_DLYCS_Msk | QSPI_MR_DLYBCT_Msk | QSPI_MR_LLB | QSPI_MR_WDRBT |
                             QSPI_MR_CSMODE_Msk | QSPI_MR_NBBITS_Msk | QSPI_MR_WDRBT));

      uint32_t scr = spip->config->slave_configs[0].csr &
                     (QSPI_SCR_CPOL | QSPI_SCR_DLYBS_Msk |
                      QSPI_SCR_CPHA);
      //ceil(clk/baudrate)=(clk+baudrate-1)/baudrate=
      //(clk-1)/baudrate+baudrate/baudrate=(clk-1)/baudrate+1
      int divider = ((SPI_QSPI_MAIN_CLOCK - 1) /
                     spip->config->slave_configs[0].baudrate) + 1;
      if (divider < 1) {
        divider = 1;
      }
      if (divider > 255) {
        divider = 255;
      }
      scr |= QSPI_SCR_SCBR(divider);
      device->QSPI_SCR = scr;

      spip->active_slave_config = 0;

      device->QSPI_IER = QSPI_IER_OVRES;

      device->QSPI_CR = QSPI_CR_QSPIEN;
    }
  }

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  if (spip->state == SPI_READY) {

    if (spip->dma_recv_channel) {
      xdmacChannelFreeI(spip->dma_recv_channel);
      spip->dma_recv_channel = NULL;
    }

    if (spip->dma_send_channel) {
      xdmacChannelFreeI(spip->dma_send_channel);
      spip->dma_send_channel = NULL;
    }

    if (spi_lld_isSpi(spip)) {
      Spi *device = spi_lld_getSpiDevice(spip);
      device->SPI_CR = SPI_CR_SPIDIS;
    }
    if (spi_lld_isQspi(spip)) {
      Qspi *device = spi_lld_getQspiDevice(spip);
      device->QSPI_CR = QSPI_CR_QSPIDIS;
    }

    /* Disables the peripheral.*/
    if (false) {
    }

#if SAMV71_SPI_USE_SPI0 == TRUE
    else if (&SPID0 == spip) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_SPI0);
        /* Disable interrupt */
        nvicDisableVector(SPI0_NVIC_NUMBER);
    }
#endif
#if SAMV71_SPI_USE_SPI1 == TRUE
    else if (&SPID1 == spip) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_SPI1);
        /* Disable interrupt */
        nvicDisableVector(SPI1_NVIC_NUMBER);
    }
#endif
#if SAMV71_SPI_USE_QSPI == TRUE
    else if (&SPID2 == spip) {
        /* Disable clock source */
        pmc_disable_periph_clk(ID_QSPI);
        /* Disable interrupt */
        nvicDisableVector(SPI_QSPI_NVIC_NUMBER);
    }
#endif

    else {
      osalDbgAssert(false, "invalid SPI instance");
    }
  }
}

#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {
  (void)spip;
  /* CS always is automatically enabled */
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    device->SPI_CR = SPI_CR_LASTXFER;
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    device->QSPI_CR = QSPI_CR_LASTXFER;
  }
}
#endif

/**
 * @brief   Ignores data on the SPI bus.
 * @details This synchronous function performs the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     In order to use this function the option @p SPI_USE_SYNCHRONIZATION
 *          must be enabled.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_ignore(SPIDriver *spip, size_t n) {
  uint32_t active_csr = spip->config->slave_configs[spip->active_slave_config].csr;
  uint32_t dwidth = (active_csr & SPI_CSR_BITS_Msk) > SPI_CSR_BITS_8_BIT ?
                    XDMAC_CC_DWIDTH_HALFWORD : XDMAC_CC_DWIDTH_BYTE;

  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    //forfeit setting up the recv dma stuff
    device->SPI_MR &= ~SPI_MR_WDRBT;
    device->SPI_IER = SPI_IER_UNDES;
    device->SPI_IDR = SPI_IDR_OVRES;

    if (spip->config->circular) {

      size_t block1_count = (n == 1) ? 0 : n / 2;
      size_t block2_count = n - block1_count;

      spi_lld_setup_circular_constant_send(spip, block1_count, block2_count, dwidth, true);
    } else {
      spi_lld_setup_linear_constant_send(spip, n, dwidth, true);
    }

    // transfer started already
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    //forfeit setting up the recv dma stuff
    device->QSPI_IDR = QSPI_IDR_OVRES;

    if (spip->config->circular) {

      size_t block1_count = (n == 1) ? 0 : n / 2;
      size_t block2_count = n - block1_count;

      spi_lld_setup_circular_constant_send(spip, block1_count, block2_count, dwidth, true);
    } else {
      spi_lld_setup_linear_constant_send(spip, n, dwidth, true);
    }

    // transfer started already
  }

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_exchange(SPIDriver *spip, size_t n,
                       const void *txbuf, void *rxbuf) {
  uint32_t active_csr = spip->config->slave_configs[spip->active_slave_config].csr;
  uint32_t dwidth = (active_csr & SPI_CSR_BITS_Msk) > SPI_CSR_BITS_8_BIT ?
                    XDMAC_CC_DWIDTH_HALFWORD : XDMAC_CC_DWIDTH_BYTE;

  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    //read to get rid of stale data, if any.
    (void)device->SPI_RDR;
    //also clear any interrupts, like read register overrun.
    (void)device->SPI_SR;

    device->SPI_MR |= spip->config->mr & SPI_MR_WDRBT;
    device->SPI_IER = SPI_IER_OVRES | SPI_IER_UNDES;

    if (spip->config->circular) {

      size_t block1_count = (n == 1) ? 0 : n / 2;
      size_t block2_count = n - block1_count;

      //setup the recv buffer first, because it will wait for the send buffer
      spi_lld_setup_circular_buffer_recv(spip, block1_count, block2_count, rxbuf, dwidth);
      spi_lld_setup_circular_buffer_send(spip, block1_count, block2_count, txbuf, dwidth, false);
    } else {
      spi_lld_setup_linear_buffer_recv(spip, n, rxbuf, dwidth);
      spi_lld_setup_linear_buffer_send(spip, n, txbuf, dwidth, false);
    }

    // transfer started already
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    //read to get rid of stale data, if any.
    (void)device->QSPI_RDR;
    //also clear any interrupts, like read register overrun.
    (void)device->QSPI_SR;

    device->QSPI_IER = QSPI_IER_OVRES;

    if (spip->config->circular) {

      size_t block1_count = (n == 1) ? 0 : n / 2;
      size_t block2_count = n - block1_count;

      //setup the recv buffer first, because it will wait for the send buffer
      spi_lld_setup_circular_buffer_recv(spip, block1_count, block2_count, rxbuf, dwidth);
      spi_lld_setup_circular_buffer_send(spip, block1_count, block2_count, txbuf, dwidth, false);
    } else {
      spi_lld_setup_linear_buffer_recv(spip, n, rxbuf, dwidth);
      spi_lld_setup_linear_buffer_send(spip, n, txbuf, dwidth, false);
    }

    // transfer started already
  }

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

  uint32_t active_csr = spip->config->slave_configs[spip->active_slave_config].csr;
  uint32_t dwidth = (active_csr & SPI_CSR_BITS_Msk) > SPI_CSR_BITS_8_BIT ?
                    XDMAC_CC_DWIDTH_HALFWORD : XDMAC_CC_DWIDTH_BYTE;

  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    //forfeit setting up the recv dma stuff
    device->SPI_MR &= ~SPI_MR_WDRBT;
    device->SPI_IER = SPI_IER_UNDES;
    device->SPI_IDR = SPI_IDR_OVRES;

    if (spip->config->circular) {

      size_t block1_count = (n == 1) ? 0 : n / 2;
      size_t block2_count = n - block1_count;

      spi_lld_setup_circular_buffer_send(spip, block1_count, block2_count, txbuf, dwidth, true);
    } else {
      spi_lld_setup_linear_buffer_send(spip, n, txbuf, dwidth, true);
    }

    // transfer started already
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    //forfeit setting up the recv dma stuff
    device->QSPI_IDR = QSPI_IDR_OVRES;

    if (spip->config->circular) {

      size_t block1_count = (n == 1) ? 0 : n / 2;
      size_t block2_count = n - block1_count;

      spi_lld_setup_circular_buffer_send(spip, block1_count, block2_count, txbuf, dwidth, true);
    } else {
      spi_lld_setup_linear_buffer_send(spip, n, txbuf, dwidth, true);
    }

    // transfer started already
  }

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {
  uint32_t active_csr = spip->config->slave_configs[spip->active_slave_config].csr;
  uint32_t dwidth = (active_csr & SPI_CSR_BITS_Msk) > SPI_CSR_BITS_8_BIT ?
                    XDMAC_CC_DWIDTH_HALFWORD : XDMAC_CC_DWIDTH_BYTE;

  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    //read to get rid of stale data, if any.
    (void)device->SPI_RDR;
    //also clear any interrupts, like read register overrun.
    (void)device->SPI_SR;

    device->SPI_MR |= spip->config->mr & SPI_MR_WDRBT;
    device->SPI_IER = SPI_IER_OVRES | SPI_IER_UNDES;

    if (spip->config->circular) {

      size_t block1_count = (n == 1) ? 0 : n / 2;
      size_t block2_count = n - block1_count;

      //setup the recv buffer first, because it will wait for the send buffer
      spi_lld_setup_circular_buffer_recv(spip, block1_count, block2_count, rxbuf, dwidth);
      spi_lld_setup_circular_constant_send(spip, block1_count, block2_count, dwidth, false);
    } else {
      spi_lld_setup_linear_buffer_recv(spip, n, rxbuf, dwidth);
      spi_lld_setup_linear_constant_send(spip, n, dwidth, false);
    }

    // transfer started already
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    //read to get rid of stale data, if any.
    (void)device->QSPI_RDR;
    //also clear any interrupts, like read register overrun.
    (void)device->QSPI_SR;

    device->QSPI_IER = QSPI_IER_OVRES;

    if (spip->config->circular) {

      size_t block1_count = (n == 1) ? 0 : n / 2;
      size_t block2_count = n - block1_count;

      //setup the recv buffer first, because it will wait for the send buffer
      spi_lld_setup_circular_buffer_recv(spip, block1_count, block2_count, rxbuf, dwidth);
      spi_lld_setup_circular_constant_send(spip, block1_count, block2_count, dwidth, false);
    } else {
      spi_lld_setup_linear_buffer_recv(spip, n, rxbuf, dwidth);
      spi_lld_setup_linear_constant_send(spip, n, dwidth, false);
    }

    // transfer started already
  }

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Aborts the ongoing SPI operation, if any.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[out] sizep    pointer to the counter of frames not yet transferred
 *                      or @p NULL
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_stop_transfer(SPIDriver *spip, size_t *sizep) {

  spi_lld_stop_abort(spip);
  if (sizep) {
    if (spip->config->circular) {
      size_t block2_count = (spip->dma_send_descriptors[1].XDMAC_MBR_UBC & XDMAC_MBR_UBC_UBLEN_Msk) >>
                            XDMAC_MBR_UBC_UBLEN_Pos;

      //figure out which half we are in. if the next descriptor is #1,
      //we are in the first and vice versa.
      if (xdmacChannelGetNextDescriptor(spip->dma_send_channel) ==
          (samv71_xdmac_linked_list_base_t*)&(spip->dma_send_descriptors[1])) {

        *sizep = xdmacChannelGetMicroblockRemaining(spip->dma_send_channel) +
                  block2_count;
      } else {
        *sizep = xdmacChannelGetMicroblockRemaining(spip->dma_send_channel);
      }
    } else {
      *sizep = xdmacChannelGetMicroblockRemaining(spip->dma_send_channel);
    }
  }

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    //read to get rid of stale data, if any.
    (void)device->SPI_RDR;
    //also clear any interrupts, like read register overrun.
    (void)device->SPI_SR;

    device->SPI_MR |= spip->config->mr & SPI_MR_WDRBT;
    device->SPI_IDR = SPI_IDR_OVRES | SPI_IDR_UNDES;

    device->SPI_TDR = frame;

    while ((device->SPI_SR & SPI_SR_RDRF) == 0)
    {}

    return device->SPI_RDR & SPI_RDR_RD_Msk;
  }
  if (spi_lld_isQspi(spip)) {
    Qspi *device = spi_lld_getQspiDevice(spip);
    //read to get rid of stale data, if any.
    (void)device->QSPI_RDR;
    //also clear any interrupts, like read register overrun.
    (void)device->QSPI_SR;

    device->QSPI_IDR = QSPI_IDR_OVRES;

    device->QSPI_TDR = frame;

    while ((device->QSPI_SR & QSPI_SR_RDRF) == 0)
    {}

    return device->QSPI_RDR & QSPI_RDR_RD_Msk;
  }

  return 0;
}

/**
 * @brief   Sets the active slave configuration
 *
 * @param[in] spip         pointer to the @p SPIDriver object
 * @param[in] slave_config number of the slave config
 * @return                 The operation status
 */
msg_t spiSetActiveSlave(SPIDriver *spip, uint8_t active_slave) {
  if (spi_lld_isSpi(spip)) {
    Spi *device = spi_lld_getSpiDevice(spip);
    if ((spip->config->mr & SPI_MR_PCSDEC) != 0) {
      osalDbgCheck(active_slave < (1 << SPI_SUPPORTED_SLAVES) - 1);
      spip->active_slave_config = active_slave * SPI_SUPPORTED_SLAVES / (1 << SPI_SUPPORTED_SLAVES);
    } else {
      osalDbgCheck(active_slave < SPI_SUPPORTED_SLAVES);
      spip->active_slave_config = active_slave;
    }

    uint32_t mr = device->SPI_MR;
    mr &= ~SPI_MR_PCS_Msk;
    if ((spip->config->mr & SPI_MR_PCSDEC) != 0) {
      mr |= SPI_MR_PCS(active_slave);
    } else {
      mr |= SPI_MR_PCS_Msk ^ SPI_MR_PCS(1 << active_slave);
    }
    device->SPI_MR = mr;

    //The device automatically switches to the configured baudrate
  }
  /* QSPI only supports one slave */

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Calculates the current baudrate
 *
 * @param[in] spip         pointer to the @p SPIDriver object
 * @return                 baudrate(symbols per second)
 */
uint32_t spiBaudrate(SPIDriver *spip) {
  if (spi_lld_isSpi(spip)) {
    //ceil(clk/baudrate)=(clk+baudrate-1)/baudrate=
    //(clk-1)/baudrate+baudrate/baudrate=(clk-1)/baudrate+1
    int divider = ((SPI_MAIN_CLOCK - 1) /
                   spip->config->slave_configs[spip->active_slave_config].baudrate) + 1;
    if (divider < 1) {
      divider = 1;
    }
    if (divider > 255) {
      divider = 255;
    }

    uint32_t baudrate;
    baudrate = (SPI_MAIN_CLOCK - 1) / (divider - 1);

    return baudrate;
  }
  if (spi_lld_isQspi(spip)) {
    //ceil(clk/baudrate)=(clk+baudrate-1)/baudrate=
    //(clk-1)/baudrate+baudrate/baudrate=(clk-1)/baudrate+1
    int divider = ((SPI_QSPI_MAIN_CLOCK - 1) /
                   spip->config->slave_configs[0].baudrate) + 1;
    if (divider < 1) {
      divider = 1;
    }
    if (divider > 255) {
      divider = 255;
    }

    uint32_t baudrate;
    baudrate = (SPI_QSPI_MAIN_CLOCK - 1) / (divider - 1);

    return baudrate;
  }
  return -1;
}


#endif /* HAL_USE_SPI == TRUE */

/** @} */
