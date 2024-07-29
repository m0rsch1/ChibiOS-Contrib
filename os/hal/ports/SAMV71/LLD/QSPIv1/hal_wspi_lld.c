
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
 * @file    QSPIv1/hal_wspi_lld.c
 * @brief   SAMV71 WSPI subsystem low level driver source.
 *
 * @addtogroup WSPI
 * @{
 */

#include "hal.h"

#if (HAL_USE_WSPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* @brief MDMA HW request is QSPI FIFO threshold Flag */
#define MDMA_REQUEST_QUADSPI_FIFO_TH      ((uint32_t)0x00000016U)

/* @brief MDMA HW request is QSPI Transfer complete Flag */
#define MDMA_REQUEST_QUADSPI_TC           ((uint32_t)0x00000017U)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief QSPI driver identifier.*/
#if SAMV71_WSPI_USE_QSPI || defined(__DOXYGEN__)
WSPIDriver WSPID1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void SCB_InvalidateDCache_by_Addr_Unaligned(uint32_t *x, size_t n) {
  size_t mask = ~(CACHE_LINE_SIZE-1);
  uintptr_t begin = ((uintptr_t)x) & mask;
  uintptr_t end = (((uintptr_t)x) + n + CACHE_LINE_SIZE-1) & mask;
  SCB_InvalidateDCache_by_Addr((uint32_t*)begin, end - begin);
}

static void SCB_CleanDCache_by_Addr_Unaligned(uint32_t *x, size_t n) {
  size_t mask = ~(CACHE_LINE_SIZE-1);
  uintptr_t begin = ((uintptr_t)x) & mask;
  uintptr_t end = (((uintptr_t)x) + n + CACHE_LINE_SIZE-1) & mask;
  SCB_CleanDCache_by_Addr((uint32_t*)begin, end - begin);
}

/**
 * @brief   Shared service routine.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 */
static void wspi_lld_serve_interrupt(WSPIDriver *wspip) {
  uint32_t sr;

  sr = wspip->qspi->QSPI_SR;

  if(sr & QSPI_SR_INSTRE) {
    _wspi_isr_code(wspip);
  }

  /* TODO errors handling.*/
}

/**
 * @brief   Shared service routine.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] flags     content of the CISR register
 */
static void wspi_lld_dma_func(void *param, uint32_t flags) {
  WSPIDriver *wspip = (WSPIDriver *)param;

  (void)flags;

  if(wspip->state == WSPI_SEND || wspip->state == WSPI_RECEIVE) {
    wspip->qspi->QSPI_CR = QSPI_CR_LASTXFER;
    if(wspip->rxbuf)
      SCB_InvalidateDCache_by_Addr_Unaligned(wspip->rxbuf, wspip->size);
    _wspi_isr_code(wspip);
  }

#if 0
  if (((flags & STM32_MDMA_CISR_CTCIF) != 0U) &&
      (wspip->state == WSPI_RECEIVE)) {
    /* Portable WSPI ISR code defined in the high level driver, note, it is
     a macro.*/
    _wspi_isr_code(wspip);

    mdmaChannelDisableX(wspip->mdma);
  }
  /* DMA errors handling.*/
#if defined(STM32_WSPI_MDMA_ERROR_HOOK)
  else if ((flags & STM32_MDMA_CISR_TEIF) != 0) {
    STM32_WSPI_MDMA_ERROR_HOOK(wspip);
  }
#endif
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SAMV71_WSPI_USE_QSPI
/**
 * @brief   QSPI IRQ Handler
 *
 * @isr
 */
OSAL_IRQ_HANDLER(QSPI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  wspi_lld_serve_interrupt(&WSPID1);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level WSPI driver initialization.
 *
 * @notapi
 */
void wspi_lld_init(void) {

#if SAMV71_WSPI_USE_QSPI
  wspiObjectInit(&WSPID1);
  WSPID1.qspi       = QSPI;
#endif
}

/**
 * @brief   Configures and activates the WSPI peripheral.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @notapi
 */
void wspi_lld_start(WSPIDriver *wspip) {

  /* If in stopped state then full initialization.*/
  if (wspip->state == WSPI_STOP) {
#if SAMV71_WSPI_USE_QSPI
    if (&WSPID1 == wspip) {
        // First enable the clock of the timer
        pmc_enable_periph_clk(ID_QSPI);
        // Enable the NVIC
        nvicEnableVector(QSPI_NVIC_NUMBER, QSPI_NVIC_PRIORITY);
    }
#endif
  }

  /* QSPI setup and enable.*/
  wspip->qspi->QSPI_MR = QSPI_MR_SMM_MEMORY |
                         QSPI_MR_CSMODE_LASTXFER | // forced by hardware
                         wspip->config->mr;
  uint32_t scbr = (QSPI_MAIN_CLK + wspip->config->speed)/wspip->config->speed;
  wspip->qspi->QSPI_SCR = wspip->config->scr |
                          QSPI_SCR_SCBR(scbr);

  if(!wspip->dma_channel)
    wspip->dma_channel = xdmacChannelAllocI(wspi_lld_dma_func, wspip);

  wspip->qspi->QSPI_CR = QSPI_CR_QSPIEN;
}

/**
 * @brief   Deactivates the WSPI peripheral.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @notapi
 */
void wspi_lld_stop(WSPIDriver *wspip) {

  /* If in ready state then disables the QUADSPI clock.*/
  if (wspip->state == WSPI_READY) {

    /* WSPI disable.*/
    wspip->qspi->QSPI_CR = QSPI_CR_QSPIDIS;

    if(wspip->dma_channel) {
      xdmacChannelFreeI(wspip->dma_channel);
      wspip->dma_channel = NULL;
    }

    /* Stopping involved clocks.*/
#if SAMV71_WSPI_USE_QSPI
    if (&WSPID1 == wspip) {
      nvicDisableVector(QSPI_NVIC_NUMBER);
      pmc_disable_periph_clk(ID_QSPI);
    }
#endif
  }
}

#define COMPAT_BITS(cmd, addr, alt, data) \
  (((cmd) << WSPI_LLD_CFG_CMD_MODE_COMPAT_Pos) | \
  ((addr) << WSPI_LLD_CFG_ADDR_MODE_COMPAT_Pos) | \
  ((alt) << WSPI_LLD_CFG_ALT_MODE_COMPAT_Pos) | \
  ((data) << WSPI_LLD_CFG_DATA_MODE_COMPAT_Pos))

#define ALL_BITS_SET(cfg, bits) ((~(cfg) & (bits)) == 0)

static uint32_t wspi_lld_setup_ifr(uint32_t ifr, uint32_t dummy, uint32_t cfg) {
  ifr &= ~(QSPI_IFR_NBDUM_Msk | QSPI_IFR_WIDTH_Msk | QSPI_IFR_INSTEN |
           QSPI_IFR_ADDREN | QSPI_IFR_OPTEN | QSPI_IFR_DATAEN |
           QSPI_IFR_TFRTYP_Msk | QSPI_IFR_OPTL_Msk | QSPI_IFR_ADDRL);

  ifr |= QSPI_IFR_NBDUM(dummy);

  ifr |= cfg & (WSPI_CFG_ADDR_SIZE_MASK | WSPI_CFG_ALT_SIZE_MASK |
    QSPI_IFR_INSTEN | QSPI_IFR_ADDREN | QSPI_IFR_OPTEN | QSPI_IFR_DATAEN);

  //each of cmd, addr, alt, data mode has either a single bit set in cfg for
  //the compatible line count, or all 3 bits if they are all "compatible",
  //because that component is not used.

  if (ALL_BITS_SET(cfg, COMPAT_BITS(1,1,1,1))) {
    ifr |= QSPI_IFR_WIDTH_SINGLE_BIT_SPI;
  } else if (ALL_BITS_SET(cfg, COMPAT_BITS(1,1,1,2))) {
    ifr |= QSPI_IFR_WIDTH_DUAL_OUTPUT;
  } else if (ALL_BITS_SET(cfg, COMPAT_BITS(1,1,1,4))) {
    ifr |= QSPI_IFR_WIDTH_QUAD_OUTPUT;
  } else if (ALL_BITS_SET(cfg, COMPAT_BITS(1,2,2,2))) {
    ifr |= QSPI_IFR_WIDTH_DUAL_IO;
  } else if (ALL_BITS_SET(cfg, COMPAT_BITS(1,4,4,4))) {
    ifr |= QSPI_IFR_WIDTH_QUAD_IO;
  } else if (ALL_BITS_SET(cfg, COMPAT_BITS(2,2,2,2))) {
    ifr |= QSPI_IFR_WIDTH_DUAL_CMD;
  } else if (ALL_BITS_SET(cfg, COMPAT_BITS(4,4,4,4))) {
    ifr |= QSPI_IFR_WIDTH_QUAD_CMD;
  } else {
    osalDbgCheck(false);
  }

  return ifr;
}

/**
 * @brief   Sends a command without data phase.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 *
 * @notapi
 */
void wspi_lld_command(WSPIDriver *wspip, const wspi_command_t *cmdp) {
  osalDbgCheck((cmdp->cfg & WSPI_CFG_DATA_MODE_MASK) == WSPI_CFG_DATA_MODE_NONE);

  //if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE &&
  //    (cmdp->cfg & WSPI_CFG_DATA_MODE_MASK) == WSPI_CFG_DATA_MODE_NONE)
  wspip->qspi->QSPI_IAR = cmdp->addr;

  //if ((cmdp->cfg & WSPI_CFG_CMD_MODE_MASK) != WSPI_CFG_CMD_MODE_NONE ||
  //    (cmdp->cfg & WSPI_CFG_ALT_MODE_MASK) != WSPI_CFG_ALT_MODE_NONE)
  wspip->qspi->QSPI_ICR = QSPI_ICR_INST(cmdp->cmd) | QSPI_ICR_OPT(cmdp->alt);


  uint32_t ifr = wspip->qspi->QSPI_IFR;

  ifr = wspi_lld_setup_ifr(ifr, cmdp->dummy, cmdp->cfg);

  wspip->qspi->QSPI_IFR = ifr;

  wspip->qspi->QSPI_IER = QSPI_IDR_INSTRE;
}

/**
 * @brief   Sends a command with data over the WSPI bus.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 * @param[in] n         number of bytes to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void wspi_lld_send(WSPIDriver *wspip, const wspi_command_t *cmdp,
                   size_t n, const uint8_t *txbuf) {
  //if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE &&
  //    (cmdp->cfg & WSPI_CFG_DATA_MODE_MASK) == WSPI_CFG_DATA_MODE_NONE)
  wspip->qspi->QSPI_IAR = cmdp->addr;

  //if ((cmdp->cfg & WSPI_CFG_CMD_MODE_MASK) != WSPI_CFG_CMD_MODE_NONE ||
  //    (cmdp->cfg & WSPI_CFG_ALT_MODE_MASK) != WSPI_CFG_ALT_MODE_NONE)
  wspip->qspi->QSPI_ICR = QSPI_ICR_INST(cmdp->cmd) | QSPI_ICR_OPT(cmdp->alt);

  uint32_t ifr = wspip->qspi->QSPI_IFR;

  ifr = wspi_lld_setup_ifr(ifr, cmdp->dummy, cmdp->cfg);

  ifr |= QSPI_IFR_TFRTYP_TRSFR_WRITE;

  wspip->qspi->QSPI_IFR = ifr;

  //dummy read to "synchronize APB and AHB accesses"
  ifr = wspip->qspi->QSPI_IFR;
  uint32_t addr = cmdp->addr;
  if((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) == WSPI_CFG_ADDR_MODE_NONE) {
    //Just read/write anywhere in the QSPI address space.
    addr = QSPIMEM_ADDR;
  } else {
    //write to memory, sequential only
    if (addr < QSPIMEM_ADDR || addr >= QSPIMEM_ADDR + 0x20000000U) {
      addr = (addr & 0x1fffffff) + QSPIMEM_ADDR;
    }
  }

  wspip->qspi->QSPI_IER = QSPI_IDR_INSTRE;

  SCB_CleanDCache_by_Addr_Unaligned((uint32_t*)txbuf, n);
  wspip->rxbuf = NULL;
  wspip->size = n;

  xdmacChannelSetInterruptCauses(wspip->dma_channel,
                                 XDMAC_CIE_LIE | XDMAC_CIE_RBIE |
                                 XDMAC_CIE_WBIE | XDMAC_CIE_ROIE);
  xdmacChannelStartSingleMicroblock(wspip->dma_channel,
                                    XDMAC_CC_TYPE_MEM_TRAN |
                                    XDMAC_CC_MBSIZE_SIXTEEN |
                                    XDMAC_CC_SWREQ_SWR_CONNECTED |
                                    XDMAC_CC_SAM_INCREMENTED_AM |
                                    XDMAC_CC_DAM_INCREMENTED_AM |
                                    XDMAC_CC_DSYNC_MEM2PER |
                                    XDMAC_CC_CSIZE_CHK_1 |
                                    XDMAC_CC_DWIDTH_BYTE |
                                    xdmacAutomaticInterfaceBits_CCReg(
                                      txbuf, addr),
                                    addr,
                                    txbuf,
                                    n);
  xdmacChannelSoftwareRequest(wspip->dma_channel);
}

/**
 * @brief   Sends a command then receives data over the WSPI bus.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 * @param[in] n         number of bytes to send
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void wspi_lld_receive(WSPIDriver *wspip, const wspi_command_t *cmdp,
                      size_t n, uint8_t *rxbuf) {
  //if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE &&
  //    (cmdp->cfg & WSPI_CFG_DATA_MODE_MASK) == WSPI_CFG_DATA_MODE_NONE)
  wspip->qspi->QSPI_IAR = cmdp->addr;

  //if ((cmdp->cfg & WSPI_CFG_CMD_MODE_MASK) != WSPI_CFG_CMD_MODE_NONE ||
  //    (cmdp->cfg & WSPI_CFG_ALT_MODE_MASK) != WSPI_CFG_ALT_MODE_NONE)
  wspip->qspi->QSPI_ICR = QSPI_ICR_INST(cmdp->cmd) | QSPI_ICR_OPT(cmdp->alt);


  uint32_t ifr = wspip->qspi->QSPI_IFR;

  ifr = wspi_lld_setup_ifr(ifr, cmdp->dummy, cmdp->cfg);

  ifr |= QSPI_IFR_TFRTYP_TRSFR_READ;

  wspip->qspi->QSPI_IFR = ifr;

  //dummy read to "synchronize APB and AHB accesses"
  ifr = wspip->qspi->QSPI_IFR;
  uint32_t addr = cmdp->addr;
  if((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) == WSPI_CFG_ADDR_MODE_NONE) {
    //Just read/write anywhere in the QSPI address space.
    addr = QSPIMEM_ADDR;
  } else {
    //write to memory, sequential only
    if (addr < QSPIMEM_ADDR || addr >= QSPIMEM_ADDR + 0x20000000U) {
      addr = (addr & 0x1fffffff) + QSPIMEM_ADDR;
    }
  }

  wspip->qspi->QSPI_IER = QSPI_IDR_INSTRE;

  wspip->rxbuf = rxbuf;
  wspip->size = n;

  xdmacChannelSetInterruptCauses(wspip->dma_channel,
                                 XDMAC_CIE_LIE | XDMAC_CIE_RBIE |
                                 XDMAC_CIE_WBIE | XDMAC_CIE_ROIE);
  xdmacChannelStartSingleMicroblock(wspip->dma_channel,
                                    XDMAC_CC_TYPE_MEM_TRAN |
                                    XDMAC_CC_MBSIZE_SIXTEEN |
                                    XDMAC_CC_SWREQ_SWR_CONNECTED |
                                    XDMAC_CC_SAM_INCREMENTED_AM |
                                    XDMAC_CC_DAM_INCREMENTED_AM |
                                    XDMAC_CC_DSYNC_PER2MEM |
                                    XDMAC_CC_CSIZE_CHK_1 |
                                    XDMAC_CC_DWIDTH_BYTE |
                                    xdmacAutomaticInterfaceBits_CCReg(
                                      addr, rxbuf),
                                    rxbuf,
                                    addr,
                                    n);
  xdmacChannelSoftwareRequest(wspip->dma_channel);
}

#if (WSPI_SUPPORTS_MEMMAP == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Maps in memory space a WSPI flash device.
 * @pre     The memory flash device must be initialized appropriately
 *          before mapping it in memory space.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 * @param[out] addrp    pointer to the memory start address of the mapped
 *                      flash or @p NULL
 *
 * @notapi
 */
void wspi_lld_map_flash(WSPIDriver *wspip,
                        const wspi_command_t *cmdp,
                        uint8_t **addrp) {
  //if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE &&
  //    (cmdp->cfg & WSPI_CFG_DATA_MODE_MASK) == WSPI_CFG_DATA_MODE_NONE)
  wspip->qspi->QSPI_IAR = cmdp->addr;

  //if ((cmdp->cfg & WSPI_CFG_CMD_MODE_MASK) != WSPI_CFG_CMD_MODE_NONE ||
  //    (cmdp->cfg & WSPI_CFG_ALT_MODE_MASK) != WSPI_CFG_ALT_MODE_NONE)
  wspip->qspi->QSPI_ICR = QSPI_ICR_INST(cmdp->cmd) | QSPI_ICR_OPT(cmdp->alt);

  uint32_t ifr = wspip->qspi->QSPI_IFR;

  ifr = wspi_lld_setup_ifr(ifr, cmdp->dummy, cmdp->cfg);

  ifr |= QSPI_IFR_TFRTYP_TRSFR_READ_MEMORY;

  wspip->qspi->QSPI_IFR = ifr;

  wspip->qspi->QSPI_IDR = QSPI_IDR_INSTRE;

  /* Mapped flash absolute base address.*/
  if (addrp != NULL) {
    *addrp = (uint8_t *)QSPIMEM_ADDR;
  }
}

/**
 * @brief   Unmaps from memory space a WSPI flash device.
 * @post    The memory flash device must be re-initialized for normal
 *          commands exchange.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @notapi
 */
void wspi_lld_unmap_flash(WSPIDriver *wspip) {
  //read to clear INTRE, CSE
  uint32_t sr = wspip->qspi->QSPI_SR;
  (void)sr;
  wspip->qspi->QSPI_CR = QSPI_CR_LASTXFER;
}
#endif /* WSPI_SUPPORTS_MEMMAP == TRUE */

#endif /* HAL_USE_WSPI */

/** @} */
