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
 * @file    XDMACv1/samv71_dma.c
 * @brief   Enhanced DMA helper driver code.
 *
 * @addtogroup SAMV71_XDMAC
 * @details DMA sharing helper driver. In the STM32 the DMA streams are a
 *          shared resource, this driver allows to allocate and free DMA
 *          streams at runtime in order to allow all the other device
 *          drivers to coordinate the access to the resource.
 * @note    The DMA ISR handlers are all declared into this module because
 *          sharing, the various device drivers can associate a callback to
 *          ISRs when allocating streams.
 * @{
 */

#include "hal.h"
#include "samv71_xdmac.h"

#define SAMV71_XDMAC_REQUIRED

/* The following macro is only defined if some driver requiring DMA services
   has been enabled.*/
#if defined(SAMV71_XDMAC_REQUIRED) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/


/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#define SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(n) { &XDMAC->XDMAC_CHID[n], n }

const samv71_xdmac_channel_t _samv71_xdmac_channels[SAMV71_XDMAC_CHANNELS] = {
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(0),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(1),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(2),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(3),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(4),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(5),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(6),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(7),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(8),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(9),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(10),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(11),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(12),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(13),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(14),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(15),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(16),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(17),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(18),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(19),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(20),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(21),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(22),
    SAMV71_XDMAC_CHANNELS_CHANNEL_INIT(23),
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Global DMA-related data structures.
 */
static struct {
  /**
   * @brief   Mask of the allocated channels.
   */
  uint32_t          allocated_mask;
  /**
   * @brief   DMA IRQ redirectors.
   */
  struct {
    /**
     * @brief   DMA callback function.
     */
    samv71_xdmacisr_t    func;
    /**
     * @brief   DMA callback parameter.
     */
    void              *param;
  } channels[SAMV71_XDMAC_CHANNELS];
} xdmac;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   DMA1 stream 0 shared interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(XDMAC_HANDLER) {
  OSAL_IRQ_PROLOGUE();

  uint32_t gis = XDMAC->XDMAC_GIS;
  uint32_t gim = XDMAC->XDMAC_GIM;
  gis &= gim;
  XDMAC->XDMAC_GID = gis & ~xdmac.allocated_mask;
  gis &= xdmac.allocated_mask;
  while (gis != 0) {
    unsigned int i = 31 - __CLZ(gis);
    gis &= ~(1U << i);

    if (xdmac.channels[i].func) {
      uint32_t cis = XDMAC->XDMAC_CHID[i].XDMAC_CIS;
      uint32_t cim = XDMAC->XDMAC_CHID[i].XDMAC_CIM;
      cis &= cim;

      xdmac.channels[i].func(xdmac.channels[i].param, cis);
    }
  }

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   STM32 DMA helper initialization.
 *
 * @init
 */
void xdmacInit(void) {
  unsigned i;

  xdmac.allocated_mask = 0U;
  for (i = 0U; i < SAMV71_XDMAC_CHANNELS; i++) {
    _samv71_xdmac_channels[i].channel->XDMAC_CC = 0;
    xdmac.channels[i].func = NULL;
  }
}

/**
 * @brief   Allocates a DMA stream.
 * @details The stream is allocated and, if required, the DMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p SAMV71_XDMAC_STREAM_ID_ANY for any stream.
 *                      - @p SAMV71_XDMAC_STREAM_ID_ANY_DMA1 for any stream
 *                        on DMA1.
 *                      - @p SAMV71_XDMAC_STREAM_ID_ANY_DMA2 for any stream
 *                        on DMA2.
 *                      .
 * @param[in] priority  IRQ priority for the DMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @param[in] prio      priority, 0..23
 * @return              Pointer to the allocated @p samv71_xdmac_channel_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @iclass
 */
const samv71_xdmac_channel_t *xdmacChannelAllocI(samv71_xdmacisr_t func,
                                          void *param, samv71_xdmacprio_t prio) {
  int i, startid, endid;
  uint32_t mask;

  osalDbgCheckClassI();

  /* it seems like the channel arbitration priority depends on the
   * channel index, higher is lower priority */
  startid = prio;
  endid   = SAMV71_XDMAC_CHANNELS - 1U;

  const samv71_xdmac_channel_t *xdmacchp = NULL;

  //at first, scan forward, towards worse priorities
  for (i = startid; i <= endid; i++) {
    mask = (1U << i);
    if ((xdmac.allocated_mask & mask) == 0U) {
      xdmacchp = SAMV71_XDMAC_CHANNEL(i);
      break;
    }
  }
  if(!xdmacchp && prio > 0) {
    //then try backwards.
    startid = prio-1;
    endid   = 0;
    for (i = startid; i >= endid; i--) {
      mask = (1U << i);
      if ((xdmac.allocated_mask & mask) == 0U) {
        xdmacchp = SAMV71_XDMAC_CHANNEL(i);
        break;
      }
    }
  }

  if(xdmacchp) {
    if (xdmac.allocated_mask == 0U) {
      pmc_enable_periph_clk(ID_XDMAC);

      XDMAC->XDMAC_GID = 0xFFFFFFFFU;
      XDMAC->XDMAC_GD = 0xFFFFFFFFU;
      XDMAC->XDMAC_GWAC = XDMAC_GWAC_PW0(15) | XDMAC_GWAC_PW1(10) | XDMAC_GWAC_PW2(5) | XDMAC_GWAC_PW3(0);

      nvicEnableVector(XDMAC_NVIC_NUMBER, XDMAC_NVIC_PRIORITY);
    }

    osalDbgCheck((xdmac.allocated_mask & mask) == 0);
    xdmac.allocated_mask  |= mask;

    /* Installs the DMA handler.*/
    xdmac.channels[i].func  = func;
    xdmac.channels[i].param = param;

    /* Putting the stream in a safe state.*/
    xdmacChannelDisable(xdmacchp);
    xdmacchp->channel->XDMAC_CC = 0;
  }

  return xdmacchp;
}

/**
 * @brief   Allocates a DMA stream.
 * @details The stream is allocated and, if required, the DMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p SAMV71_XDMAC_STREAM_ID_ANY for any stream.
 *                      - @p SAMV71_XDMAC_STREAM_ID_ANY_DMA1 for any stream
 *                        on DMA1.
 *                      - @p SAMV71_XDMAC_STREAM_ID_ANY_DMA2 for any stream
 *                        on DMA2.
 *                      .
 * @param[in] priority  IRQ priority for the DMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p samv71_xdmac_channel_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @api
 */
const samv71_xdmac_channel_t *xdmacChannelAlloc(samv71_xdmacisr_t func,
                                         void *param, samv71_xdmacprio_t prio) {
  const samv71_xdmac_channel_t *xdmacchp;

  osalSysLock();
  xdmacchp = xdmacChannelAllocI(func, param, prio);
  osalSysUnlock();

  return xdmacchp;
}

/**
 * @brief   Releases a DMA stream.
 * @details The stream is freed and, if required, the DMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmastp    pointer to a samv71_xdmac_channel_t structure
 *
 * @iclass
 */
void xdmacChannelFreeI(const samv71_xdmac_channel_t *xdmacchp) {

  osalDbgCheck(xdmacchp != NULL);

  /* Check if the streams is not taken.*/
  osalDbgAssert((xdmac.allocated_mask & (1U << xdmacchp->selfindex)) != 0U,
                "not allocated");

  /* Marks the stream as not allocated.*/
  xdmac.allocated_mask &= ~(1U << xdmacchp->selfindex);
  XDMAC->XDMAC_GID = XDMAC_GID_ID0 << xdmacchp->selfindex;

  if ( xdmac.allocated_mask == 0U ) {

    XDMAC->XDMAC_GID = 0xFFFFFFFFU;
    XDMAC->XDMAC_GD = 0xFFFFFFFFU;

    nvicDisableVector( XDMAC_NVIC_NUMBER );
    pmc_disable_periph_clk ( ID_XDMAC );
  }
}

void xdmacDisableAll(void) {
 uint32_t i, startid, endid;

  startid = 0U;
  endid   = SAMV71_XDMAC_CHANNELS - 1U;

  for (i = startid; i <= endid; i++) {
    uint32_t mask = (1U << i);
    if ((xdmac.allocated_mask & mask) != 0U) {
      const samv71_xdmac_channel_t *xdmacchp = SAMV71_XDMAC_CHANNEL(i);

      xdmacChannelDisable(xdmacchp);
    }
  }
}

#endif /* SAMV71_XDMAC_REQUIRED */

/** @} */
