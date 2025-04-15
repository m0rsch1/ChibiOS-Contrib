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
 * @file    hal_spi_lld.h
 * @brief   SAMV71 spi subsystem low level driver header.
 *
 * When using this driver with cache enabled, remember to invalidate and clean
 * the affected memory regions as needed. The XDMAC descriptors are handled
 * internally.
 *
 * DCACHE_WRITE_BACK(send_buf, size);
 * spiExchange/spiSend/SpiReceive();
 * DCACHE_INVALIDATE_FOR_READ(recv_buf, size);
 *
 * Also remember to do this in the callback for the receive side, and, when
 * using circular buffers, before reading the filled data and after filling the
 * to-be-sent data.
 *
 * On the receive buffer side, there is a risk of invalidating "live" data in
 * the same cache line; this can be avoided by aligning the beginning of the
 * receive buffer to 32 bytes and sizeing it so that it occopies a whole
 * number of 32 byte cache lines.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef HAL_SPI_LLD_H
#define HAL_SPI_LLD_H

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

#include "samv71.h"
//#include "hal_pmc_lld.h"
//#include "hal_st_lld.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define SPI_MAIN_CLOCK (SystemCoreClock / 2)
#define SPI_NVIC_PRIORITY CORTEX_MIN_KERNEL_PRIORITY-1
#define SPI_QSPI_MAIN_CLOCK (SystemCoreClock / 2)

#if defined(__SAMV71Q21B__)
#define SPI0_NVIC_NUMBER SPI0_IRQn
#define SPI0_HANDLER Vector94
//14 => 42
#define SPI1_NVIC_NUMBER SPI1_IRQn
#define SPI1_HANDLER VectorE8

#define SPI_QSPI_NVIC_NUMBER QSPI_IRQn
#define SPI_QSPI_HANDLER VectorEC
#endif
/**
 * @brief   SPI0 recv dma priority
 * @details 0 to 23, lower number is higher priority
 * @note    The default is @p 0.
 */
#if !defined(SAMV71_SPI0_RECV_DMA_PRIO) || defined(__DOXYGEN__)
#define SAMV71_SPI0_RECV_DMA_PRIO                  16
#endif
/**
 * @brief   SPI0 send dma priority
 * @details 0 to 23, lower number is higher priority
 * @note    The default is @p 16.
 */
#if !defined(SAMV71_SPI0_SEND_DMA_PRIO) || defined(__DOXYGEN__)
#define SAMV71_SPI0_SEND_DMA_PRIO                  16
#endif
/**
 * @brief   SPI1 recv dma priority
 * @details 0 to 23, lower number is higher priority
 * @note    The default is @p 16.
 */
#if !defined(SAMV71_SPI1_RECV_DMA_PRIO) || defined(__DOXYGEN__)
#define SAMV71_SPI1_RECV_DMA_PRIO                  16
#endif
/**
 * @brief   SPI1 send dma priority
 * @details 0 to 23, lower number is higher priority
 * @note    The default is @p 16.
 */
#if !defined(SAMV71_SPI1_SEND_DMA_PRIO) || defined(__DOXYGEN__)
#define SAMV71_SPI1_SEND_DMA_PRIO                  16
#endif


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

#define SPI_SUPPORTS_CIRCULAR TRUE
#define SPI_SUPPORTS_SLAVE_MODE TRUE

/**
 * @name    SAMV71 configuration options
 * @{
 */
/**
 * @brief   SPI0 driver enable switch.
 * @details If set to @p TRUE the support for SPI0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_SPI_USE_SPI0) || defined(__DOXYGEN__)
#define SAMV71_SPI_USE_SPI0             FALSE
#endif
#if !defined(SAMV71_SPI_USE_SPI1) || defined(__DOXYGEN__)
#define SAMV71_SPI_USE_SPI1             FALSE
#endif
#if !defined(SAMV71_SPI_USE_QSPI) || defined(__DOXYGEN__)
#define SAMV71_SPI_USE_QSPI             FALSE
#endif

/** @} */

#define SPI_SUPPORTED_SLAVES 4

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (SPI_NVIC_PRIORITY < ST_NVIC_PRIORITY)
#warning "Setting SPI prio higher than systick prio might cause problems"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(SPI_NVIC_PRIORITY)
#error "Invalid USART interrupt priority"
#endif

#if SAMV71_SPI_USE_SPI0 && !defined(ID_SPI0)
#error "SPI0 is not present on this device"
#endif
#if SAMV71_SPI_USE_SPI1 && !defined(ID_SPI1)
#error "SPI1 is not present on this device"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef struct hal_spi_lld_slave_config {
  /** CSR bits
   *
   * combination of
   *  * SPI_CSR_BITS_*_BIT  transfer unit size
   *  * SPI_CSR_CPOL     Inactive clock state is logic one instead of zero
   *  * SPI_CSR_NCPHA    Capture on first edge, send on 0th edge, instead
   *                     of capture on 2nd edge and send on first edge.
   *  * SPI_CSR_CSAAT    Chip select is deasserted after last transfer, instead
   *                     of keeping it selected until a different chip select
   *                     is requested.
   *  * SPI_CSR_CSNAAT   Chip select is always deasserted between data units
   *                     No effect if SPI_CSR_CSAAT is set
   *  * SPI_CSR_DLYBCT() Delay between transfers, in 32 bit times units
   *  * SPI_CSR_DLYBS()  Delay between chip select assert and clocking begin in
   *                     bit time units
   */
  uint32_t csr;
  /** Maximum baudrate to use to configure the CSR
   *
   * The actual baudrate is at most this. It will be an integer divider
   * of SPI_MAIN_CLOCK, at most SPI_MAIN_CLOCK, and at least SPI_MAIN_CLOCK/255.
   */
  uint32_t baudrate;
} SpiLLDSlaveConfig;

/**
 * @brief   SAMV71 SPI Driver configuration structure.
 *
 * mr: combination of
 *  * SPI_MR_DLYBCS()   delay between deasserting the last chip select and
 *                      asserting the next chip select
 *  * SPI_MR_LLB        local loopback
 *  * SPI_MR_WDRBT      In master mode, wait when receive register is full
 *  * SPI_MR_MODFDIS    Disable mode fault detection
 *  * SPI_MR_PCSDEC     Use decoder on the chip select outputs
 *                      If set, the first 4 slave selects use slave_config[0],
 *                      etc. 15 cannot be used as a slave number.
 */
#define spi_lld_config_fields                                              \
  uint32_t mr;                                                             \
  SpiLLDSlaveConfig slave_configs[SPI_SUPPORTED_SLAVES]

/**
 * @brief   @p SPIDriver specific data.
 */
#define spi_lld_driver_fields                                              \
  uint8_t dma_recv_hwid;                                                   \
  uint8_t dma_send_hwid;                                                   \
  uint8_t active_slave_config;                                             \
  uint8_t dma_recv_descriptors_buf[CACHE_ALIGNABLE_ALLOC_SIZE(sizeof(samv71_xdmac_linked_list_view_0_t)*2)]; \
  uint8_t dma_send_descriptors_buf[CACHE_ALIGNABLE_ALLOC_SIZE(sizeof(samv71_xdmac_linked_list_view_0_t)*2)]; \
  const samv71_xdmac_channel_t* dma_recv_channel;                          \
  const samv71_xdmac_channel_t* dma_send_channel;                          \
  samv71_xdmac_linked_list_view_0_t *dma_recv_descriptors;                 \
  samv71_xdmac_linked_list_view_0_t *dma_send_descriptors

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (SAMV71_SPI_USE_SPI0 == TRUE) && !defined(__DOXYGEN__)
extern SPIDriver SPID0;
#endif
#if (SAMV71_SPI_USE_SPI1 == TRUE) && !defined(__DOXYGEN__)
extern SPIDriver SPID1;
#endif
#if (SAMV71_SPI_USE_QSPI == TRUE) && !defined(__DOXYGEN__)
extern SPIDriver SPID2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void spi_lld_init(void);
  msg_t spi_lld_start(SPIDriver *spip);
  void spi_lld_stop(SPIDriver *spip);
#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
  void spi_lld_select(SPIDriver *spip);
  void spi_lld_unselect(SPIDriver *spip);
#endif
  msg_t spi_lld_ignore(SPIDriver *spip, size_t n);
  msg_t spi_lld_exchange(SPIDriver *spip, size_t n,
                         const void *txbuf, void *rxbuf);
  msg_t spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf);
  msg_t spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf);
  msg_t spi_lld_stop_transfer(SPIDriver *spip, size_t *sizep);
  uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame);

  msg_t spiSetActiveSlave(SPIDriver *spip, uint8_t active_slave_config);
  uint32_t spiBaudrate(SPIDriver *spip);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI == TRUE */

#endif /* HAL_SPI_LLD_H */

/** @} */
