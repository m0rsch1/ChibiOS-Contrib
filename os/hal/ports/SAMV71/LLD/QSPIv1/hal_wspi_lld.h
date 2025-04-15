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
 * @file    QSPIv1/hal_wspi_lld.h
 * @brief   SAMV71 WSPI subsystem low level driver header.
 *
 * When using this driver with cache enabled, remember to invalidate and clean
 * the affected memory regions as needed. The XDMAC descriptors are handled
 * internally.
 *
 * DCACHE_WRITE_BACK(send_buf, size);
 * wspiSend/wspiReceive();
 * DCACHE_INVALIDATE_FOR_READ(recv_buf, size);
 *
 * On the receive buffer side, there is a risk of invalidating "live" data in
 * the same cache line; this can be avoided by aligning the beginning of the
 * receive buffer to 32 bytes and sizeing it so that it occopies a whole
 * number of 32 byte cache lines.
 *
 * @addtogroup WSPI
 * @{
 */

#ifndef HAL_WSPI_LLD_H
#define HAL_WSPI_LLD_H

#if (HAL_USE_WSPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    WSPI implementation capabilities
 * @{
 */
#define WSPI_SUPPORTS_MEMMAP                TRUE
#define WSPI_DEFAULT_CFG_MASKS              FALSE
/** @} */

/**
 * @name    Transfer options
 * @note    The low level driver has the option to override the following
 *          definitions and use its own ones. It must take care to use
 *          the same name for the same function or compatibility is not
 *          ensured.
 * @note    There are the following limitations in this implementation:
 *          - Eight lines are not supported.
 *          - Only 8 bit instructions are supported.
 *          - Alt field can be 1,2,4,8 bits.
 *          - Address field can be 24 or 32 bits.
 *          - Address and Alt fields must use the same number of lanes,
 *            and if they do use multiple, that must match the number of
 *            lanes used by data
 *          - If the command field uses multiple lanes, it must match the
 *            number of lanes used by data, address and alt
 *          - Double Transfer Rate can only be enabled for data, address and
 *            alt or for data, address, alt and command
 * @{
 */
/*
 * used cfg bits:
 * 31                                       0
 * .... ....  .... ....  xx.. .xxx  xxxx ....    used QSPI_IFR register bits
 * .... ....  ...x xxxx    xx            .xxx    QSPI_IFR register bits filled
 *                                               by other means
 * xxxx x.x.  xxx. ....  .... x...  .... x       undefined QSPI_IFR bits
 *
 * .... ....  .... ....  .... ....  ...x ....    QSPI_IFR_INSTEN
 * .... ....  .... ....  .... ....  ..x. ....    QSPI_IFR_ADDREN
 * .... ....  .... ....  .... ....  .x.. ....    QSPI_IFR_OPTEN
 * .... ....  .... ....  .... ....  x... ....    QSPI_IFR_DATAEN
 * .... ....  .... ....  .... ..xx  .... ....    QSPI_IFR_OPTL_Msk
 * .... ....  .... ....  .... .x..  .... ....    QSPI_IFR_ADDRL
 * .... ....  .... ....  ...x ....  .... ....    memory or register access mode
 *                                               allows enabling scrambler
 * .... ....  .... ....  .x.. ....  .... ....    QSPI_IFR_CRM
 * .... ....  .... ....  x... ....  .... ....    QSPI_IFR_DDREN
 * .... ....  .... xxx.  .... ....  .... ....    cmd line compatibility mask
 * .... ....  .xxx ....  .... ....  .... ....    addr line compatibility mask
 * .... ..xx  x... ....  .... ....  .... ....    alt line compatibility mask
 * .... .x..  .... ....  .... ....  .... ....    QSPI_IFR_DDRCMDEN
 * xxx. ....  .... ....  .... ....  .... ....    data line compatibility mask
*/

#define WSPI_LLD_CFG_CMD_MODE_COMPAT_Pos 17LU
#define WSPI_LLD_CFG_ADDR_MODE_COMPAT_Pos 20LU
#define WSPI_LLD_CFG_ALT_MODE_COMPAT_Pos 23LU
#define WSPI_LLD_CFG_DATA_MODE_COMPAT_Pos 29LU
#define WSPI_LLD_CFG_TYPE_Pos 12LU

#define WSPI_CFG_CMD_MODE_MASK              ((7LU << WSPI_LLD_CFG_CMD_MODE_COMPAT_Pos) | QSPI_IFR_INSTEN)
#define WSPI_CFG_CMD_MODE_NONE              (7LU << WSPI_LLD_CFG_CMD_MODE_COMPAT_Pos)
#define WSPI_CFG_CMD_MODE_ONE_LINE          ((1LU << WSPI_LLD_CFG_CMD_MODE_COMPAT_Pos) | QSPI_IFR_INSTEN)
#define WSPI_CFG_CMD_MODE_TWO_LINES         ((2LU << WSPI_LLD_CFG_CMD_MODE_COMPAT_Pos) | QSPI_IFR_INSTEN)
#define WSPI_CFG_CMD_MODE_FOUR_LINES        ((4LU << WSPI_LLD_CFG_CMD_MODE_COMPAT_Pos) | QSPI_IFR_INSTEN)

#define WSPI_CFG_CMD_SIZE_MASK              0LU
#define WSPI_CFG_CMD_SIZE_8                 0LU

#define WSPI_CFG_ADDR_MODE_MASK             ((7LU << WSPI_LLD_CFG_ADDR_MODE_COMPAT_Pos) | QSPI_IFR_ADDREN)
#define WSPI_CFG_ADDR_MODE_NONE             (7LU << WSPI_LLD_CFG_ADDR_MODE_COMPAT_Pos)
#define WSPI_CFG_ADDR_MODE_ONE_LINE         ((1LU << WSPI_LLD_CFG_ADDR_MODE_COMPAT_Pos) | QSPI_IFR_ADDREN)
#define WSPI_CFG_ADDR_MODE_TWO_LINES        ((2LU << WSPI_LLD_CFG_ADDR_MODE_COMPAT_Pos) | QSPI_IFR_ADDREN)
#define WSPI_CFG_ADDR_MODE_FOUR_LINES       ((4LU << WSPI_LLD_CFG_ADDR_MODE_COMPAT_Pos) | QSPI_IFR_ADDREN)

#define WSPI_CFG_ADDR_SIZE_MASK             QSPI_IFR_ADDRL
#define WSPI_CFG_ADDR_SIZE_24               QSPI_IFR_ADDRL_24_BIT
#define WSPI_CFG_ADDR_SIZE_32               QSPI_IFR_ADDRL_32_BIT

#define WSPI_CFG_ALT_MODE_MASK              ((7LU << WSPI_LLD_CFG_ALT_MODE_COMPAT_Pos) | QSPI_IFR_OPTEN)
#define WSPI_CFG_ALT_MODE_NONE              (7LU << WSPI_LLD_CFG_ALT_MODE_COMPAT_Pos)
#define WSPI_CFG_ALT_MODE_ONE_LINE          ((1LU << WSPI_LLD_CFG_ALT_MODE_COMPAT_Pos) | QSPI_IFR_OPTEN)
#define WSPI_CFG_ALT_MODE_TWO_LINES         ((2LU << WSPI_LLD_CFG_ALT_MODE_COMPAT_Pos) | QSPI_IFR_OPTEN)
#define WSPI_CFG_ALT_MODE_FOUR_LINES        ((4LU << WSPI_LLD_CFG_ALT_MODE_COMPAT_Pos) | QSPI_IFR_OPTEN)

#define WSPI_CFG_ALT_SIZE_MASK              QSPI_IFR_OPTL_Msk
#define WSPI_CFG_ALT_SIZE_1                 QSPI_IFR_OPTL_OPTION_1BIT
#define WSPI_CFG_ALT_SIZE_2                 QSPI_IFR_OPTL_OPTION_2BIT
#define WSPI_CFG_ALT_SIZE_4                 QSPI_IFR_OPTL_OPTION_4BIT
#define WSPI_CFG_ALT_SIZE_8                 QSPI_IFR_OPTL_OPTION_8BIT

#define WSPI_CFG_DATA_MODE_MASK             ((7LU << WSPI_LLD_CFG_DATA_MODE_COMPAT_Pos) | QSPI_IFR_DATAEN)
#define WSPI_CFG_DATA_MODE_NONE             (7LU << WSPI_LLD_CFG_DATA_MODE_COMPAT_Pos)
#define WSPI_CFG_DATA_MODE_ONE_LINE         ((1LU << WSPI_LLD_CFG_DATA_MODE_COMPAT_Pos) | QSPI_IFR_DATAEN)
#define WSPI_CFG_DATA_MODE_TWO_LINES        ((2LU << WSPI_LLD_CFG_DATA_MODE_COMPAT_Pos) | QSPI_IFR_DATAEN)
#define WSPI_CFG_DATA_MODE_FOUR_LINES       ((4LU << WSPI_LLD_CFG_DATA_MODE_COMPAT_Pos) | QSPI_IFR_DATAEN)

#define WSPI_CFG_SIOO                       QSPI_IFR_CRM_ENABLED

#define WSPI_CFG_CMD_DTR                    QSPI_IFR_DDRCMDEN
#define WSPI_CFG_ADDR_DTR                   (0)
#define WSPI_CFG_ALT_DTR                    (0)
#define WSPI_CFG_DATA_DTR                   QSPI_IFR_DDREN

#define WSPI_CFG_ALL_DTR                    (WSPI_CFG_CMD_DTR   |       \
                                             WSPI_CFG_ADDR_DTR  |       \
                                             WSPI_CFG_ALT_DTR   |       \
                                             WSPI_CFG_DATA_DTR)

#define WSPI_CFG_DQS_ENABLE                 (0)

#define WSPI_LLD_CFG_TYPE_MASK              (1LU << WSPI_LLD_CFG_TYPE_Pos)
#define WSPI_CFG_TYPE_REGISTER              (0LU << WSPI_LLD_CFG_TYPE_Pos)
#define WSPI_CFG_TYPE_MEMORY                (1LU << WSPI_LLD_CFG_TYPE_Pos)
/** @} */

#define WSPI_QSPI_MAIN_CLK (SystemCoreClock / 2)

#define WSPI_QSPI_NVIC_PRIORITY CORTEX_MIN_KERNEL_PRIORITY-1

#if defined(__SAMV71Q21B__)
#define WSPI_QSPI_NVIC_NUMBER QSPI_IRQn
#define WSPI_QSPI_HANDLER VectorEC
#endif


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   WSPID1 driver enable switch.
 * @details If set to @p TRUE the support for QSPI is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SAMV71_WSPI_USE_QSPI) || defined(__DOXYGEN__)
#define SAMV71_WSPI_USE_QSPI             FALSE
#endif
/**
 * @brief   Whether to use DMA to access memory
 * @details When @p TRUE, the DMA is used to read/write the mapped memory.
 *          This may stall the DMA engine while an access is going on.
 *          When @p FALSE, the MCU is used to read/write the mapped memory.
 * @note    The default is @p TRUE
 */
#if !defined(SAMV71_QSPI_USE_DMA) || defined(__DOXYGEN__)
#define SAMV71_QSPI_USE_DMA              TRUE
#endif
/**
 * @brief   QSPI dma priority
 * @details 0 to 23, lower number is higher priority
 * @note    The default is @p 16.
 */
#if !defined(SAMV71_QSPI_DMA_PRIO) || defined(__DOXYGEN__)
#define SAMV71_QSPI_DMA_PRIO                  16
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if SAMV71_WSPI_USE_QSPI && !defined(ID_QSPI)
#error "QSPI not present in the selected device"
#endif

#if !SAMV71_WSPI_USE_QSPI
#error "WSPI driver activated but no QUADSPI peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the WSPI configuration structure.
 */
#define wspi_lld_config_fields                                              \
  uint32_t speed;                                                           \
  uint32_t mr;                                                              \
  uint32_t scr

/**
 * @brief   Low level fields of the WSPI driver structure.
 */
#if SAMV71_QSPI_USE_DMA
#define wspi_lld_driver_fields                                              \
  /* Pointer to the QSPIx registers block.*/                                \
  Qspi                          *qspi;                                      \
  const samv71_xdmac_channel_t* dma_channel
#else
#define wspi_lld_driver_fields                                              \
  /* Pointer to the QSPIx registers block.*/                                \
  Qspi                          *qspi;
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (SAMV71_WSPI_USE_QSPI == TRUE) && !defined(__DOXYGEN__)
extern WSPIDriver WSPID1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void wspi_lld_init(void);
  void wspi_lld_start(WSPIDriver *wspip);
  void wspi_lld_stop(WSPIDriver *wspip);
  void wspi_lld_command(WSPIDriver *wspip, const wspi_command_t *cmdp);
  void wspi_lld_send(WSPIDriver *wspip, const wspi_command_t *cmdp,
                     size_t n, const uint8_t *txbuf);
  void wspi_lld_receive(WSPIDriver *wspip, const wspi_command_t *cmdp,
                        size_t n, uint8_t *rxbuf);
#if WSPI_SUPPORTS_MEMMAP == TRUE
  void wspi_lld_map_flash(WSPIDriver *wspip,
                          const wspi_command_t *cmdp,
                          uint8_t **addrp);
  void wspi_lld_unmap_flash(WSPIDriver *wspip);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_WSPI */

#endif /* HAL_WSPI_LLD_H */

/** @} */
