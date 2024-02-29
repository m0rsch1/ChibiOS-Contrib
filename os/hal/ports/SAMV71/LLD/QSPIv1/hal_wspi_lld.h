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
 *          definitions and use its own ones. In must take care to use
 *          the same name for the same function or compatibility is not
 *          ensured.
 * @note    There are the following limitations in this implementation:
 *          - Eight lines are not supported.
 *          - Only 8 bits instructions are supported.
 *          - Alt field can be 1,2,4,8 bits.
 *          - Address field can be 24 or 32 bits.
 *          - Address and Alt fields must use the same number of lanes,
 *            and if they do use multiple, that must match the number of
 *            lanes used by data
 *          - If the command field uses multiple lanes, it must match the
 *            number of lanes used by data, address and alt
 *          .
 * @{
 */
/*
 * used cfg bits:
 * 31                                       0
 * .... ....  .... ....  .... .xxx  xxxx ....
 * .... ....  .... ....  .... ....  ...x ....    QSPI_IFR_INSTEN
 * .... ....  .... ....  .... ....  ..x. ....    QSPI_IFR_ADDREN
 * .... ....  .... ....  .... ....  .x.. ....    QSPI_IFR_OPTEN
 * .... ....  .... ....  .... ....  x... ....    QSPI_IFR_DATAEN
 * .... ....  .... ....  .... .x..  .... ....    QSPI_IFR_ADDRL
 * .... ....  .... ....  .... ..xx  .... ....    QSPI_IFR_OPTL_Msk
 * .... ....  .xxx ....  .... ....  .... ....    cmd line compatibility mask
 * .... ..xx  x... ....  .... ....  .... ....    addr line compatibility mask
 * ...x xx..  .... ....  .... ....  .... ....    alt line compatibility mask
 * xxx. ....  .... ....  .... ....  .... ....    data line compatibility mask
*/

#define WSPI_LLD_CFG_CMD_MODE_COMPAT_Pos 20LU
#define WSPI_LLD_CFG_ADDR_MODE_COMPAT_Pos 23LU
#define WSPI_LLD_CFG_ALT_MODE_COMPAT_Pos 26LU
#define WSPI_LLD_CFG_DATA_MODE_COMPAT_Pos 29LU

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
/** @} */

#define QSPI_MAIN_CLK (SystemCoreClock / 2)

#define QSPI_NVIC_PRIORITY CORTEX_MIN_KERNEL_PRIORITY-1

#if defined(__SAMV71Q21B__)
#define QSPI_NVIC_NUMBER QSPI_IRQn
#define QSPI_HANDLER VectorEC
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
#define wspi_lld_driver_fields                                              \
  /* Pointer to the QSPIx registers block.*/                                \
  Qspi                          *qspi;                                      \
  const samv71_xdmac_channel_t* dma_channel


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
