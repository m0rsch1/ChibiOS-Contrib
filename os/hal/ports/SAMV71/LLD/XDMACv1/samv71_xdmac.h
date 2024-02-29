
/*
    ChibiOS - Copyright (C) 2006..2019 Giovanni Di Sirio

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
 * @file    XDMACv1/samv71_xdmac.h
 * @brief   Enhanced-DMA helper driver header.
 *
 * @addtogroup SAMV71_XDMAC
 * @{
 */

#ifndef SAMV71_XDMAC_H
#define SAMV71_XDMAC_H

#include "hal_matrix_lld.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Total number of DMA streams.
 * @details This is the total number of streams among all the DMA units.
 */
#define SAMV71_XDMAC_CHANNELS           XDMACCHID_NUMBER

#define XDMAC_NVIC_PRIORITY CORTEX_MIN_KERNEL_PRIORITY-1

#if defined(__SAMV71Q21B__)
#define XDMAC_NVIC_NUMBER XDMAC_IRQn
#define XDMAC_HANDLER Vector128
#endif

#define SAMV71_XDMAC_HWREQ_HSMCI         0
#define SAMV71_XDMAC_HWREQ_SPI0_XMIT     1
#define SAMV71_XDMAC_HWREQ_SPI0_RECV     2
#define SAMV71_XDMAC_HWREQ_SPI1_XMIT     3
#define SAMV71_XDMAC_HWREQ_SPI1_RECV     4
#define SAMV71_XDMAC_HWREQ_QSPI_XMIT     5
#define SAMV71_XDMAC_HWREQ_QSPI_RECV     6
#define SAMV71_XDMAC_HWREQ_USART0_XMIT   7
#define SAMV71_XDMAC_HWREQ_USART0_RECV   8
#define SAMV71_XDMAC_HWREQ_USART1_XMIT   9
#define SAMV71_XDMAC_HWREQ_USART1_RECV  10
#define SAMV71_XDMAC_HWREQ_USART2_XMIT  11
#define SAMV71_XDMAC_HWREQ_USART2_RECV  12
#define SAMV71_XDMAC_HWREQ_PWM0         13
#define SAMV71_XDMAC_HWREQ_TWIHS0_XMIT  14
#define SAMV71_XDMAC_HWREQ_TWIHS0_RECV  15
#define SAMV71_XDMAC_HWREQ_TWIHS1_XMIT  16
#define SAMV71_XDMAC_HWREQ_TWIHS1_RECV  17
#define SAMV71_XDMAC_HWREQ_TWIHS2_XMIT  18
#define SAMV71_XDMAC_HWREQ_TWIHS2_RECV  19
#define SAMV71_XDMAC_HWREQ_UART0_XMIT   20
#define SAMV71_XDMAC_HWREQ_UART0_RECV   21
#define SAMV71_XDMAC_HWREQ_UART1_XMIT   22
#define SAMV71_XDMAC_HWREQ_UART1_RECV   23
#define SAMV71_XDMAC_HWREQ_UART2_XMIT   24
#define SAMV71_XDMAC_HWREQ_UART2_RECV   25
#define SAMV71_XDMAC_HWREQ_UART3_XMIT   26
#define SAMV71_XDMAC_HWREQ_UART3_RECV   27
#define SAMV71_XDMAC_HWREQ_UART4_XMIT   28
#define SAMV71_XDMAC_HWREQ_UART4_RECV   29
#define SAMV71_XDMAC_HWREQ_DACC_CH0     30
#define SAMV71_XDMAC_HWREQ_DACC_CH1     31
#define SAMV71_XDMAC_HWREQ_SSC_XMIT     32
#define SAMV71_XDMAC_HWREQ_SSC_RECV     33
#define SAMV71_XDMAC_HWREQ_PIOA         34
#define SAMV71_XDMAC_HWREQ_AFEC0        35
#define SAMV71_XDMAC_HWREQ_AFEC1        36
#define SAMV71_XDMAC_HWREQ_AES_XMIT     37
#define SAMV71_XDMAC_HWREQ_AES_RECV     38
#define SAMV71_XDMAC_HWREQ_PWM1         39
#define SAMV71_XDMAC_HWREQ_TC0          40
#define SAMV71_XDMAC_HWREQ_TC3          41
#define SAMV71_XDMAC_HWREQ_TC6          42
#define SAMV71_XDMAC_HWREQ_TC9          43
#define SAMV71_XDMAC_HWREQ_I2SC0_L_XMIT 44
#define SAMV71_XDMAC_HWREQ_I2SC0_L_RECV 45
#define SAMV71_XDMAC_HWREQ_I2SC1_L_XMIT 46
#define SAMV71_XDMAC_HWREQ_I2SC1_L_RECV 47
#define SAMV71_XDMAC_HWREQ_I2SC0_R_XMIT 48
#define SAMV71_XDMAC_HWREQ_I2SC0_R_RECV 49
#define SAMV71_XDMAC_HWREQ_I2SC1_R_XMIT 50
#define SAMV71_XDMAC_HWREQ_I2SC1_R_RECV 51

/**
 * @brief   Returns a pointer to a stm32_dma_stream_t structure.
 *
 * @param[in] id        the stream numeric identifier
 * @return              A pointer to the stm32_dma_stream_t constant structure
 *                      associated to the DMA stream.
 */
#define SAMV71_XDMAC_CHANNEL(id)        (&_samv71_xdmac_channels[id])

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   STM32 DMA ISR function type.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] flags     pre-shifted content of the xISR register, the bits
 *                      are aligned to bit zero
 */
typedef void (*samv71_xdmacisr_t)(void *p, uint32_t flags);

/**
 * @brief   STM32 DMA stream descriptor structure.
 */
typedef struct samv71_xdmac_channel {
  XdmacChid             *channel;       /**< @brief Associated DMA channel
                                             Registers.                    */
  uint8_t               selfindex;      /**< @brief Index in array.        */
} samv71_xdmac_channel_t;

#define XDMAC_MBR_UBC_UBLEN_Pos 0
#define XDMAC_MBR_UBC_UBLEN_Msk (0xffffffu << XDMAC_MBR_UBC_UBLEN_Pos)
#define XDMAC_MBR_UBC_UBLEN(value) ((XDMAC_MBR_UBC_UBLEN_Msk & ((value) << XDMAC_MBR_UBC_UBLEN_Pos)))
#define XDMAC_MBR_UBC_NDE (0x1u << 24)
#define XDMAC_MBR_UBC_NSEN (0x1u << 25)
#define XDMAC_MBR_UBC_NDEN (0x1u << 26)
#define XDMAC_MBR_UBC_NVIEW_Pos 27
#define XDMAC_MBR_UBC_NVIEW_Msk (0x3u << XDMAC_MBR_UBC_NVIEW_Pos)
#define XDMAC_MBR_UBC_NVIEW(value) ((XDMAC_MBR_UBC_NVIEW_Msk & ((value) << XDMAC_MBR_UBC_NVIEW_Pos)))
#define XDMAC_MBR_UBC_NVIEW_NDV0 (0x0u << 27)
#define XDMAC_MBR_UBC_NVIEW_NDV1 (0x1u << 27)
#define XDMAC_MBR_UBC_NVIEW_NDV2 (0x2u << 27)
#define XDMAC_MBR_UBC_NVIEW_NDV3 (0x3u << 27)

//note that there is nothing in this struct saying what it is; that information
//is either in the previous descriptor or in the CNDC register before the
//channel is started.
typedef struct samv71_xdmac_linked_list_base {
    struct samv71_xdmac_linked_list_base *XDMAC_MBR_NDA;
    //NDE use next descriptor; gets loaded into CNDC.NDE
    //NSEN update destination parameters with next view; gets loaded into CNDC.NDSUP
    //NDEN update source parameters with next view; gets loaded into CNDC.NDDUP
    //NVIEW which type of view the next descriptor is; 0-3, map to samv71_xdmac_linked_list_view_{0-3}_t; gets loaded into CNDC.NDVIEW
    //UBLEN gets loaded into CUBC
    uint32_t XDMAC_MBR_UBC;
} samv71_xdmac_linked_list_base_t;

typedef struct {
    struct samv71_xdmac_linked_list_base *XDMAC_MBR_NDA;
    uint32_t XDMAC_MBR_UBC;
    void const *XDMAC_MBR_TA; //"transfer address member" either sa or da, depending on the nsen/nden bits in the previous descriptor; thus, this will be loaded into either CSA or CDA.
} samv71_xdmac_linked_list_view_0_t;

typedef struct {
    struct samv71_xdmac_linked_list_base *XDMAC_MBR_NDA;
    uint32_t XDMAC_MBR_UBC;
    void const *XDMAC_MBR_SA;//source address; gets loaded into CSA(depending on CNDC.NDSUP/nsen?)
    void *XDMAC_MBR_DA;//destination address; gets loaded into CDA(depending on CNDC.NDDUP/nden?)
} samv71_xdmac_linked_list_view_1_t;

typedef struct {
    struct samv71_xdmac_linked_list_base *XDMAC_MBR_NDA;
    uint32_t XDMAC_MBR_UBC;
    void *XDMAC_MBR_SA;
    void *XDMAC_MBR_DA;
    uint32_t XDMAC_MBR_CFG;//configuration register; gets loaded into CC (selectively depending on CNDC.NDSUP/nsen/CNDC.NDDUP/nden?)
} samv71_xdmac_linked_list_view_2_t;

typedef struct {
    struct samv71_xdmac_linked_list_base *XDMAC_MBR_NDA;
    uint32_t XDMAC_MBR_UBC;
    void *XDMAC_MBR_SA;
    void *XDMAC_MBR_DA;
    uint32_t XDMAC_MBR_CFG;
    uint32_t XDMAC_MBR_BC; //block control register; gets loaded into CBC
    uint32_t XDMAC_MBR_DS; //data stride register; gets loaded into CDS_MSP(selectively depending on CNDC.NDSUP/nsen/CNDC.NDDUP/nden?)
    uint32_t XDMAC_MBR_SUS; //source microblock stride register; gets loaded into CSUS(depending on CNDC.NDSUP/nsen?)
    uint32_t XDMAC_MBR_DUS; //destination microblock stride register; gets loaded into CDUS(depending on CNDC.NDDUP/nden?)
} samv71_xdmac_linked_list_view_3_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */

/**
 * @brief   Sets the interrupt causes.
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 * @param[in] causes      value to be written in the CIE/CID registers
 *
 * @special
 */
#define xdmacChannelSetInterruptCauses(xdmacchp, causes) {                    \
  (xdmacchp)->channel->XDMAC_CID  = ~(uint32_t)(causes);                            \
  (xdmacchp)->channel->XDMAC_CIE  = (uint32_t)(causes);                             \
}

/**
 * @brief   Set the source address for the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 * @param[in] addr      value to be written in the CSA register
 *
 * @special
 */
#define xdmacChannelSetSource(xdmacchp, addr) {                              \
  (xdmacchp)->channel->XDMAC_CSA  = (uint32_t)(addr);                        \
}

/**
 * @brief   Set the destination address for the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 * @param[in] addr      value to be written in the CDA register
 *
 * @special
 */
#define xdmacChannelSetDestination(xdmacchp, addr) {                         \
  (xdmacchp)->channel->XDMAC_CDA  = (uint32_t)(addr);                        \
}

/**
 * @brief   Set the address and bus interface of the next descriptor for the
 *          channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @param[in] addr      pointer to a samv71_xdmac_linked_list_view_[[0123]]_t,
 *                      to be written in the CNDA register
 * @param[in] ndaif     Bus interface to use for fetching the descriptor
 * @special
 */
#define xdmacChannelSetNextDescriptor(xdmacchp, addr, ndaif) {               \
  (xdmacchp)->channel->XDMAC_CNDA  = (uint32_t)(addr) |                      \
                                     (ndaif?XDMAC_CNDA_NDAIF:0);             \
}

/**
 * @brief   Get the address of the next descriptor for the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @return              A pointer to the samv71_xdmac_linked_list_view*_t
 *                      structure containing the next descriptor
 *
 * @special
 */
#define xdmacChannelGetNextDescriptor(xdmacchp)                \
  (samv71_xdmac_linked_list_base_t*)((xdmacchp)->channel->XDMAC_CNDA & ~XDMAC_CNDA_NDAIF)


/**
 * @brief   Set the mode to be used with the next descriptor of the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @param[in] mode      value to be written in the CNDC register
 *
 * @special
 */
#define xdmacChannelSetNextDescriptorMode(xdmacchp, mode) {                  \
  (xdmacchp)->channel->XDMAC_CNDC  = (uint32_t)(mode);                       \
}

/**
 * @brief   Set the microblock length of the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @param[in] size      value to be written in the CUBC register
 *
 * @special
 */
#define xdmacChannelSetMicroblockLength(xdmacchp, size) {                  \
  (xdmacchp)->channel->XDMAC_CUBC  = (uint32_t)(size);                     \
}

/**
 * @brief   Get the remaining microblock length of the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @return    Number of bytes remaining in this microblock
 *
 * @special
 */
#define xdmacChannelGetMicroblockRemaining(xdmacchp) \
  ((xdmacchp)->channel->XDMAC_CUBC)

/**
 * @brief   Set the block length of the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @param[in] size      value to be written in the CBC register
 *                    (actual count -1, i.e. one microblock in this block is 0)
 *
 * @special
 */
#define xdmacChannelSetBlockLength(xdmacchp, size) {                  \
  (xdmacchp)->channel->XDMAC_CBC  = (uint32_t)(size);                     \
}

/**
 * @brief   Get the remaining block length of the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @return    number of microblocks -1
 *
 * @special
 */
#define xdmacChannelGetBlockRemaining(xdmacchp)                   \
  ((xdmacchp)->channel->XDMAC_CBC)

/**
 * @brief   Set the mode of the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 * @param[in] mode      value to be written in the CC register
 *
 * @special
 */
#define xdmacChannelSetMode(xdmacchp, mode) {                  \
  (xdmacchp)->channel->XDMAC_CC  = (uint32_t)(mode);                     \
}

/**
 * @brief   Set the stride of the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 * @param[in] stride      value to be written in the CDS_MSP register
 *
 * @special
 */
#define xdmacChannelSetStride(xdmacchp, stride) {                  \
  (xdmacchp)->channel->XDMAC_CDS_MSP  = (uint32_t)(stride);        \
}

/**
 * @brief   Set the memset value when XDMAC_CC_MEMSET_HW_MODE is used
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 * @param[in] c           value to be written in the CDS_MSP register
 *
 * @special
 */
#define xdmacChannelSetMemsetValue(xdmacchp, c) {             \
  (xdmacchp)->channel->XDMAC_CDS_MSP  = (uint32_t)(c);        \
}

/**
 * @brief   Set the source microblock stride of the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 * @param[in] stride      value to be written in the CSUS register
 *
 * @special
 */
#define xdmacChannelSetSourceMicroblockStride(xdmacchp, stride) {           \
  (xdmacchp)->channel->XDMAC_CSUS  = (int32_t)(stride);                     \
}

/**
 * @brief   Set the destination microblock stride of the channel
 * @note    This function can be invoked in both ISR or thread context.
 *          This function must not be invoked on an enabled channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 * @param[in] stride      value to be written in the CDUS register
 *
 * @special
 */
#define xdmacChannelSetDestinationMicroblockStride(xdmacchp, stride) {      \
  (xdmacchp)->channel->XDMAC_CDUS  = (int32_t)(stride);                     \
}

/**
 * @brief   DMA channel enable.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 *
 * @special
 */
#define xdmacChannelEnable(xdmacchp) {                                      \
  XDMAC->XDMAC_GE = XDMAC_GE_EN0 << ((xdmacchp)->selfindex);                \
  XDMAC->XDMAC_GIE = XDMAC_GIE_IE0 << ((xdmacchp)->selfindex);              \
}

/**
 * @brief   DMA channel disable.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 *
 * @special
 */
#define xdmacChannelDisable(xdmacchp) {                                     \
  XDMAC->XDMAC_GID = XDMAC_GID_ID0 << ((xdmacchp)->selfindex);              \
  XDMAC->XDMAC_GD = XDMAC_GD_DI0 << ((xdmacchp)->selfindex);                \
  while (XDMAC->XDMAC_GS & (XDMAC_GS_ST0 << (xdmacchp)->selfindex))         \
    ;                                                                       \
  xdmacChannelClearInterrupt(xdmacchp);                                     \
}

/**
 * @brief   DMA channel suspend.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 *
 * @special
 */
#define xdmacChannelSuspend(xdmacchp) {                                      \
  XDMAC->XDMAC_GRWS = XDMAC_GRWS_RWS0 << ((xdmacchp)->selfindex);            \
}

/**
 * @brief   DMA channel resume.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 *
 * @special
 */
#define xdmacChannelResume(xdmacchp) {                                      \
  XDMAC->XDMAC_GRWR = XDMAC_GRWR_RWR0 << ((xdmacchp)->selfindex);           \
}

/**
 * @brief   DMA channel software request.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 *
 * @special
 */
#define xdmacChannelSoftwareRequest(xdmacchp) {                             \
  XDMAC->XDMAC_GSWR = XDMAC_GSWR_SWREQ0 << ((xdmacchp)->selfindex);         \
}

/**
 * @brief   DMA channel software flush.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 *
 * @special
 */
#define xdmacChannelSoftwareFlush(xdmacchp) {                             \
  XDMAC->XDMAC_GSWF = XDMAC_GSWF_SWF0 << ((xdmacchp)->selfindex);         \
}

/**
 * @brief   DMA channel interrupt sources clear.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 *
 * @special
 */
#define xdmacChannelClearInterrupt(xdmacchp) {                                 \
  uint32_t cis = (xdmacchp)->channel->XDMAC_CIS;                              \
  (void)cis;                                                                  \
}

/**
 * @brief   Starts a microblock operation using the specified channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @param[in] mode      value to be written in the CC register
 *                      a combination of
 *                      - @p XDMAC_CC_TYPE if not memory to memory
 *                      - @p XDMAC_CC_MBSIZE_*
 *                      - @p XDMAC_CC_SWREQ_*
 *                      - @p XDMAC_CC_SAM_*
 *                      - @p XDMAC_CC_DAM_*
 *                      - @p XDMAC_CC_DSYNC_* if not memory to memory
 *                      - @p XDMAC_CC_CSIZE_*
 *                      - @p XDMAC_CC_DWIDTH_*
 *                      - @p XDMAC_CC_SIF_*
 *                      - @p XDMAC_CC_DIF_*
 *                      - @p XDMAC_CC_PERID(SAMV71_XDMAC_HWREQ_*) if peripheral triggered
 * @param[in] src       source address
 * @param[in] dst       destination address
 * @param[in] n         number of data units to copy
 */
#define xdmacChannelStartSingleMicroblock(xdmacchp, mode, dst, src, n) {    \
  xdmacChannelClearInterrupt(xdmacchp);                                     \
  xdmacChannelSetSource(xdmacchp,src);                                      \
  xdmacChannelSetDestination(xdmacchp,dst);                                 \
  xdmacChannelSetMicroblockLength(xdmacchp,n);                              \
  xdmacChannelSetMode(xdmacchp,mode);                                       \
  xdmacChannelSetNextDescriptorMode(xdmacchp,0);                            \
  xdmacChannelSetBlockLength(xdmacchp,0);                                   \
  xdmacChannelSetStride(xdmacchp,0);                                        \
  xdmacChannelSetSourceMicroblockStride(xdmacchp,0);                        \
  xdmacChannelSetDestinationMicroblockStride(xdmacchp,0);                   \
  xdmacChannelEnable(xdmacchp);                                             \
}

/**
 * @brief   Starts a memory to memory operation using the specified channel.
 * @note    The default transfer data mode is "byte to byte" but it can be
 *          changed by specifying extra options in the @p mode parameter.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @param[in] mode      value to be written in the CC register, this value
 *                      is implicitly ORed with:
 *                      - @p XDMAC_CC_SWREQ_SWR_CONNECTED
 *                      - @p XDMAC_CC_SAM_INCREMENTED_AM
 *                      - @p XDMAC_CC_DAM_INCREMENTED_AM
 *                      .
 * @param[in] src       source address
 * @param[in] dst       destination address
 * @param[in] n         number of data units to copy
 */
#define xdmacStartMemCopy(xdmacchp, mode, src, dst, n) {                    \
  xdmacChannelStartSingleMicroblock(xdmacchp, mode |                        \
    XDMAC_CC_SWREQ_SWR_CONNECTED |                                          \
    XDMAC_CC_SAM_INCREMENTED_AM |                                           \
    XDMAC_CC_DAM_INCREMENTED_AM, src, dst, n );                             \
}

/**
 * @brief   Starts a Block operation using the specified channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @param[in] mode      value to be written in the CC register
 *                      a combination of
 *                      - @p XDMAC_CC_TYPE if not memory to memory
 *                      - @p XDMAC_CC_MBSIZE_*
 *                      - @p XDMAC_CC_SWREQ_*
 *                      - @p XDMAC_CC_SAM_*
 *                      - @p XDMAC_CC_DAM_*
 *                      - @p XDMAC_CC_DSYNC_* if not memory to memory
 *                      - @p XDMAC_CC_CSIZE_*
 *                      - @p XDMAC_CC_DWIDTH_*
 *                      - @p XDMAC_CC_SIF_*
 *                      - @p XDMAC_CC_DIF_*
 *                      - @p XDMAC_CC_PERID(SAMV71_XDMAC_HWREQ_*) if peripheral triggered
 * @param[in] src       source address
 * @param[in] dst       destination address
 * @param[in] n         number of data units to copy per microblock
 * @param[in] nb        number of microblocks to copy
 *                      effective number of bytes transfered is @p n times @p nb
 */
#define xdmacChannelStartSingleBlock(xdmacchp, mode, dst, src, n, nb) {     \
  xdmacChannelClearInterrupt(xdmacchp);                                     \
  xdmacChannelSetSource(xdmacchp,src);                                      \
  xdmacChannelSetDestination(xdmacchp,dst);                                 \
  xdmacChannelSetMicroblockLength(xdmacchp,n);                              \
  xdmacChannelSetMode(xdmacchp,mode);                                       \
  xdmacChannelSetBlockLength(xdmacchp,nb-1);                                \
  xdmacChannelSetNextDescriptorMode(xdmacchp,0);                            \
  xdmacChannelSetStride(xdmacchp,0);                                        \
  xdmacChannelSetSourceMicroblockStride(xdmacchp,0);                        \
  xdmacChannelSetDestinationMicroblockStride(xdmacchp,0);                   \
  xdmacChannelEnable(xdmacchp);                                             \
}

/**
 * @brief   Starts a master transfer operation using the specified channel.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp  pointer to a samv71_xdmac_channel_t structure
 * @param[in] dmamode   Mode to be used with the first transfer
 *                      a combination of
 *                      - @p XDMAC_CNDC_NDVIEW_*
 *                      - @p XDMAC_CNDC_NDDUP_*
 *                      - @p XDMAC_CNDC_NDSUP_*
 *                      this is implicitly ORed with @p XDMAC_CNDC_NDE
 * @param[in] dmalist   pointer to a one of the
 *                      samv71_xdmac_linked_list_view_*_t structures.
 *                      The XDMAC_CNDC_NDVIEW_NDV must in dmamode must match
 *                      the structure.
 * @param[in] ndaif     Bus interface to use for fetching the linked list
 *                      structure. 0 for IF0, or anything else for IF1
 *
 */
#define xdmacChannelStartMasterTransfer(xdmacchp, dmamode, dmalist, ndaif) {  \
  xdmacChannelClearInterrupt(xdmacchp);                                     \
  xdmacChannelSetNextDescriptor(xdmacchp, dmalist, ndaif);                  \
  xdmacChannelSetNextDescriptorMode(xdmacchp,dmamode|XDMAC_CNDC_NDE);       \
  xdmacChannelEnable(xdmacchp);                                             \
}
/* Note that when NDE is enabled, enabling the channel via GE directly leads
 * to a fetch of the next descriptor.
 */

/**
 * @brief   Polled wait for DMA transfer end.
 * @pre     The channel must have been allocated using @p xdmacChannelAlloc().
 * @post    After use the channel can be released using @p xdmacChannelFree().
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 */
#define xdmacWaitCompletion(xdmacchp) {                                     \
  XDMAC->XDMAC_GID = (XDMAC_GID_ID0 << (xdmacchp)->selfindex);              \
  while (XDMAC->XDMAC_GS & (XDMAC_GS_ST0 << (xdmacchp)->selfindex))         \
    ;                                                                       \
  xdmacChannelClearInterrupt(xdmacchp);                                     \
}

#define xdmacAutomaticInterfaceBits_CCReg(srcaddr,dstaddr)                  \
  ((MATRIX_DMA_IF(srcaddr) == 3 && MATRIX_DMA_IF(dstaddr) == 3)?            \
      (XDMAC_CC_SIF_AHB_IF0|XDMAC_CC_DIF_AHB_IF1):                          \
   (MATRIX_DMA_IF(srcaddr) == 3 && MATRIX_DMA_IF(dstaddr) == 2)?            \
      (XDMAC_CC_SIF_AHB_IF0|XDMAC_CC_DIF_AHB_IF1):                          \
   (MATRIX_DMA_IF(srcaddr) == 3 && MATRIX_DMA_IF(dstaddr) == 1)?            \
      (XDMAC_CC_SIF_AHB_IF1|XDMAC_CC_DIF_AHB_IF0):                          \
   (MATRIX_DMA_IF(srcaddr) == 1 && MATRIX_DMA_IF(dstaddr) == 3)?            \
      (XDMAC_CC_SIF_AHB_IF0|XDMAC_CC_DIF_AHB_IF1):                          \
   (MATRIX_DMA_IF(srcaddr) == 2 && MATRIX_DMA_IF(dstaddr) == 3)?            \
      (XDMAC_CC_SIF_AHB_IF1|XDMAC_CC_DIF_AHB_IF0):                          \
   (MATRIX_DMA_IF(srcaddr) == 1 && MATRIX_DMA_IF(dstaddr) == 2)?            \
      (XDMAC_CC_SIF_AHB_IF0|XDMAC_CC_DIF_AHB_IF1):                          \
   (MATRIX_DMA_IF(srcaddr) == 2 && MATRIX_DMA_IF(dstaddr) == 1)?            \
      (XDMAC_CC_SIF_AHB_IF1|XDMAC_CC_DIF_AHB_IF0):                          \
   (MATRIX_DMA_IF(srcaddr) == 1 && MATRIX_DMA_IF(dstaddr) == 1)?            \
      (XDMAC_CC_SIF_AHB_IF0|XDMAC_CC_DIF_AHB_IF0):                          \
   (MATRIX_DMA_IF(srcaddr) == 2 && MATRIX_DMA_IF(dstaddr) == 2)?            \
      (XDMAC_CC_SIF_AHB_IF1|XDMAC_CC_DIF_AHB_IF1):                          \
      0)


/**
 * @brief   Releases a DMA stream.
 * @details The stream is freed and, if required, the DMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] xdmacchp    pointer to a samv71_xdmac_channel_t structure
 *
 * @api
 */
#define xdmacChannelFree(xdmacchp) {                                        \
  osalSysLock();                                                            \
  xdmacChannelFreeI(xdmacchp);                                              \
  osalSysUnlock();                                                          \
}


/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern const samv71_xdmac_channel_t _samv71_xdmac_channels[SAMV71_XDMAC_CHANNELS];
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void xdmacInit(void);
  const samv71_xdmac_channel_t *xdmacChannelAllocI(samv71_xdmacisr_t func,
                                            void *param);
  const samv71_xdmac_channel_t *xdmacChannelAlloc(samv71_xdmacisr_t func,
                                           void *param);
  void xdmacChannelFreeI(const samv71_xdmac_channel_t *dmachp);
#ifdef __cplusplus
}
#endif

#endif /* SAMV71_XDMAC_H */

/** @} */
