#pragma once

#include "hal.h"
#include "samv71.h"

#define MATRIX_MASTER_ID_CORTEX_M7_CODE 0
#define MATRIX_MASTER_ID_CORTEX_M7_USB_HS_EXT_MEM_PERIPH 1
#define MATRIX_MASTER_ID_CORTEX_M7_PERIPHERAL 2
#define MATRIX_MASTER_ID_ICM 3
#define MATRIX_MASTER_ID_DMA_IF0 4
#define MATRIX_MASTER_ID_DMA_IF1 5
#define MATRIX_MASTER_ID_ISI 6
#define MATRIX_MASTER_ID_MEDIALB 7
#define MATRIX_MASTER_ID_USB 8
#define MATRIX_MASTER_ID_GMAC 9
#define MATRIX_MASTER_ID_CAN0 10
#define MATRIX_MASTER_ID_CAN1 11
#define MATRIX_MASTER_ID_CORTEX_M7_QSPI 12

#define MATRIX_SLAVE_SRAM_ICM_DMA_IF0 0
#define MATRIX_SLAVE_SRAM_OTHER_DMA 1
#define MATRIX_SLAVE_ROM 2
#define MATRIX_SLAVE_FLASH 3
#define MATRIX_SLAVE_USB_HS_RAM 4
#define MATRIX_SLAVE_EXT_BUS 5
#define MATRIX_SLAVE_QSPI 6
#define MATRIX_SLAVE_PERIPHERAL_BRIDGE 7
#define MATRIX_SLAVE_CORTEX_M7_AHBS 8

/**
 * Determine if an address can be reached via XDMAC interface 0
 *
 * @return 0: cannot be reached through interface 0
 *         1: can be reached through interface 0
 */
#define MATRIX_DMA_IF0(addr) \
  ((((uintptr_t)(addr) >= 0x20000000U && (uintptr_t)(addr) < 0x40000000U) || \
    ((uintptr_t)(addr) >= 0x60000000U && (uintptr_t)(addr) < 0x80000000U) || \
    ((uintptr_t)(addr) >= 0xe0000000U && (uintptr_t)(addr) <= 0xffffffffU))?1:0)

/**
 * Determine if an address can be reached via XDMAC interface 1
 *
 * @return 0: cannot be reached through interface 1
 *         1: can be reached through interface 1
 */
#define MATRIX_DMA_IF1(addr) \
  ((((uintptr_t)(addr) >= 0x00400000U && (uintptr_t)(addr) < 0x00800000U) || \
    ((uintptr_t)(addr) >= 0x20000000U && (uintptr_t)(addr) < 0xa0000000U))?1:0)

/**
 * Returns a bitmask for the wired DMA interfaces, by address
 *
 * @return bit 0: interface 0, bit 1: interface 1; or
 *         0: cannot be reached through either interface
 *         1: can only be reached through interface 0
 *         2: can only be reached through interface 1
 *         3: can be reached through both interfaces
 */
#define MATRIX_DMA_IF(addr) \
  ((MATRIX_DMA_IF1(addr) << 1) | MATRIX_DMA_IF0(addr))

uint32_t matrix_set_periph_master_addr(uint32_t ul_master_id, void *addr);
