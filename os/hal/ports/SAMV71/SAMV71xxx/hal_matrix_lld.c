
#include "hal_matrix_lld.h"

uint32_t matrix_set_periph_master_addr(uint32_t ul_master_id, void *addr) {

    volatile uint32_t *reg;
    switch(ul_master_id) {
    case MATRIX_MASTER_ID_CAN0:
        reg = &MATRIX->CCFG_CAN0;
        break;
    case MATRIX_MASTER_ID_CAN1:
        reg = &MATRIX->CCFG_SYSIO;
        break;
    default:
        return 1;
    }

    uint32_t val = *reg;
    val &= 0x0000ffffU;
    val |= ((uintptr_t)addr) & 0xffff0000U;
    *reg = val;
    return 0;
}

