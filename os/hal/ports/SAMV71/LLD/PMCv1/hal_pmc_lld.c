#include "hal_pmc_lld.h"

/**
 * \brief Enable the specified peripheral clock.
 *
 * \note The ID must NOT be shifted (i.e., 1 << ID_xxx).
 *
 * \param ul_id Peripheral ID (ID_xxx).
 *
 * \retval 0 Success.
 * \retval 1 Invalid parameter.
 */
uint32_t pmc_enable_periph_clk(uint32_t ul_id)
{
#if defined(REG_PMC_PCR)
        uint32_t pcr;
        PMC->PMC_PCR = ul_id & 0x7F;
        pcr = PMC->PMC_PCR | PMC_PCR_EN | PMC_PCR_CMD;
        PMC->PMC_PCR = pcr;
        return 0;
#else
        if (ul_id > MAX_PERIPH_ID) {
                return 1;
        }

        if (ul_id < 32) {
                if ((PMC->PMC_PCSR0 & (1u << ul_id)) != (1u << ul_id)) {
                        PMC->PMC_PCER0 = 1 << ul_id;
                }
        } else {
                ul_id -= 32;
                if ((PMC->PMC_PCSR1 & (1u << ul_id)) != (1u << ul_id)) {
                        PMC->PMC_PCER1 = 1 << ul_id;
                }
        }

        return 0;
#endif /* defined(REG_PMC_PCR) */
}

