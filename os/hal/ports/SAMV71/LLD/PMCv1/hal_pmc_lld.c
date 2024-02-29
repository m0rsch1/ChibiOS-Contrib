#include "hal_pmc_lld.h"

#include "system_samv71.h"
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
        //read the current settings and modify to contain enable bit
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

uint32_t pmc_disable_periph_clk(uint32_t ul_id)
{
#if defined(REG_PMC_PCR)
        uint32_t pcr;
        PMC->PMC_PCR = ul_id & 0x7F;
        pcr = PMC->PMC_PCR | PMC_PCR_CMD;
        PMC->PMC_PCR = pcr;
        return 0;
#else
        if (ul_id > MAX_PERIPH_ID) {
                return 1;
        }

        if (ul_id < 32) {
                if ((PMC->PMC_PCSR0 & (1u << ul_id)) == (1u << ul_id)) {
                        PMC->PMC_PCDR0 = 1 << ul_id;
                }
        } else {
                ul_id -= 32;
                if ((PMC->PMC_PCSR1 & (1u << ul_id)) == (1u << ul_id)) {
                        PMC->PMC_PCDR1 = 1 << ul_id;
                }
        }

        return 0;
#endif /* defined(REG_PMC_PCR) */
}

uint32_t pmc_enable_generic_clk(uint32_t ul_id)
{
    PMC->PMC_PCR = PMC_PCR_PID(ul_id);
    uint32_t pcr = PMC->PMC_PCR;
    pcr |= PMC_PCR_CMD;
    pcr |= PMC_PCR_GCLKEN;
    PMC->PMC_PCR = pcr;
    return 0;
}

uint32_t pmc_disable_generic_clk(uint32_t ul_id)
{
    PMC->PMC_PCR = PMC_PCR_PID(ul_id);
    uint32_t pcr = PMC->PMC_PCR;
    pcr |= PMC_PCR_CMD;
    pcr &= ~PMC_PCR_GCLKEN;
    PMC->PMC_PCR = pcr;
    return 0;
}

uint32_t pmc_configure_generic_clk(uint32_t ul_id, uint32_t source, uint32_t divider)
{
    PMC->PMC_PCR = PMC_PCR_PID(ul_id);
    uint32_t pcr = PMC->PMC_PCR;
    pcr |= PMC_PCR_CMD;
    pcr &= ~(PMC_PCR_GCLKCSS_Msk | PMC_PCR_GCLKDIV_Msk);
    pcr |= source & PMC_PCR_GCLKCSS_Msk;
    pcr |= PMC_PCR_GCLKDIV(divider+1);
    PMC->PMC_PCR = pcr;
    return 0;
}

uint32_t pmc_enable_programmable_clk ( uint32_t pck_id )
{
    if(pck_id >= 8)
        return -1;
    PMC->PMC_SCER = PMC_SCER_PCK0 << pck_id;
    ProgrammableClockUpdate(pck_id);
    return 0;
}

uint32_t pmc_disable_programmable_clk ( uint32_t pck_id )
{
    if(pck_id >= 8)
        return -1;
    PMC->PMC_SCDR = PMC_SCDR_PCK0 << pck_id;
    ProgrammableClockUpdate(pck_id);
    return 0;
}

uint32_t pmc_configure_programmable_clk ( uint32_t pck_id, uint32_t source, uint32_t divider )
{
    if(pck_id >= 8 || divider <= 0 || divider > 256)
        return -1;
    PMC->PMC_PCK[pck_id] = (PMC_PCK_CSS_Msk & source) | PMC_PCK_PRES(divider-1);
    ProgrammableClockUpdate(pck_id);
    return 0;
}

#define SYS_BOARD_UPLLCOUNT CKGR_PLLAR_PLLACOUNT((300*CHIP_FREQ_SLCK_RC_MAX+1000000-1)/1000000)

uint32_t pmc_enable_utmi_clk()
{
    PMC->CKGR_UCKR = CKGR_UCKR_UPLLCOUNT(SYS_BOARD_UPLLCOUNT) | CKGR_UCKR_UPLLEN;
    return 0;
}

uint32_t pmc_disable_utmi_clk()
{
    PMC->CKGR_UCKR = 0;
    for(int pck_id = 0; pck_id < 8; pck_id++) {
        ProgrammableClockUpdate(pck_id);
    }
    return 0;
}

uint32_t pmc_configure_utmi_clk ( uint32_t freq_mode )
{
    if(freq_mode == ~0U) {
        freq_mode = SAMV71_MAINCLK == 12000000?UTMI_CKTRIM_FREQ_XTAL12:
                    UTMI_CKTRIM_FREQ_XTAL16;
    }
    UTMI->UTMI_CKTRIM = freq_mode;
    for(int pck_id = 0; pck_id < 8; pck_id++) {
        ProgrammableClockUpdate(pck_id);
    }
    return 0;
}
