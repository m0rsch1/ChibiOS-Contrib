#pragma once

#include "hal.h"
#include "samv71.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t pmc_enable_periph_clk(uint32_t ul_id);
uint32_t pmc_disable_periph_clk(uint32_t ul_id);

uint32_t pmc_enable_generic_clk(uint32_t ul_id);
uint32_t pmc_disable_generic_clk(uint32_t ul_id);

/**
 * @param source one of PMC_PCR_GCLKCSS_SLOW_CLK, PMC_PCR_GCLKCSS_MAIN_CLK,
 *   PMC_PCR_GCLKCSS_PLLA_CLK, PMC_PCR_GCLKCSS_UPLL_CLK, PMC_PCR_GCLKCSS_MCK_CLK
 * @param divider 1-256
 */
uint32_t pmc_configure_generic_clk(uint32_t ul_id, uint32_t source, uint32_t divider);

uint32_t pmc_enable_programmable_clk(uint32_t pck_id);
uint32_t pmc_disable_programmable_clk(uint32_t pck_id);

/**
 * @param source one of PMC_PCK_CSS_SLOW_CLK, PMC_PCK_CSS_MAIN_CLK,
 *  PMC_PCK_CSS_PLLA_CLK, PMC_PCK_CSS_UPLL_CLK, PMC_PCK_CSS_MCK
 * @param divider 1-256
 */
uint32_t pmc_configure_programmable_clk(uint32_t pck_id, uint32_t source, uint32_t divider);

uint32_t pmc_enable_utmi_clk(void);
uint32_t pmc_disable_utmi_clk(void);

/**
 * @param freq_mode one of UTMI_CKTRIM_FREQ_XTAL12, UTMI_CKTRIM_FREQ_XTAL16 for
 *                  PLL frequency multiplication of x40 and x30.
 */
uint32_t pmc_configure_utmi_clk(uint32_t freq_mode);

#ifdef __cplusplus
}
#endif
