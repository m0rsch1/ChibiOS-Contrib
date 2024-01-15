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
 * @file    hal_adc_lld.c
 * @brief   PLATFORM ADC subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "hal.h"
#include "samv71_xdmac.h"

#if (HAL_USE_ADC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   ADC1 driver identifier.
 */
#if (SAMV71_ADC_USE_ADC0 == TRUE) || defined(__DOXYGEN__)
ADCDriver ADCD0;
#endif
#if (SAMV71_ADC_USE_ADC1 == TRUE) || defined(__DOXYGEN__)
ADCDriver ADCD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void adc_lld_serve_interrupt(ADCDriver *adcp) {
    uint32_t isr = adcp->device->AFEC_ISR;

    if((isr & AFEC_ISR_GOVRE) != 0 && adcp->state == ADC_ACTIVE) {
        //reading to clear the interrupt
        uint32_t over = adcp->device->AFEC_OVER;
        (void)over;
        //this calls adc_lld_stop_conversion
        _adc_isr_error_code(adcp, ADC_ERR_OVERFLOW);
    }
    if((isr & AFEC_ISR_COMPE) != 0 && adcp->state == ADC_ACTIVE) {
        //this calls adc_lld_stop_conversion
        _adc_isr_error_code(adcp, ADC_ERR_AWD);
    }
    if((adcp->config->flags & ADC_FLAG_USE_DMA) == 0 &&
        (isr & AFEC_ISR_DRDY) != 0 && adcp->state == ADC_ACTIVE) {
        if(adcp->samples && adcp->depth * adcp->grpp->num_channels > adcp->current_pos) {
            uint32_t lcdr = adcp->device->AFEC_LCDR;
            adcp->samples[adcp->current_pos] =
                lcdr & AFEC_LCDR_LDATA_Msk;
            osalDbgAssert(((lcdr & AFEC_LCDR_CHNB_Msk) >> AFEC_LCDR_CHNB_Pos) ==
            ((adcp->grpp->channel_sequence >> (adcp->current_channel*4)) & 0xf), "Unexpected channel");
            adcp->current_pos++;
            do {
                adcp->current_channel++;
                if(adcp->current_channel >= 12) {
                    adcp->current_channel = 0;
                }
            } while ((adcp->grpp->channel_enabled & (1 << adcp->current_channel)) == 0);

            if(adcp->current_pos ==
                adcp->depth * adcp->grpp->num_channels / 2) {
                //this may call adc_lld_stop_conversion
                _adc_isr_half_code(adcp);
            } else if(adcp->current_pos >= adcp->depth * adcp->grpp->num_channels) {
                if(adcp->grpp->circular && adcp->state == ADC_ACTIVE) {
                    adcp->current_pos = 0;
                    adcp->current_channel = 0;
                    while ( ( adcp->grpp->channel_enabled & ( 1 << adcp->current_channel ) ) == 0 ) {
                        adcp->current_channel++;
                    }
                }
                //this may call adc_lld_stop_conversion
                _adc_isr_full_code(adcp);
            }
        }
    }
}

void adc_lld_dma_func(void *param, uint32_t flags) {
    ADCDriver *adcp = (ADCDriver *)param;
    if((flags & (XDMAC_CIS_RBEIS | XDMAC_CIS_WBEIS | XDMAC_CIS_ROIS)) != 0 &&
            adcp->state == ADC_ACTIVE) {
        //this calls adc_lld_stop_conversion
        _adc_isr_error_code(adcp, ADC_ERR_DMAFAILURE);
    }
    if((flags & (XDMAC_CIS_BIS | XDMAC_CIS_LIS)) != 0 &&
            adcp->state == ADC_ACTIVE) {
        //after the wraparound, the Next Descriptor will already point to the
        //next dma descriptor (the one after #0). This situation does not
        //happen at startup; the Block interrupt happens after moving to the
        //second descriptor, and Next Descriptor then points to the third.
        //If it is not a circular buffer, wraparound does not happen but
        //LIS is usable instead.
        if(xdmacChannelGetNextDescriptor(adcp->dma_channel) ==
                (samv71_xdmac_linked_list_base_t*)&adcp->dma_descriptors[1] ||
                (flags & XDMAC_CIS_LIS) != 0) {
            //this may call adc_lld_stop_conversion
            _adc_isr_full_code(adcp);
        } else {
            //this may call adc_lld_stop_conversion
            _adc_isr_half_code(adcp);
        }
    }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (SAMV71_ADC_USE_ADC0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(AFEC0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    adc_lld_serve_interrupt(&ADCD0);
    OSAL_IRQ_EPILOGUE();
}
#endif

#if (SAMV71_ADC_USE_ADC1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(AFEC1_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    adc_lld_serve_interrupt(&ADCD1);
    OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {

#if SAMV71_ADC_USE_ADC0 == TRUE
  /* Driver initialization.*/
  adcObjectInit(&ADCD0);
  ADCD0.device = AFEC0;
#endif
#if SAMV71_ADC_USE_ADC1 == TRUE
  /* Driver initialization.*/
  adcObjectInit(&ADCD1);
  ADCD1.device = AFEC1;
#endif
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start(ADCDriver *adcp) {

  if (adcp->state == ADC_STOP) {
    /* Enables the peripheral.*/
#if SAMV71_ADC_USE_ADC0 == TRUE
    if (&ADCD0 == adcp) {
      pmc_enable_periph_clk(ID_AFEC0);
      nvicEnableVector ( AFEC0_NVIC_NUMBER, AFEC_NVIC_PRIORITY );
    }
#endif
#if SAMV71_ADC_USE_ADC1 == TRUE
    if (&ADCD1 == adcp) {
      pmc_enable_periph_clk(ID_AFEC1);
      nvicEnableVector ( AFEC1_NVIC_NUMBER, AFEC_NVIC_PRIORITY );
    }
#endif
  }
  /* Configures the peripheral.*/

  uint8_t pre = AFEC_MAIN_CLK / 17 / adcp->config->speed;

  adcp->device->AFEC_MR =
        AFEC_MR_TRACKTIM(15) | AFEC_MR_TRANSFER(2) | AFEC_MR_ONE | //constants
        AFEC_MR_PRESCAL(pre) |
        (adcp->config->mode & AFEC_CONFIG_MODE_MASK);
  adcp->device->AFEC_EMR = AFEC_EMR_TAG;
  adcp->device->AFEC_ACR = AFEC_ACR_IBCTL(3) | AFEC_ACR_PGA0EN | AFEC_ACR_PGA1EN;

  //if needed, these can be changeable afterwards. since some of these
  //settings are expensive (relatively), we only have the channel enable
  //in the conversion group. Only the channel offsets would be problematic when
  //we precalculate the register contents, though.
  if((adcp->config->flags & ADC_FLAG_NUMERIC_CHANNEL_ORDER) != 0)
    adcp->device->AFEC_MR &= ~AFEC_MR_USEQ;
  else {
    adcp->device->AFEC_MR |= AFEC_MR_USEQ;
  }
  adcp->device->AFEC_DIFFR = adcp->config->channel_differential;
  adcp->device->AFEC_SHMR = adcp->config->channel_sh_dual;

  for(int i = 0; i < 12; i++) {
      adcp->device->AFEC_CSELR = AFEC_CSELR_CSEL(i);
      adcp->device->AFEC_COCR = AFEC_COCR_AOFF(adcp->config->channel_offset[i]);
  }

  adcp->device->AFEC_CGR = AFEC_CGR_GAIN0(adcp->config->channel_gain[0]) |
                             AFEC_CGR_GAIN1 ( adcp->config->channel_gain[1] ) |
                             AFEC_CGR_GAIN2 ( adcp->config->channel_gain[2] ) |
                             AFEC_CGR_GAIN3 ( adcp->config->channel_gain[3] ) |
                             AFEC_CGR_GAIN4 ( adcp->config->channel_gain[4] ) |
                             AFEC_CGR_GAIN5 ( adcp->config->channel_gain[5] ) |
                             AFEC_CGR_GAIN6 ( adcp->config->channel_gain[6] ) |
                             AFEC_CGR_GAIN7 ( adcp->config->channel_gain[7] ) |
                             AFEC_CGR_GAIN8 ( adcp->config->channel_gain[8] ) |
                             AFEC_CGR_GAIN9 ( adcp->config->channel_gain[9] ) |
                             AFEC_CGR_GAIN10 ( adcp->config->channel_gain[10] ) |
                             AFEC_CGR_GAIN11 ( adcp->config->channel_gain[11] );

  if((adcp->config->flags & ADC_FLAG_USE_DMA) != 0) {
    if(!adcp->dma_channel)
      adcp->dma_channel = xdmacChannelAllocI(adc_lld_dma_func, adcp);
  } else {
    adcp->dma_channel = NULL;
  }
}

/**
 * @brief   Deactivates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop(ADCDriver *adcp) {

  if (adcp->state == ADC_READY) {
    if(adcp->dma_channel)
      xdmacChannelFreeI(adcp->dma_channel);
    adcp->dma_channel = NULL;

    /* Resets the peripheral.*/
    ( void ) adcp->config;
    adcp->device->AFEC_CHDR = 0xfff;

    /* Disables the peripheral.*/
#if SAMV71_ADC_USE_ADC0 == TRUE
    if (&ADCD0 == adcp) {
      nvicDisableVector ( AFEC0_NVIC_NUMBER );
      pmc_disable_periph_clk(ID_AFEC0);
    }
#endif
#if SAMV71_ADC_USE_ADC1 == TRUE
    if (&ADCD1 == adcp) {
      nvicDisableVector ( AFEC1_NVIC_NUMBER );
      pmc_disable_periph_clk(ID_AFEC1);
    }
#endif
  }
}

/**
 * @brief   Starts an ADC conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start_conversion(ADCDriver *adcp) {
  osalDbgCheck(adcp->grpp->channel_enabled != 0);
  if (CH_DBG_ENABLE_CHECKS != FALSE) {
    if((adcp->config->flags & ADC_FLAG_NUMERIC_CHANNEL_ORDER) == 0) {
      uint64_t seq = adcp->grpp->channel_sequence;
      uint16_t en = adcp->grpp->channel_enabled;
      for(int i = 0; i < 12; i++,en>>=1,seq>>=4) {
        //check the sequence position i is disabled or the channel at that
        //position is enabled as well.
        //The SAMV71 uses the channel enable register for both enabling the
        //channel in the sequence and for enabling the hardware setup/switchover
        //of the input. For the latter, the sequence is not used.
        int ch = seq & 0xf;
        osalDbgCheck((en & 1) == 0 ||
                           (adcp->grpp->channel_enabled & (1 << ch)) != 0);
      }
    }
    uint16_t en = adcp->grpp->channel_enabled;
    uint8_t count = 0;
    for(int i = 0; i < 12; i++,en>>=1) {
      //check the sequence position i is disabled or the channel at that
      //position is enabled as well.
      //The SAMV71 uses the channel enable register for both enabling the
      //channel in the sequence and for enabling the hardware setup/switchover
      //of the input. For the latter, the sequence is not used.
      if((en & 1) != 0)
          count++;
    }
    osalDbgCheck(adcp->grpp->num_channels == count);
  }

  adcp->device->AFEC_CHDR = ~adcp->grpp->channel_enabled;
  adcp->device->AFEC_CHER = adcp->grpp->channel_enabled;
  adcp->device->AFEC_SEQ1R = (adcp->grpp->channel_sequence >> 0) & 0xffffffffULL;
  adcp->device->AFEC_SEQ2R = (adcp->grpp->channel_sequence >> 32) & 0xffffffffULL;

  //other configuration:
  //DMA(if enabled)
  //per result interrupt(if not using DMA)
  //management information setup
  if((adcp->config->flags & ADC_FLAG_USE_DMA) == 0) {
    //non-dma path: one interrupt per sample
    adcp->device->AFEC_IER = AFEC_IER_DRDY;
    adcp->current_pos = 0;
    adcp->current_channel = 0;
    while ((adcp->grpp->channel_enabled & (1 << adcp->current_channel)) == 0) {
      adcp->current_channel++;
    }
  } else {
    //dma path

    size_t sample_count = adcp->grpp->num_channels * adcp->depth;
    size_t block1_count = (sample_count==1)?0:sample_count/2;
    size_t block2_count = sample_count-block1_count;

    adcp->dma_descriptors[0].XDMAC_MBR_NDA =
            (samv71_xdmac_linked_list_base_t*)&adcp->dma_descriptors[1];
    adcp->dma_descriptors[0].XDMAC_MBR_UBC = XDMAC_MBR_UBC_NDE |
                XDMAC_MBR_UBC_NDEN | XDMAC_MBR_UBC_NVIEW_NDV0 |
                XDMAC_MBR_UBC_UBLEN(block1_count);
    adcp->dma_descriptors[0].XDMAC_MBR_TA = adcp->samples;

    adcp->dma_descriptors[1].XDMAC_MBR_NDA =
            (samv71_xdmac_linked_list_base_t*)&adcp->dma_descriptors[0];
    adcp->dma_descriptors[1].XDMAC_MBR_UBC =
                XDMAC_MBR_UBC_NDEN | XDMAC_MBR_UBC_NVIEW_NDV0 |
                XDMAC_MBR_UBC_UBLEN(block2_count);
    adcp->dma_descriptors[1].XDMAC_MBR_TA = adcp->samples + block1_count;
    if(adcp->grpp->circular)
      adcp->dma_descriptors[1].XDMAC_MBR_UBC |= XDMAC_MBR_UBC_NDE;

    xdmacChannelSetSource(adcp->dma_channel,&(adcp->device->AFEC_LCDR));
    xdmacChannelSetDestination(adcp->dma_channel, adcp->samples);
    xdmacChannelSetMicroblockLength(adcp->dma_channel,0);
    uint32_t hwreq = 0xffffffffU;
#if SAMV71_ADC_USE_ADC0 == TRUE
    if (&ADCD0 == adcp) {
        hwreq = SAMV71_XDMAC_HWREQ_AFEC0;
    }
#endif
#if SAMV71_ADC_USE_ADC1 == TRUE
    if (&ADCD1 == adcp) {
        hwreq = SAMV71_XDMAC_HWREQ_AFEC1;
    }
#endif
    osalDbgCheck(hwreq != 0xffffffffU);

    xdmacChannelSetMode(adcp->dma_channel,
                        XDMAC_CC_TYPE_PER_TRAN |
                        XDMAC_CC_MBSIZE_SIXTEEN |
                        XDMAC_CC_SWREQ_HWR_CONNECTED |
                        XDMAC_CC_SAM_FIXED_AM |
                        XDMAC_CC_DAM_INCREMENTED_AM |
                        XDMAC_CC_DSYNC_PER2MEM |
                        XDMAC_CC_CSIZE_CHK_1 |
                        XDMAC_CC_DWIDTH_HALFWORD |
                        xdmacAutomaticInterfaceBits_CCReg(
                            &(adcp->device->AFEC_LCDR), adcp->samples) |
                        XDMAC_CC_PERID(hwreq));
    xdmacChannelSetBlockLength(adcp->dma_channel,0);
    xdmacChannelSetNextDescriptorMode(adcp->dma_channel,0);
    xdmacChannelSetStride(adcp->dma_channel,0);
    xdmacChannelSetSourceMicroblockStride(adcp->dma_channel,0);
    xdmacChannelSetDestinationMicroblockStride(adcp->dma_channel,0);
    xdmacChannelSetInterruptCauses(adcp->dma_channel, XDMAC_CIE_BIE |
                                   XDMAC_CIE_LIE | XDMAC_CIE_RBIE |
                                   XDMAC_CIE_WBIE | XDMAC_CIE_ROIE );
    xdmacChannelStartMasterTransfer(adcp->dma_channel,
                                    XDMAC_CNDC_NDVIEW_NDV0 |
                                    XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED,
                                    (block1_count==0)?
                                    (samv71_xdmac_linked_list_base_t*)&adcp->dma_descriptors[1]:
                                    (samv71_xdmac_linked_list_base_t*)&adcp->dma_descriptors[0],
                                    0);
  }

  //either directly trigger the conversion or enable the trigger signal
  if(adcp->grpp->trigger_selection >= 14) {
      //software trigger
      adcp->device->AFEC_MR &= ~AFEC_MR_TRGEN;
      adcp->device->AFEC_CR = AFEC_CR_START;
  } else {
      uint32_t mr = adcp->device->AFEC_MR;
      mr &= ~AFEC_MR_TRGSEL_Msk;
      mr |= adcp->grpp->trigger_selection;
      mr |= AFEC_MR_TRGEN_EN;
      adcp->device->AFEC_MR = mr;
  }

}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {

  (void)adcp;
  (void)adcp->config;
  (void)adcp->samples;
  (void)adcp->depth;
  (void)adcp->grpp;

  adcp->device->AFEC_MR &= ~AFEC_MR_TRGEN;

  if((adcp->config->flags & ADC_FLAG_USE_DMA) == 0) {
    //non-dma path
    adcp->device->AFEC_IDR = AFEC_IDR_DRDY;
  } else {
    //dma path
    xdmacChannelDisable(adcp->dma_channel);
  }

}

#endif /* HAL_USE_ADC == TRUE */

/** @} */
