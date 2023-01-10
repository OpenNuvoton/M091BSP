/**************************************************************************//**
 * @file     dac.c
 * @version  V2.00
 * $Revision: 5 $
 * $Date: 20/06/10 10:54a $
 * @brief    M091 series DAC driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
*****************************************************************************/
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup DAC_Driver DAC Driver
  @{
*/

/** @addtogroup DAC_EXPORTED_FUNCTIONS DAC Exported Functions
  @{
*/

/**
  * @brief      This function make DAC module be ready to convert.
  * @param[in]  dac     Base address of DAC module.
  * @param[in] u32Ch    Not used in M091 Series DAC.
  * @param[in] u32TrgSrc Decides the trigger source. Valid values are:
  *                 - \ref DAC_WRITE_DAT_TRIGGER             :Write DAC_DAT trigger
  *                 - \ref DAC_SOFTWARE_TRIGGER              :Software trigger
  *                 - \ref DAC_LOW_LEVEL_TRIGGER             :STDAC pin low level trigger
  *                 - \ref DAC_HIGH_LEVEL_TRIGGER            :STDAC pin high level trigger
  *                 - \ref DAC_FALLING_EDGE_TRIGGER          :STDAC pin falling edge trigger
  *                 - \ref DAC_RISING_EDGE_TRIGGER           :STDAC pin rising edge trigger
  *                 - \ref DAC_TIMER0_TRIGGER                :Timer 0 trigger
  *                 - \ref DAC_TIMER1_TRIGGER                :Timer 1 trigger
  *                 - \ref DAC_TIMER2_TRIGGER                :Timer 2 trigger
  *                 - \ref DAC_TIMER3_TRIGGER                :Timer 3 trigger
  *                 - \ref DAC_TIMER4_TRIGGER                :Timer 4 trigger
  *                 - \ref DAC_TIMER5_TRIGGER                :Timer 5 trigger
  *                 - \ref DAC_BPWM1_TRIGGER                 :BPWM1 trigger
  * @return     None
  * @details    The DAC conversion can be started by writing DAC_DAT, software trigger or hardware trigger.
  *             When TRGEN (DAC_CTL[4]) is 0, the data conversion is started by writing DAC_DAT register.
  *             When TRGEN (DAC_CTL[4]) is 1, the data conversion is started by SWTRG (DAC_SWTRG[0]) is set to 1,
  *             external STDAC pin, timer event, or BPWM timer event.
  */
void DAC_Open(DAC_T *dac,
              uint32_t u32Ch,
              uint32_t u32TrgSrc)
{
    dac->CTL &= ~(DAC_CTL_ETRGSEL_Msk | DAC_CTL_TRGSEL_Msk | DAC_CTL_TRGEN_Msk);

    dac->CTL |= (u32TrgSrc | DAC_CTL_DACEN_Msk);
}

/**
  * @brief      Disable DAC analog power.
  * @param[in]  dac     Base address of DAC module.
  * @param[in]  u32Ch   Not used in M091 Series DAC.
  * @return     None
  * @details    Disable DAC analog power for saving power consumption.
  */
void DAC_Close(DAC_T *dac, uint32_t u32Ch)
{
    dac->CTL &= (~DAC_CTL_DACEN_Msk);
}

/**
  * @brief      Set delay time for DAC to become stable.
  * @param[in]  dac     Base address of DAC module.
  * @param[in]  u32Delay Decides the DAC conversion settling time, the range is from 0~(1023/PCLK*1000000) micro seconds.
  * @return     Real DAC conversion settling time (micro second).
  * @details    For example, DAC controller clock speed is 72MHz and DAC conversion setting time is 1 us, SETTLET (DAC_TCTL[9:0]) value must be greater than 0x48.
  * @note       User needs to write appropriate value to meet DAC conversion settling time base on PCLK (APB clock) speed.
  */
float DAC_SetDelayTime(DAC_T *dac, uint32_t u32Delay)
{
    SystemCoreClockUpdate();

    dac->TCTL = ((SystemCoreClock * u32Delay / 1000000) & 0x3FFF);

    return ((dac->TCTL) * 1000000 / SystemCoreClock);
}

/**
  * @brief      Set sample number per sine waveform.
  * @param[in]  dac     Base address of DAC module.
  * @param[in]  u32SampleNum Sample number
  *                 - \ref DAC_AUTO_SINE_0SAMPLES      :no sine waveform will be generated
  *                 - \ref DAC_AUTO_SINE_8SAMPLES      :8 samples per sine waveform
  *                 - \ref DAC_AUTO_SINE_16SAMPLES     :16 samples per sine waveform
  *                 - \ref DAC_AUTO_SINE_32SAMPLES     :32 samples per sine waveform
  * @return     None
  * @details    Set sample number per sine waveform
  * @note       Only DAC0 provides this function
  */
void DAC_SetAutoSineSampleNum(DAC_T *dac, uint32_t u32SampleNum)
{
    u32SampleNum &= 0x03;
    dac->ADGCTL = ((dac->ADGCTL & ~DAC_ADGCTL_SAMPSEL_Msk) | (u32SampleNum << DAC_ADGCTL_SAMPSEL_Pos));
}

/**
  * @brief      Set sample content for auto sine waveform.
  * @param[in]  dac     Base address of DAC module.
  * @param[in]  pu16SampleBase Base address of sample content
  * @param[in]  u32SampleNum Sample number
  * @return     None
  * @details    Set sample content for auto sine waveform in Manchester application
  * @note       Only DAC0 provides this function
  */
void DAC_SetAutoSineSampleContent(DAC_T *dac,
                                  uint16_t *pu16SampleBase,
                                  uint32_t u32SampleNum)
{
    uint32_t i;

    if (u32SampleNum > 32)
        u32SampleNum = 32;

    for (i=0; i< u32SampleNum; i++)
    {
        dac->ADCTL[i] = *pu16SampleBase++;
    }
}

/**
  * @brief      Set sine waveform frequency.
  * @param[in]  dac     Base address of DAC module.
  * @param[in]  u32SineFreq The expected frequency for sine waveform in Hz.
  * @return     None
  * @details    Set auto sine waveform frequency in Hz
  * @note       Only DAC0 provides this function.
  * @note       Sine wave frequency is related to sample numer per sine waveform.
  *             Therefore, sample number must be given before to set frequency.
  */
void DAC_SetAutoSineFreq(DAC_T *dac, uint32_t u32SineFreq)
{
    uint32_t u32Clk, u32SampleNum, u32SettleTime;

    /* get PCLK1 clock freq (DAC clock from PCLK1) */
    u32Clk = CLK_GetPCLK1Freq();

    /* get sample numner per sine waveform */
    u32SampleNum = (dac->ADGCTL&DAC_ADGCTL_SAMPSEL_Msk)>>DAC_ADGCTL_SAMPSEL_Pos;
    switch (u32SampleNum)
    {
    case 0:
        u32SampleNum = 0x00;
        break;
    case 1:
        u32SampleNum = 0x08;
        break;
    case 2:
        u32SampleNum = 0x10;
        break;
    case 3:
    default:
        u32SampleNum = 0x20;
        break;
    }
    if (!u32SampleNum)
        DAC_SET_SETTLE_TIME(dac, 0x3FF);

    u32SettleTime = u32Clk/(u32SampleNum*u32SineFreq);
    DAC_SET_SETTLE_TIME(dac, u32SettleTime);
}


/*@}*/ /* end of group DAC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group DAC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
