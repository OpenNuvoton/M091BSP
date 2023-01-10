/**************************************************************************//**
 * @file     manch.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/11/18 3:17p $
 * @brief    M091 series MANCH driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
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

/** @addtogroup MANCH_Driver MANCH Driver
  @{
*/

/** @addtogroup MANCH_EXPORTED_FUNCTIONS MANCH Exported Functions
  @{
*/

/**
  * @brief      Open MANCH operation mode and specify bus clock frequency
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32BusFreq The expected frequency of MANCH bus clock in Hz.
  * @return     Actual frequency of MANCH bus clock.
  * @details    This function is used to configure MANCH in specified operation mode and bus clock frequency.
  *             If MANCH cannot work in targeted frequency, the closest frequency will be chosen and returned.
  * @note       TBITNUM and RBITNUM dividers are set to 0x64.
  )* @note       If u32BusFreq = 0, BITDIV divider will be set to the maximum value (0xFFF).
  * @note       If u32BusFreq >= PCLK0/100, BITDIV divider will be set to 0x03.
  */
uint32_t MANCH_Open(MANCH_T *manch, uint32_t u32BusFreq)
{
    uint32_t u32Clk, u32BitDivider;

    /* get PCLK0 clock freq (MANCH clock from PCLK0) */
    u32Clk = CLK_GetPCLK0Freq();

    /* Set MANCH to NONE mode before to set related format */
    MANCH_SetMode(manch, MANCH_NONE);

    /* given TBITNUM and RBITNUM default values */
    MANCH_SetTXBitNum(manch, MANCH_TBITNUM_DEFAULT);
    MANCH_SetRXBitNum(manch, MANCH_RBITNUM_DEFAULT);

    /* set BITDIV value */
    u32Clk /= MANCH_TBITNUM_DEFAULT;
    if(!u32BusFreq)
    {
        u32BitDivider = 0x1000;
    }
    else if(u32BusFreq > u32Clk)
    {
        u32BitDivider = 0x4;
    }
    else
    {
        u32BitDivider = u32Clk/u32BusFreq;
        u32BitDivider--;
    }
    MANCH_SetBitClockDiv(manch, u32BitDivider);
    u32BitDivider++;
    return (u32BitDivider*MANCH_TBITNUM_DEFAULT);
}

/**
  * @brief      Disable MANCH controller.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @return     None
  * @details    This function will reset MANCH controller.
  */
void MANCH_Close(MANCH_T *manch)
{
    if(manch == MANCH)
    {
        /* Reset MANCH */
        SYS->IPRST2 |= SYS_IPRST2_MANCHRST_Msk;
        SYS->IPRST2 &= ~SYS_IPRST2_MANCHRST_Msk;
    }
}

/**
  * @brief      Set the 1st stage bit clock divider.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32ClkDiv   The 1st stage bit clock divider
  * @return     None
  * @details    Set MANCH 1st stage bit clock divider. This is the 1st stage 12-bit width clock divider from
  *             engine clock. After setting 1st stage divider, 2nd stage TX/RX bit divider
  *             must also be set at the same time.
  * @note       The real bit clock divider is given by (u32ClkDiv + 1).
  */
void MANCH_SetBitClockDiv(MANCH_T *manch, uint32_t u32ClkDiv)
{
    manch->CTL = ((manch->CTL)&(~MANCH_CTL_BITDIV_Msk)| ((u32ClkDiv&
                  (MANCH_CTL_BITDIV_Msk>>MANCH_CTL_BITDIV_Pos))<<MANCH_CTL_BITDIV_Pos));
}

/**
  * @brief      Get the 1st stage bit clock divider.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @return     The 1st stage bit clock divider
  * @details    Get MANCH 1st stage bit clock divider. This is the 1st stage clock divider from
  *             engine clock.
  */
uint32_t MANCH_GetBitClockDiv(MANCH_T *manch)
{
    return ((manch->CTL & MANCH_CTL_BITDIV_Msk)>>MANCH_CTL_BITDIV_Pos);
}

/**
  * @brief      Set deglitch clock divider.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32ClkDiv   Deglitch clock divider
  * @return     None
  * @details    After setting MANCH 3-bit width deglitch clock divider, MANCH_SetDegWidth() must be called to select desired deglitch settings.
  */
void MANCH_SetDegClockDiv(MANCH_T *manch, uint32_t u32ClkDiv)
{
    manch->CTL = ((manch->CTL)&(~MANCH_CTL_DEGDIV_Msk)| ((u32ClkDiv&
                  (MANCH_CTL_DEGDIV_Msk>>MANCH_CTL_DEGDIV_Pos))<<MANCH_CTL_DEGDIV_Pos));
}

/**
  * @brief      Get deglitch clock divider.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @return     Deglitch clock divider
  * @details    Get current MANCH deglitch clock divider.
  */
uint32_t MANCH_GetDegClockDiv(MANCH_T *manch)
{
    return ((manch->CTL & MANCH_CTL_DEGDIV_Msk)>>MANCH_CTL_DEGDIV_Pos);
}

/**
  * @brief      Set deglitch width.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32DegWidth Deglitch width
  *                 - \ref MANCH_DEGLITCH_DIS
  *                 - \ref MANCH_DEGLITCH_025
  *                 - \ref MANCH_DEGLITCH_050
  *                 - \ref MANCH_DEGLITCH_075
  *                 - \ref MANCH_DEGLITCH_100
  *                 - \ref MANCH_DEGLITCH_125
  * @return     None
  * @details    After setting MANCH deglitch clock divider, MANCH_SetDegWidth() must be called to select desired deglitch settings.
  */
void MANCH_SetDegWidth(MANCH_T *manch, uint32_t u32DegWidth)
{
    manch->CTL = ((manch->CTL)&(~MANCH_CTL_DEGSEL_Msk)| ((u32DegWidth&
                  (MANCH_CTL_DEGSEL_Msk>>MANCH_CTL_DEGSEL_Pos))<<MANCH_CTL_DEGSEL_Pos));
}

/**
  * @brief      Get deglitch width.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @return     Deglitch width
  * @details    Get current MANCH deglitch width.
  */
uint32_t MANCH_GetDegWidth(MANCH_T *manch)
{
    return ((manch->CTL & MANCH_CTL_DEGSEL_Msk)>>MANCH_CTL_DEGSEL_Pos);
}

/**
  * @brief      Set Preamble pattern.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32Preamble Preamble pattern (8-bit)
  * @return     None
  * @details    Based on selected Manchester mode, this function sets MANCH 8-bit width Preamble pattern.
  */
void MANCH_SetPreamble(MANCH_T *manch, uint32_t u32Preamble)
{
    manch->PREAM = ((manch->PREAM)&(~MANCH_PREAM_PREAMBLE_Msk)|((u32Preamble&
                    (MANCH_PREAM_PREAMBLE_Msk>>MANCH_PREAM_PREAMBLE_Pos))<<MANCH_PREAM_PREAMBLE_Pos));
}

/**
  * @brief      Set Preamble number.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32PreambleNum Preamble number
  * @return     None
  * @details    Based on selected Manchester mode, this function sets MANCH 5-bit width Preamble number.
  */
void MANCH_SetPreambleNum(MANCH_T *manch, uint32_t u32PreambleNum)
{
    manch->PREAM = ((manch->PREAM)&(~MANCH_PREAM_PRENUM_Msk))|((u32PreambleNum&
                   (MANCH_PREAM_PRENUM_Msk>>MANCH_PREAM_PRENUM_Pos))<<MANCH_PREAM_PRENUM_Pos);
}

/**
  * @brief      Set Idle pattern.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32Idle Idle pattern (8-bit)
  * @return     None
  * @details    Based on selected Manchester mode, this function sets MANCH 8-bit width Idle pattern.
  */
void MANCH_SetIdle(MANCH_T *manch, uint32_t u32Idle)
{
    manch->PREAM = ((manch->PREAM)&(~MANCH_PREAM_IDLEPAT_Msk))|((u32Idle&
                   (MANCH_PREAM_IDLEPAT_Msk>>MANCH_PREAM_IDLEPAT_Pos))<<MANCH_PREAM_IDLEPAT_Pos);
}

/**
  * @brief      Set MANCH frame byte number.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32FrameNum Frame byte number
  * @return     None
  * @details    Based on selected Manchester mode, this function sets MANCH 8-bit width frame byte number.
  *             If u32FrameNum is given to 0x00, it means that the frame number is 256 bytes.
  */
void MANCH_SetFrameNum(MANCH_T *manch, uint32_t u32FrameNum)
{
    manch->PREAM = ((manch->PREAM)&(~MANCH_PREAM_FMTNUM_Msk))|((u32FrameNum&(MANCH_PREAM_FMTNUM_Msk>>MANCH_PREAM_FMTNUM_Pos))<<MANCH_PREAM_FMTNUM_Pos);
}

/**
  * @brief      Set MANCH mode.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32Mode
  *                 - \ref MANCH_NONE
  *                 - \ref MANCH_MODE1
  *                 - \ref MANCH_MODE2
  *                 - \ref MANCH_MODE_OTHER
  * @return     None
  * @details    Set desired MANCH modulation mode.
  * @note       If MANCH mode is not in MANCH_NONE mode, and other related format setting is changed. The MANCH mode must be given
                to MANCH_NONE first and then set to desired mode. For example, when MANCH mode is in MANCH_MODE1, the deglitch width
                is changed from MANCH_DEGLITCH_025 to MANCH_DEGLITCH_050. We must set MANCH to MANCH_NONE mode first and then change
                to MANCH_MODE1 later.
  */
void MANCH_SetMode(MANCH_T *manch, uint32_t u32Mode)
{
    manch->CTL = (manch->CTL & ~MANCH_CTL_MODESEL_Msk)|((u32Mode&
                 (MANCH_CTL_MODESEL_Msk>>MANCH_CTL_MODESEL_Pos))<<MANCH_CTL_MODESEL_Pos);
}

/**
  * @brief      Set TX bit clock divider number
  * @param[in]  manch   The pointer of the specified MANCH module..
  * @param[in]  u32TXBitNum TX bit clock divider number
  * @return     None
  * @details    After setting MANCH 1st stage bit clock divider. This is the 2nd stage 8-bit width clock divider
  *             for TX. Both 2nd stage TX/RX bit dividers are suggested to be the same.
  */
void MANCH_SetTXBitNum(MANCH_T *manch, uint32_t u32TXBitNum)
{
    manch->BITCNT &= ~MANCH_BITCNT_TBITNUM_Msk;
    manch->BITCNT |= ((u32TXBitNum&(MANCH_BITCNT_TBITNUM_Msk>>MANCH_BITCNT_TBITNUM_Pos))
                      <<MANCH_BITCNT_TBITNUM_Pos);
}

/**
  * @brief      Get TX bit clock divider number.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @return     TX bit clock divider
  * @details    Get 2nd stage TX bit clock divider number.
  */
uint32_t MANCH_GetTXBitNum(MANCH_T *manch)
{
    return ((manch->BITCNT & MANCH_BITCNT_TBITNUM_Msk)>> MANCH_BITCNT_TBITNUM_Pos);
}

/**
  * @brief      Set RX bit clock divider number.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32RXBitNum RX bit clock divider number
  * @return     None
  * @details    After setting MANCH 1st stage bit clock divider. This is the 2nd stage 8-bit width clock divider
  *             for RX. Both 2nd stage TX/RX bit dividers are suggested to be the same.
  */
void MANCH_SetRXBitNum(MANCH_T *manch, uint32_t u32RXBitNum)
{
    manch->BITCNT &= ~MANCH_BITCNT_RBITNUM_Msk;
    manch->BITCNT |= ((u32RXBitNum&(MANCH_BITCNT_RBITNUM_Msk>>MANCH_BITCNT_RBITNUM_Pos))
                      <<MANCH_BITCNT_RBITNUM_Pos);
}

/**
  * @brief      Get RX bit clock divider number.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @return     RX bit clock divider number
  * @details    Get 2nd stage RX bit clock divider number.
  */
uint32_t MANCH_GetRXBitNum(MANCH_T *manch)
{
    return ((manch->BITCNT & MANCH_BITCNT_RBITNUM_Msk)>> MANCH_BITCNT_RBITNUM_Pos);
}

/**
  * @brief      Get current RX bit clock divider number.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @return     current RX bit clock divider number
  * @details    Get current RX bit clock divider number.
  */
uint32_t MANCH_GetCurrentRXBitNum(MANCH_T *manch)
{
    return ((manch->BITCNT & MANCH_BITCNT_CRBITNUM_Msk)>> MANCH_BITCNT_CRBITNUM_Pos);
}

/**
  * @brief      Set RX bit error tolerance number.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @param[in]  u32RXBitErrTolNum
  *                 - \ref MANCH_BERRTN_1_4
  *                 - \ref MANCH_BERRTN_1_8
  *                 - \ref MANCH_BERRTN_1_16
  *                 - \ref MANCH_BERRTN_1_32
  *                 - \ref MANCH_BERRTN_1_64
  * @return     None
  * @details    Set RX fraction bit error tolerance number.
  */
void MANCH_SetRXBitTolNum(MANCH_T *manch, uint32_t u32RXBitErrTolNum)
{
    manch->BITCNT &= ~MANCH_BITCNT_RBERRTN_Msk;
    manch->BITCNT |= ((u32RXBitErrTolNum&(MANCH_BITCNT_RBERRTN_Msk>>MANCH_BITCNT_RBERRTN_Pos))
                      <<MANCH_BITCNT_RBERRTN_Pos);
}

/**
  * @brief      Get RX bit error tolerance number.
  * @param[in]  manch   The pointer of the specified MANCH module.
  * @return     u32RXBitErrTolNum
  * @details    Get the error tolerance number of RX fraction bit.
  */
uint32_t MANCH_GetRXBitTolNum(MANCH_T *manch)
{
    return ((manch->BITCNT & MANCH_BITCNT_RBERRTN_Msk)>> MANCH_BITCNT_RBERRTN_Pos);
}


/*@}*/ /* end of group MANCH_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MANCH_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
