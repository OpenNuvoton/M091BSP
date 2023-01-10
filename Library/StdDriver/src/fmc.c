/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 20/06/11 7:54a $
 * @brief    M091 series FMC driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>

#include "NuMicro.h"



/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FMC_Driver FMC Driver
  @{
*/


/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

int32_t  g_FMC_i32ErrCode;

/**
  * @brief Disable FMC ISP function.
  * @return None
  */
void FMC_Close(void)
{
FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief Execute FMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 512 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 512 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval   0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Erase failed or erase time-out
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;

    if (u32PageAddr == FMC_SPROM_BASE)
    {
        return FMC_Erase_SPROM();
    }
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_ERASE;
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
  * @brief Execute FMC_ISPCMD_PAGE_ERASE command to erase SPROM. The page size is 512 bytes.
  * @return   SPROM page erase success or not.
  * @retval   0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Erase failed or erase time-out
  */
int32_t FMC_Erase_SPROM(void)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = FMC_SPROM_BASE;
    FMC->ISPDAT = 0x0055AA03UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_ERASE;
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;

    }
    return 0;
}

/**
  * @brief Get the current boot source.
  * @return The current boot source.
  * @retval   0  Is boot from APROM.
  * @retval   1  Is boot from LDROM.
  */
int32_t FMC_GetBootSource (void)
{
    int32_t  ret = 0;

    if (FMC->ISPCTL & FMC_ISPCTL_BS_Msk)
    {
        ret = 1;
    }

    return ret;
}


/**
  * @brief Enable FMC ISP function
  * @return None
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief Execute FMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  *          Return 0xFFFFFFFF if read failed.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_Read(uint32_t u32Addr)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_READ;

    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return FMC->ISPDAT;

}


/**
  * @brief    Get the base address of Data Flash if enabled.
  * @retval   The base address of Data Flash
  */
uint32_t FMC_ReadDataFlashBaseAddr(void)
{
    return FMC->DFBA;
}

/**
  * @brief      Set boot source from LDROM or APROM after next software reset
  * @param[in]  i32BootSrc
  *                1: Boot from LDROM
  *                0: Boot from APROM
  * @return    None
  * @details   This function is used to switch APROM boot or LDROM boot. User need to call
  *            FMC_SetBootSource to select boot source first, then use CPU reset or
  *            System Reset Request to reset system.
  */
void FMC_SetBootSource(int32_t i32BootSrc)
{
    if(i32BootSrc)
    {
        FMC->ISPCTL |= FMC_ISPCTL_BS_Msk; /* Boot from LDROM */
    }
    else
    {
        FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk;/* Boot from APROM */
    }
}

/**
  * @brief Execute ISP FMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return   0   Success
  * @return   -1  Program Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out

  */
int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_WRITE;

    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
  * @brief Execute FMC_ISPCMD_READ command to read User Configuration.
  * @param[out]  u32Config A three-word array.
  *              u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
  * @param[in] u32Count Available word count in u32Config.
  * @return Success or not.
  * @retval   0  Success.
  * @retval   -1  Read failed
  * @retval   -2  Invalid parameter.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read failed
  *           -2  Invalid parameter
  */
int32_t FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count)
{
    int32_t   ret = 0;

    u32Config[0] = FMC_Read(FMC_CONFIG_BASE);

    if (g_FMC_i32ErrCode != 0)
        return g_FMC_i32ErrCode;

    if (u32Count > 3UL)
    {
        ret = -2;
    }
    else
    {
        if(u32Count > 1UL)
        {
            u32Config[1] = FMC_Read(FMC_CONFIG_BASE+4UL);

            if (g_FMC_i32ErrCode != 0)
                return g_FMC_i32ErrCode;

        }
        if(u32Count > 2UL)
        {
            u32Config[2] = FMC_Read(FMC_CONFIG_BASE+8UL);
            if (g_FMC_i32ErrCode != 0)
                return g_FMC_i32ErrCode;
        }
    }
    return ret;
}

/**
 * @brief Execute ISP commands to erase then write User Configuration.
 * @param[in] u32Config   A two-word array.
 *            u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
 * @param[in] u32Count  Always be 2 in this BSP.
 * @return Success or not.
 * @retval   0  Success.
 * @retval   -1  Erase/program/read/verify failed
 *
 * @note     Global error code g_FMC_i32ErrCode
 *           < 0  Errors caused by erase/program/read failed or time-out
 */
int32_t FMC_WriteConfig(uint32_t u32Config[], uint32_t u32Count)
{
    uint32_t i;

    FMC_ENABLE_CFG_UPDATE();

    if (FMC_Erase(FMC_CONFIG_BASE) != 0)
        return -1;

    if ((FMC_Read(FMC_CONFIG_BASE) != 0xFFFFFFFF) || (FMC_Read(FMC_CONFIG_BASE+4) != 0xFFFFFFFF) ||
            (FMC_Read(FMC_CONFIG_BASE+8) != 0xFFFFFF5A))
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if (g_FMC_i32ErrCode != 0)
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    for (i = 0u; i < u32Count; i++)
    {
        if (FMC_Write(FMC_CONFIG_BASE+i*4UL, u32Config[i]) != 0)
        {
            FMC_DISABLE_CFG_UPDATE();
            return -1;
        }

        if (FMC_Read(FMC_CONFIG_BASE+i*4UL) != u32Config[i])
        {
            FMC_DISABLE_CFG_UPDATE();
            return -1;
        }
        if (g_FMC_i32ErrCode != 0)
        {
            FMC_DISABLE_CFG_UPDATE();
            return -1;
        }
    }

    FMC_DISABLE_CFG_UPDATE();

    return 0;
}

/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
  * @return Success or not.
  * @retval   0           Success.
  * @retval   0xFFFFFFFF  Invalid parameter or command failed.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Run/Read check sum time-out failed
  *           -2  u32addr or u32count must be aligned with 512
  */
uint32_t  FMC_GetChkSum(uint32_t u32addr, uint32_t u32count)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;

    if ((u32addr % 512UL) || (u32count % 512UL))
    {
        g_FMC_i32ErrCode = -2;
        return 0xFFFFFFFF;

    }
    FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
    FMC->ISPADDR = u32addr;
    FMC->ISPDAT  = u32count;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_CHKSUM;
    while ((--tout > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
    FMC->ISPADDR    = u32addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_CHKSUM;
    while ((--tout > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }

    return FMC->ISPDAT;
}


/*@}*/ /* end of group FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group FMC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


