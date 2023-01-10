/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 20/05/28 4:40p $
 * @brief    Implement CRC in CRC-CCITT mode and get the CRC checksum result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t CRC_SWResult(uint32_t mode, uint32_t polynom, uint32_t seed, uint8_t *string, uint32_t count, int8_t IsInput1sCOM, int8_t IsInputRVS, int8_t IsCRC1sCOM, int8_t IsCRCRVS);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable CRC clock */
    CLK_EnableModuleClock(CRC_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    const uint16_t u16CRCSrcPattern[] = {0x3231, 0x3433, 0x3635, 0x3837};
    uint32_t polynom = 0x8408;
    uint32_t seed = 0xFFFF;
    uint32_t IsWrite1sCOM = 1;
    uint32_t IsWriteRVS = 1;
    uint32_t IsCRC1sCOM = 1;
    uint32_t IsCRCRVS = 1;
    uint32_t i, u32HWChecksum = 0, u32SWChecksum = 0;
    uint32_t u32Attribute = 0;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------+\n");
    printf("|    CRC-16 Polynomial Mode Sample Code     |\n");
    printf("+-------------------------------------------+\n\n");

    printf("# Calculate [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38] CRC16 checksum value.\n");
    printf("    - Seed value is 0x%X             \n",seed);
    printf("    - Polynomial value is 0x%X       \n",polynom);
    printf("    - CPU write data length is 16-bit \n");
    printf("    - Checksum complement %s    \n", IsWrite1sCOM?"Enable":"Disable");
    printf("    - Checksum reverse %s       \n", IsWriteRVS?"Enable":"Disable");
    printf("    - Write data complement %s  \n", IsCRC1sCOM?"Enable":"Disable");
    printf("    - Write data reverse %s     \n", IsCRCRVS?"Enable":"Disable");

    u32SWChecksum = CRC_SWResult(CRC_16, polynom, seed, (uint8_t *)u16CRCSrcPattern, sizeof(u16CRCSrcPattern), IsWrite1sCOM, IsWriteRVS, IsCRC1sCOM, IsCRCRVS);

    printf("    - Checksum should be 0x%X        \n\n", u32SWChecksum);

    if(IsWrite1sCOM)
        u32Attribute |= CRC_WDATA_COM;
    if(IsWriteRVS)
        u32Attribute |= CRC_WDATA_RVS;
    if(IsCRC1sCOM)
        u32Attribute |= CRC_CHECKSUM_COM;
    if(IsCRCRVS)
        u32Attribute |= CRC_CHECKSUM_RVS;

    /* Configure CRC controller for CRC-CCITT CPU mode */
    CRC_Open(CRC_16, u32Attribute, seed, CRC_WDATA_16);

    CRC_SET_POLYNOMIAL(polynom);

    /* Start to execute CRC-CCITT operation */
    for(i = 0; i < sizeof(u16CRCSrcPattern) / sizeof(u16CRCSrcPattern[0]); i++)
    {
        CRC_WRITE_DATA((u16CRCSrcPattern[i]));
    }

    /* Get CRC-CCITT checksum value */
    u32HWChecksum = CRC_GetChecksum();

    printf("CRC H/W checksum is 0x%X ... %s.\n", u32HWChecksum, (u32HWChecksum == u32SWChecksum) ? "PASS" : "FAIL");

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC_MODULE);

    while(1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
