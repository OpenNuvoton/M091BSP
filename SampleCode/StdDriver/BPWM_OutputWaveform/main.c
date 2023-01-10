/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/06/08 7:28p $
 * @brief    Demonstrate how to use BPWM counter output waveform.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable BPWM1 module clock */
    CLK_EnableModuleClock(BPWM1_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set multi-function pins for BPWM1 Channel0~5 */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_BPWM1_CH0;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_BPWM1_CH1;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA12MFP_Msk)) | SYS_GPA_MFPH_PA12MFP_BPWM1_CH2;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA13MFP_Msk)) | SYS_GPA_MFPH_PA13MFP_BPWM1_CH3;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA14MFP_Msk)) | SYS_GPA_MFPH_PA14MFP_BPWM1_CH4;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA15MFP_Msk)) | SYS_GPA_MFPH_PA15MFP_BPWM1_CH5;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with BPWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM1_CH0(PF.3), BPWM1_CH1(PF.2), BPWM1_CH2(PA.12), BPWM1_CH3(PA.13), BPWM1_CH4(PA.14), BPWM1_CH5(PA.15)\n");


    /* BPWM1 channel 0~5 frequency and duty configuration are as follows */
    BPWM_ConfigOutputChannel(BPWM1, 0, 30000, 10);
    BPWM_ConfigOutputChannel(BPWM1, 1, 30000, 20);
    BPWM_ConfigOutputChannel(BPWM1, 2, 30000, 30);
    BPWM_ConfigOutputChannel(BPWM1, 3, 30000, 40);
    BPWM_ConfigOutputChannel(BPWM1, 4, 30000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 5, 30000, 60);

    /* Enable output of BPWM1 channel 0~5 */
    BPWM_EnableOutput(BPWM1, 0x3F);

    /* Start BPWM1 counter */
    BPWM_Start(BPWM1, 0x3F);

    printf("Press any key to stop.\n");
    getchar();

    /* Start BPWM1 counter */
    BPWM_ForceStop(BPWM1, 0x3F);

    printf("Done.");

    while (1);

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
