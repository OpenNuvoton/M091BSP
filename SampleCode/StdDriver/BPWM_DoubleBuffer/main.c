/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/06/09 10:14a $
 * @brief
 *           Change duty cycle and period of output waveform by BPWM Double Buffer function.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       BPWM1 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM1 interrupt event
 */
void BPWM_IRQHandler(void)
{
    static int i8Toggle = 0;

    // Update BPWM1 channel 0 period and duty
    if (i8Toggle == 0)
    {
        BPWM_SET_CNR(BPWM1, 0, 99);
        BPWM_SET_CMR(BPWM1, 0, 39);
    }
    else
    {
        BPWM_SET_CNR(BPWM1, 0, 399);
        BPWM_SET_CMR(BPWM1, 0, 199);
    }

    i8Toggle ^= 1;
    // Clear channel 0 period interrupt flag
    BPWM_ClearPeriodIntFlag(BPWM1, 0);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable BPWM1 clock source */
    CLK_EnableModuleClock(BPWM1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PF.3 multi-function pin for BPWM1 channel 0 */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_BPWM1_CH0;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM1 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM1 channel 0(PF.3)\n");
    printf("\nUse double buffer feature.\n");

    /*
        BPWM1 channel 0 waveform of this sample shown below:

        |<-        CNR + 1  clk     ->|  CNR + 1 = 399 + 1 CLKs
                       |<-CMR+1 clk ->|  CMR + 1 = 199 + 1 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                               |<CMR+1>|  CMR + 1 = 39 + 1 CLKs

      __                ______________          _______
        |______200_____|     200      |____60__|   40  |_____BPWM waveform

    */

    /*
      Configure BPWM1 channel 0 init period and duty.
      Period is PCLK / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (1 * (399 + 1)) = 120000 Hz
      Duty ratio = (200) / (399 + 1) = 50%
    */
    // BPWM1 channel 0 frequency is 120000Hz, duty 50%,
    BPWM_ConfigOutputChannel(BPWM1, 0, 120000, 50);

    // Enable output of BPWM1 channel 0
    BPWM_EnableOutput(BPWM1, BPWM_CH_0_MASK);

    // Enable BPWM1 channel 0 period interrupt, use channel 0 to measure time.
    BPWM_EnablePeriodInt(BPWM1, 0, 0);
    NVIC_EnableIRQ(BPWM_IRQn);

    // Start
    BPWM_Start(BPWM1, BPWM_CH_0_MASK);

    while (1);

}


/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
