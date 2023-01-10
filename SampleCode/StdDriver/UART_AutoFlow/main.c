/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive data using auto flow control.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK   FREQ_96MHZ

#define RXBUFSIZE 256

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t g_i32pointer = 0;
uint8_t g_u8RecData[RXBUFSIZE] = {0};

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest(void);
void AutoFlow_FunctionTxTest(void);
void AutoFlow_FunctionRxTest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB multi-function pins for UART0 CTS and RTS */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB15MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB14MFP_UART0_nRTS | SYS_GPB_MFPH_PB15MFP_UART0_nCTS);

    /* Lock protected registers */
    SYS_LockReg();

}
/*---------------------------------------------------------------------------------------------------------*/
/*                                                 Init UART0                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for Test */
    UART0_Init();


    /*---------------------------------------------------------------------------------------------------------*/
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------+\n");
    printf("|  Auto-Flow function test |\n");
    printf("+--------------------------+\n");

    /* UART auto flow sample function */
    AutoFlow_FunctionTest();

    printf("\nUART Sample Program End\n");

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest(void)
{
    uint8_t u8Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|- UART0_TXD(PB.13) <==> UART0_RXD(PB.12)- |Slave| |\n");
    printf("| |      |-UART0_nCTS(PB.15) <==> UART0_nRTS(PB.14)-|     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test                              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave. Slave will check if received data is correct |\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    u8Item = getchar();

    if(u8Item == '0')
        AutoFlow_FunctionTxTest();
    else
        AutoFlow_FunctionRxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Master)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTxTest(void)
{
    uint32_t u32i;

    /* Enable RTS and CTS autoflow control */
    UART_EnableFlowCtrl(UART0);

    /* Send 1k bytes data */
    for(u32i = 0; u32i < RXBUFSIZE; u32i++)
    {
        /* Send 1 byte data */
        UART_WRITE(UART0, u32i & 0xFF);

        /* Wait if Tx FIFO is full */
        while(UART_IS_TX_FULL(UART0));
    }

    printf("\n Transmit Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Slave)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionRxTest(void)
{
    uint32_t u32i;

    /* Enable RTS and CTS autoflow control */
    UART_EnableFlowCtrl(UART0);

    /* Set Timeout time 0x3E bit-time and time-out counter enable */
    UART_SetTimeoutCnt(UART0, 0x3E);

    /* Enable RDA and RTO Interrupt */
    NVIC_EnableIRQ(UART0_IRQn);
    UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    printf("\n Starting to receive data...\n");

    /* Wait for receive 1k bytes data */
    while(g_i32pointer < RXBUFSIZE);

    /* Compare Data */
    for(u32i = 0; u32i < RXBUFSIZE; u32i++)
    {
        if(g_u8RecData[u32i] != (u32i & 0xFF))
        {
            printf("Compare Data Failed\n");
            while(1);
        }
    }
    printf("\n Receive OK & Check OK\n");

    /* Disable RDA and RTO Interrupt */
    UART_DisableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint8_t u8InChar = 0xFF;

    /* Rx Ready or Time-out INT */
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))
    {
        /* Read data until RX FIFO is empty */
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            u8InChar = UART_READ(UART0);
            g_u8RecData[g_i32pointer++] = u8InChar;
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}



/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
