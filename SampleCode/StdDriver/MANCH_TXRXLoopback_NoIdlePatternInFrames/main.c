/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/06/10 2:23p $
 * @brief    Some standard doesn't contain IDLE pattern between two frames. Since
 *           Manchester TX hardware generates IDLE pattern, TX must be replaced
 *           by pure software. As for RX, it also needs IDLE pattern for decoding.
 *           So, RX will be inserted IDLE pattern between two frames.
 *
 *           The method is that pseudo RX (TMR0_EXT) will be duplicated to another
 *           GPIO port (RXm) and the first PREAMBLE byte is replaced by IDLE
 *           pattern. Then, connect RXm to the real Manchester RX port for test.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"
#include "manchester_decode.h"
#include "manchester_encode.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t u8ManchTxBuf[64];
uint8_t g_u8ManchRxBuf_ReArranged[2][64];

/*---------------------------------------------------------------------------------------------------------*/
/* Function declaration                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

void SYS_Init(void)
{
    /*--------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                    */
    /*--------------------------------------------------------------------------------------*/
    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable MANCH module clock */
    CLK_EnableModuleClock(MANCH_MODULE);

    /* Enable DAC module clock */
    CLK_EnableModuleClock(DAC01_MODULE);

    /* Enable TIMER peripheral clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);

    /* Select TIMER module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HIRC, 0);

    /* Enable CRC clock */
    CLK_EnableModuleClock(CRC_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*--------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                              */
    /*--------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB15MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB15MFP_TM0_EXT);

    /* Set PB multi-function pins for MANCH RXD=PB.6 and TXD=PB.7 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk)) |
                    (SYS_GPB_MFPL_PB7MFP_MANCH_TXD | SYS_GPB_MFPL_PB6MFP_MANCH_RXD);

    /* Set PA multi-function pins for DAC voltage output */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_DAC0_OUT;

    /* Set PA.0 to input mode */
    PA->MODE &= ~(GPIO_MODE_MODE0_Msk) ;
}

void UART0_Init()
{
    /*--------------------------------------------------------------------------------------*/
    /* Init UART                                                                            */
    /*--------------------------------------------------------------------------------------*/
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t u8Id = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+--------------------------------------------------------------------------------------+\n");
    printf("|                               MANCH Driver Sample Code                               |\n");
    printf("+--------------------------------------------------------------------------------------+\n");
    printf("  This sample code sends MANCH encoded data from PA2 (TX) by pure software. \n");
    printf("  Please connect PA2 to PB15 (pseudo RX, TMR0_EXT) for loopback test. In addition,\n");
    printf("  the modified RX (RXm) which one preamble byte is replaced by IDLE pattern \n");
    printf("  is generated at PB5. Please also connect PB5 to the real RX (PB.6) at the same time.\n\n");

    printf("  Press any key when to be ready.\n\n");
    getchar();

    for(i=0; i<4; i++)
    {
        u8ManchTxBuf[i] = 0x7E;
    }
    for(i=4; i<63; i++)
    {
        u8ManchTxBuf[i] = i-4;
    }

    add_crc(u8ManchTxBuf);

    u8ManchTxBuf[63] = 0x7E;

    Encode_Buf_Fill(u8ManchTxBuf, 0);
    Encode_Buf_Fill(u8ManchTxBuf, 1);

    GPIO_SetMode(PB, BIT5, GPIO_MODE_OUTPUT);

    Manchester_Encode_Init();
    Manchester_Decode_Init();

    while(1)
    {
        if(g_u8ManchTxDoneFlag == 1)
        {
            g_u8ManchTxDoneFlag = 0;

            for(i=0; i<4; i++)
            {
                u8ManchTxBuf[i] = 0x7E;
            }
            for(i=4; i<63; i++)
            {
                u8ManchTxBuf[i] = j;
                //u8ManchTxBuf[i] = 0x55;
            }
            u8ManchTxBuf[63] = 0x7E;

            add_crc(u8ManchTxBuf);
            j++;

            Encode_Buf_Fill(u8ManchTxBuf, u8Id);
            u8Id ^= 1;
        }

        if(g_u8ManchRxDoneFlag == 1)
        {
            g_u8ManchRxDoneFlag = 0;

            printf("\n\nRX: ");
            for(i=0; i<63; i++)
            {
                printf("0x%x, ", g_u8ManchRxBuf[g_u8ManchRxId^1][i]);
            }
            printf("\n\n");

            if (!check_crc(&g_u8ManchRxBuf[g_u8ManchRxId^1][0]))
                printf("-> CRC: OK \n");
            else
                printf("-> CRC: Fail \n");

#ifdef OPT_REARRANGE_RX
            printf("\n\nRX_re: ");
            rearrange_rx(&g_u8ManchRxBuf[g_u8ManchRxId^1][0], &g_u8ManchRxBuf_ReArranged[g_u8ManchRxId^1][0]);
            for(i=0; i<64; i++)
            {
                printf("0x%x, ", g_u8ManchRxBuf_ReArranged[g_u8ManchRxId^1][i]);
            }
            printf("\n\n");

            if (!check_crc(&g_u8ManchRxBuf[g_u8ManchRxId^1][0]))
            {
                printf("-> CRC: OK \n");
            }
            else
                printf("-> CRC: Fail \n");
#endif

        }
    }
}


/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
