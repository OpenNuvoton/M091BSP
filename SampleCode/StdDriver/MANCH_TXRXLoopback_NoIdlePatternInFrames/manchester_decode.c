/**************************************************************************//**
 * @file     manchester_decode.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/06/10 2:23p $
 * @brief
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "manchester_decode.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

uint8_t g_u8ManchRxBuf[2][64];
uint8_t g_u8ManchRxId = 0;
uint8_t g_u8ManchRxDoneFlag = 0;
uint8_t g_u8RxStatus = 0;
uint32_t g_32DecodeTxdPinStatus = 0;
uint32_t g_u32DecodeTxBuf[2] = {0, 1};
DMA_DESC_T PDMA_RX0_DESC[2];
DMA_DESC_T PDMA_RX1_DESC[2];

/*---------------------------------------------------------------------------------------------------------*/
/* Function declaration                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

void Timer_PDMA_Enable(void)
{
    TIMER0->CTL = TIMER0->CTL | TIMER_CTL_TRGPDMA_Msk;
}

void Timer_PDMA_Disable(void)
{
    TIMER0->CTL = (TIMER0->CTL & ~(TIMER_CTL_TRGPDMA_Msk));
}

void Timer_Decode_Capture_Set(uint32_t u32Edge)
{
    TIMER0->EXTCTL = (TIMER0->EXTCTL & ~(TIMER_EXTCTL_CAPEDGE_Msk)) | u32Edge;
}

void Timer2_CNT_Reset(void)
{
    TIMER_Stop(TIMER2);
    TIMER2->CTL |= TIMER_CTL_RSTCNT_Msk;
}

void Rx_Done_Process(void)
{
    Timer_PDMA_Disable();

    if(DECODE_TXD == 0)
    {
        g_u32DecodeTxBuf[0] ^= 1;
        g_u32DecodeTxBuf[1] ^= 1;
    }

    Timer2_CNT_Reset();
    DECODE_TXD = 1;
    Timer_Decode_Capture_Set(TIMER_CAPTURE_RISING_EDGE);
    TIMER_ClearCaptureIntFlag(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
}

/* For timeout */
void Timer2_Init(void)
{
    TIMER_Open(TIMER2, TIMER_ONESHOT_MODE, 1);
    TIMER_EnableInt(TIMER2);
    NVIC_EnableIRQ(TMR2_IRQn);
}

void TMR2_IRQHandler(void)
{
    /* clear Timer1 interrupt flag */
    TIMER_ClearIntFlag(TIMER2);

    Rx_Done_Process();
}

/* For detecting Preamble pattern 0x7E by software */
void Manch_Receive_By_Software(uint32_t u32Time)
{
    if(g_u8RxStatus == 0)
    {
        Timer_Decode_Capture_Set(TIMER_CAPTURE_FALLING_EDGE);
        g_u8RxStatus = 1;
    }
    else if((g_u8RxStatus==1) || (g_u8RxStatus==2) || (g_u8RxStatus==3) || (g_u8RxStatus==4) || (g_u8RxStatus==5) || (g_u8RxStatus==6))
    {
        if((u32Time>(TIME_2T-TIME_OFFSET)) && (u32Time<(TIME_2T+TIME_OFFSET)))
        {
            if(g_u8RxStatus == 6)
            {
                Timer_Decode_Capture_Set(TIMER_CAPTURE_RISING_EDGE);
            }
            g_u8RxStatus++;
        }
        else
        {
            Timer_Decode_Capture_Set(TIMER_CAPTURE_RISING_EDGE);
            g_u8RxStatus = 0;
        }
    }
    else if(g_u8RxStatus == 7)
    {
        if((u32Time>(TIME_2T-TIME_OFFSET)) && (u32Time<(TIME_2T+TIME_OFFSET)))
        {
            g_u8RxStatus = 0;
            NVIC_DisableIRQ(TMR0_IRQn);

            TIMER_Start(TIMER2);
            Timer_Decode_Capture_Set(TIMER_CAPTURE_FALLING_AND_RISING_EDGE);
            Timer_PDMA_Enable();
        }
        else
        {
            Timer_Decode_Capture_Set(TIMER_CAPTURE_RISING_EDGE);
            g_u8RxStatus = 0;
        }
    }
}

void Timer_Decode_Init(void)
{
    /* Enable Timer1 event counter input and external capture function */
    TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER0, 47);   /* 1 MHz */
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_COUNTER_RESET_MODE, TIMER_CAPTURE_RISING_EDGE);
    TIMER_EnableCaptureDebounce(TIMER0);
    TIMER_SetTriggerSource(TIMER0, TIMER_TRGSRC_CAPTURE_EVENT);
    TIMER_EnableInt(TIMER0);
    TIMER_EnableCaptureInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    TIMER_Start(TIMER0);
}

void TMR0_IRQHandler(void)
{
    uint32_t u32Time = 0;

    if(TIMER_GetCaptureIntFlag(TIMER0) == 1)
    {
        /* Clear Timer1 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER0);

        u32Time = TIMER_GetCaptureData(TIMER0);

        /* Check if Preamble pattern 0x7E is received */
        Manch_Receive_By_Software(u32Time);
    }
}

void MANCH_Init(void)
{
    /*--------------------------------------------------------------------------------------*/
    /* Set MANCH TX and RX related format                                                   */
    /*--------------------------------------------------------------------------------------*/
    /* Open MANCH */
    MANCH_Open(MANCH, 1000);

    /* Set MANCH to NONE mode before to set related format */
    MANCH_SetMode(MANCH, MANCH_NONE);

    /* Set encoded format */
    MANCH_ENCODE_THOMAS(MANCH);

    /* Set Frame number */
    MANCH_SetFrameNum(MANCH, FRAME_LENGTH-1);

    /* Set Idle pattern */
    MANCH_SetIdle(MANCH, 0xFF);

    /* Set Preamble pattern */
    MANCH_SetPreamble(MANCH, PREAMBLE);

    /* Set Preamble number */
    MANCH_SetPreambleNum(MANCH, 1);

    /* Set TX/RX polarity */
    MANCH_TX_NOT_INVERTED(MANCH);
    MANCH_RX_NOT_INVERTED(MANCH);

    /* Disable upload bit number */
    MANCH_DISABLE_RX_UPLOAD_BITNUM_EACH_FRAME(MANCH);
    MANCH_DISABLE_RX_UPLOAD_BITNUM_EACH_BYTE(MANCH);

    /* Reset transmit/receive FIFO control */
    MANCH_CLR_TX_FIFO(MANCH);
    MANCH_NOTCLR_TX_FIFO(MANCH);
    MANCH_CLR_RX_FIFO(MANCH);
    MANCH_NOTCLR_RX_FIFO(MANCH);

    /* Clear Status register */
    MANCH_CLR_STATUS(MANCH);

    /*--------------------------------------------------------------------------------------*/
    /* Set MANCH mode switch finally                                                        */
    /*--------------------------------------------------------------------------------------*/
    /* Set selected MANCH mode */
    MANCH_SetMode(MANCH, MANCH_MODE_OTHER);

    MANCH_ENABLE_RXFRAME_DONE_INT(MANCH);
    MANCH_ENABLE_RX_OVER_INT(MANCH);
    MANCH_ENABLE_BIT_ERR_INT(MANCH);
    MANCH_ENABLE_IDLE_ERR_INT(MANCH);

    NVIC_EnableIRQ(MANCH_IRQn);
}

void PDMA_Rx_Init(void)
{
    /* Enable PDMA channels */
    PDMA_Open(PDMA, 1<<PDMA_DECODE_CH0);
    PDMA_SetTransferMode(PDMA, PDMA_DECODE_CH0, PDMA_TMR0, TRUE, (uint32_t)&PDMA_RX0_DESC[0]);

    PDMA_RX0_DESC[0].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_RX0_DESC[0].src = (uint32_t)&g_u32DecodeTxBuf[0];
    PDMA_RX0_DESC[0].dest = (uint32_t)&DECODE_TXD;
    PDMA_RX0_DESC[0].offset = (uint32_t)&PDMA_RX0_DESC[1] - (PDMA->SCATBA);

    PDMA_RX0_DESC[1].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_RX0_DESC[1].src = (uint32_t)&g_u32DecodeTxBuf[1];
    PDMA_RX0_DESC[1].dest = (uint32_t)&DECODE_TXD;
    PDMA_RX0_DESC[1].offset = (uint32_t)&PDMA_RX0_DESC[0] - (PDMA->SCATBA);   //link to first description

    /* Disable RX DMA */
    MANCH_DISABLE_RX_DMA(MANCH);

    /*=======================================================================
    MANCH RX PDMA channel configuration:
    ========================================================================*/
    PDMA_Open(PDMA, 1<<PDMA_DECODE_CH1);
    PDMA_SetTransferMode(PDMA, PDMA_DECODE_CH1, PDMA_MANCH_RX, TRUE, (uint32_t)&PDMA_RX1_DESC[0]);

    PDMA_RX1_DESC[0].ctl = ((FRAME_LENGTH - 2) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_8 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_RX1_DESC[0].src = (uint32_t)&MANCH->RXDAT;
    PDMA_RX1_DESC[0].dest = (uint32_t)g_u8ManchRxBuf[0];
    PDMA_RX1_DESC[0].offset = (uint32_t)&PDMA_RX1_DESC[1] - (PDMA->SCATBA);

    PDMA_RX1_DESC[1].ctl = ((FRAME_LENGTH - 2) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_8 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_RX1_DESC[1].src = (uint32_t)&MANCH->RXDAT;
    PDMA_RX1_DESC[1].dest = (uint32_t)g_u8ManchRxBuf[1];
    PDMA_RX1_DESC[1].offset = (uint32_t)&PDMA_RX1_DESC[0] - (PDMA->SCATBA);   //link to first description

    /* Enable RX DMA */
    MANCH_ENABLE_RX_DMA(MANCH);
}

void MANCH_IRQHandler(void)
{
    if(MANCH_IS_RXFRAME_DONE(MANCH))
    {
        MANCH_CLR_RXFRAME_DONE(MANCH);
        g_u8ManchRxDoneFlag = 1;
        g_u8ManchRxId = g_u8ManchRxId^1;
        Rx_Done_Process();
    }

    if(MANCH_IS_TXFRAME_DONE(MANCH))
    {
        MANCH_CLR_TXFRAME_DONE(MANCH);
        printf("Tx done. \n");
    }

    if(MANCH_IS_RX_OVER(MANCH))
    {
        MANCH_CLR_RX_OVER(MANCH);
        printf("RX FIFO overflow. \n");
    }

    if(MANCH_IS_BIT_ERR(MANCH))
    {
        MANCH_CLR_BIT_ERR(MANCH);
        printf("Bit error. \n");
    }

    if(MANCH_IS_IDLE_ERR(MANCH))
    {
        MANCH_CLR_IDLE_ERR(MANCH);
        printf("Idle error. \n");
    }
}

void Manchester_Decode_Init(void)
{
    GPIO_SetMode(DECODE_GPIO_PORT, DECODE_GPIO_PIN, GPIO_MODE_OUTPUT);
    DECODE_TXD = 1;
    PDMA_Rx_Init();
    Timer_Decode_Init();
    MANCH_Init();
    Timer2_Init();
}

int check_crc(uint8_t* buf)
{
    uint8_t i, j=0;
    uint8_t buf_checksum, checksum;
    uint32_t u32TimeOutCount = SystemCoreClock;

    /* Configure CRC, seed is X8+X5+X4+1 */
    CRC_Open(CRC_8, 0, SEED_X8_X5_X4_1, CRC_WDATA_8);

    /* Check the first non-Preamble data */
    for (i=0; i<FRAME_LENGTH; i++)
    {
        if (buf[i] != PREAMBLE)
        {
            j = i;
            break;
        }
        else
            j++;
    }

#ifdef OPT_CHECK_CRC_BY_PDMA
    buf += j;

    /* Start to caluculate CRC-8 checksum */
    for(i = 0; i < MSG_BEFORE_CRC; i++)
    {
        CRC_WRITE_DATA(*buf);
        buf++;
    }

    /* Store checksum in buffer */
    buf_checksum = *buf;
    buf++;

    /* Open Channel PDMA_DECODE_CRC_CH */
    PDMA_Open(PDMA,1 << PDMA_DECODE_CRC_CH);
    /* Transfer count is MSG_LENGTH, transfer width is 8 bits(one word) */
    PDMA_SetTransferCnt(PDMA, PDMA_DECODE_CRC_CH, PDMA_WIDTH_8, MSG_LENGTH);
    /* Set source address is u32Address, destination address is CRC->DAT, Source increment size is 32 bits(one word), Destination increment size is 0 */
    PDMA_SetTransferAddr(PDMA, PDMA_DECODE_CRC_CH, (uint32_t)buf, PDMA_SAR_INC, (uint32_t)&CRC->DAT, PDMA_DAR_FIX);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA, PDMA_DECODE_CRC_CH, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA, PDMA_DECODE_CRC_CH, PDMA_REQ_BURST, PDMA_BURST_1);

    /* Generate a software request to trigger transfer with PDMA */
    PDMA_Trigger(PDMA, PDMA_DECODE_CRC_CH);

    /* Waiting for transfer done */
    while(!((PDMA_GET_TD_STS(PDMA)&(PDMA_TDSTS_TDIF0_Msk<<PDMA_DECODE_CRC_CH))))
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nTimeout is happened, please check if something is wrong. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    /* Clear transfer done flag */
    PDMA_CLR_TD_FLAG(PDMA, (PDMA_TDSTS_TDIF0_Msk<<PDMA_DECODE_CRC_CH));

    /* Get CRC-8 checksum value */
    checksum = CRC_GetChecksum();

#else

    /* Start to calculate CRC checksum in buffer */
    for(i = 0; i < MSG_BEFORE_CRC; i++)
    {
        CRC_WRITE_DATA(buf[j]);
        j++;
    }

    /* Store checksum in buffer */
    buf_checksum = buf[j];
    j++;

    /* Continue to calculate CRC checksum in buffer */
    for(i = 0; i < MSG_LENGTH; i++)
    {
        CRC_WRITE_DATA(buf[j]);
        j++;
    }

    /* Get CRC-8 checksum value */
    checksum = CRC_GetChecksum();
#endif

    /* Compare whether CRC is correct or not */
    if (checksum == buf_checksum)
        return 0;
    else
        return -1;
}

#ifdef OPT_REARRANGE_RX
int rearrange_rx(uint8_t* bufSrc, uint8_t* bufDest)
{
    uint8_t i;

    /* Check the first non-Preamble data */
    for (i=0; i<FRAME_LENGTH; i++)
    {
        if (*bufSrc != PREAMBLE)
            break;
        else
            bufSrc++;
    }

    /* Given 0x7E to rearranged buffer */
    memset(bufDest, PREAMBLE, PREAMBLE_LENGTH);
    bufDest += PREAMBLE_LENGTH;

    /* Copy non-0x7E to rearranged buffer */
    memcpy(bufDest, bufSrc, FRAME_LENGTH-PREAMBLE_LENGTH-1);

    /* Given 0x7E to Frame End */
    bufDest += (FRAME_LENGTH-PREAMBLE_LENGTH-1);
    *bufDest =  PREAMBLE;
    return 0;
}
#endif
