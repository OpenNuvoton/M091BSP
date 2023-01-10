/**************************************************************************//**
 * @file     manchester_encode.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/06/10 2:23p $
 * @brief
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "manchester_encode.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

DMA_DESC_T PDMA_TX0_DESC[2];
DMA_DESC_T PDMA_TX1_DESC[3];
DMA_DESC_T PDMA_TX2_DESC[3];

uint8_t g_u8EncodeBuf[2][1024];
uint32_t g_u32DacEnable = 0x0000000D;
uint32_t g_u32DacDisable = 0x0000000C;
uint32_t g_u32DacData = DAC_DATA;
uint32_t g_u32PinHigh = 1;
uint32_t g_u32PinLow = 0;
uint16_t g_sineBuf[32];
uint8_t g_u8ManchTxDoneFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Function declaration                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

void PDMA_IRQHandler(void)
{
    /* Check channel transfer done status */
    if ((PDMA_GET_TD_STS(PDMA)&(PDMA_TDSTS_TDIF0_Msk<<PDMA_ENCODE_CH0)) == PDMA_TDSTS_TDIF0_Msk)
    {
        /* Clear transfer done flag of channel 0 */
        PDMA_CLR_TD_FLAG(PDMA, (PDMA_TDSTS_TDIF0_Msk<<PDMA_ENCODE_CH0));

        g_u8ManchTxDoneFlag = 1;
    }

    if ((PDMA_GET_TD_STS(PDMA)&(PDMA_TDSTS_TDIF0_Msk<<PDMA_ENCODE_CH1)) == PDMA_TDSTS_TDIF1_Msk)
    {
        /* Clear transfer done flag of channel 1 */
        PDMA_CLR_TD_FLAG(PDMA, (PDMA_TDSTS_TDIF0_Msk<<PDMA_ENCODE_CH1));
    }
}

void PDMA_Encode_Init(void)
{
    SYS_ResetModule(PDMA_RST);

    /* Enable PDMA channels */
    PDMA_Open(PDMA, 1<<PDMA_ENCODE_CH0);
    PDMA_SetTransferMode(PDMA, PDMA_ENCODE_CH0, PDMA_TMR1, TRUE, (uint32_t)&PDMA_TX0_DESC[0]);

    PDMA_TX0_DESC[0].ctl = ((1024 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_8 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_TX0_DESC[0].src = (uint32_t)g_u8EncodeBuf[0];
    PDMA_TX0_DESC[0].dest = (uint32_t)&PDMA->SWREQ;
    PDMA_TX0_DESC[0].offset = (uint32_t)&PDMA_TX0_DESC[1] - (PDMA->SCATBA);

    PDMA_TX0_DESC[1].ctl = ((1024 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_8 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_TX0_DESC[1].src = (uint32_t)g_u8EncodeBuf[1];
    PDMA_TX0_DESC[1].dest = (uint32_t)&PDMA->SWREQ;
    PDMA_TX0_DESC[1].offset = (uint32_t)&PDMA_TX0_DESC[0] - (PDMA->SCATBA);   //link to first description

    PDMA_EnableInt(PDMA, PDMA_ENCODE_CH0, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);

#ifdef OPT_AUTO_SINE_HIGH
    /* Enable PDMA channels */
    PDMA_Open(PDMA, 1<<PDMA_ENCODE_CH1);
    PDMA_SetTransferMode(PDMA, PDMA_ENCODE_CH1, PDMA_MEM, TRUE, (uint32_t)&PDMA_TX1_DESC[0]);

    PDMA_TX1_DESC[0].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_BURST | PDMA_OP_SCATTER;
    PDMA_TX1_DESC[0].src = (uint32_t)&g_u32DacEnable;
    PDMA_TX1_DESC[0].dest = (uint32_t)&DAC0->ADGCTL;
    PDMA_TX1_DESC[0].offset = (uint32_t)&PDMA_TX1_DESC[1] - (PDMA->SCATBA);

    PDMA_TX1_DESC[1].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_TX1_DESC[1].src = (uint32_t)&g_u32PinHigh;
    PDMA_TX1_DESC[1].dest = (uint32_t)&ENCODE_TXD;
    PDMA_TX1_DESC[1].offset = (uint32_t)&PDMA_TX1_DESC[0] - (PDMA->SCATBA);   //link to first description

    /* Enable PDMA channels */
    PDMA_Open(PDMA, 1<<PDMA_ENCODE_CH2);

    PDMA_SetTransferMode(PDMA, PDMA_ENCODE_CH2, PDMA_MEM, TRUE, (uint32_t)&PDMA_TX2_DESC[0]);

    PDMA_TX2_DESC[0].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_BURST | PDMA_OP_SCATTER;
    PDMA_TX2_DESC[0].src = (uint32_t)&g_u32DacDisable;
    PDMA_TX2_DESC[0].dest = (uint32_t)&DAC0->ADGCTL;
    PDMA_TX2_DESC[0].offset = (uint32_t)&PDMA_TX2_DESC[1] - (PDMA->SCATBA);

    PDMA_TX2_DESC[1].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_BURST | PDMA_OP_SCATTER;
    PDMA_TX2_DESC[1].src = (uint32_t)&g_u32DacData;
    PDMA_TX2_DESC[1].dest = (uint32_t)&DAC0->DAT;
    PDMA_TX2_DESC[1].offset = (uint32_t)&PDMA_TX2_DESC[2] - (PDMA->SCATBA);   //link to first description

    PDMA_TX2_DESC[2].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_TX2_DESC[2].src = (uint32_t)&g_u32PinLow;
    PDMA_TX2_DESC[2].dest = (uint32_t)&ENCODE_TXD;
    PDMA_TX2_DESC[2].offset = (uint32_t)&PDMA_TX2_DESC[0] - (PDMA->SCATBA);   //link to first description
#else
    /* Enable PDMA channels */
    PDMA_Open(PDMA, 1<<PDMA_ENCODE_CH1);

    PDMA_SetTransferMode(PDMA, PDMA_ENCODE_CH1, PDMA_MEM, TRUE, (uint32_t)&PDMA_TX1_DESC[0]);

    PDMA_TX1_DESC[0].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_BURST | PDMA_OP_SCATTER;
    PDMA_TX1_DESC[0].src = (uint32_t)&g_u32DacDisable;
    PDMA_TX1_DESC[0].dest = (uint32_t)&DAC0->ADGCTL;
    PDMA_TX1_DESC[0].offset = (uint32_t)&PDMA_TX1_DESC[1] - (PDMA->SCATBA);

    PDMA_TX1_DESC[1].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_BURST | PDMA_OP_SCATTER;
    PDMA_TX1_DESC[1].src = (uint32_t)&g_u32DacData;
    PDMA_TX1_DESC[1].dest = (uint32_t)&DAC0->DAT;
    PDMA_TX1_DESC[1].offset = (uint32_t)&PDMA_TX1_DESC[2] - (PDMA->SCATBA);   //link to first description

    PDMA_TX1_DESC[2].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_TX1_DESC[2].src = (uint32_t)&g_u32PinHigh;
    PDMA_TX1_DESC[2].dest = (uint32_t)&ENCODE_TXD;
    PDMA_TX1_DESC[2].offset = (uint32_t)&PDMA_TX1_DESC[0] - (PDMA->SCATBA);   //link to first description

    /* Enable PDMA channels */
    PDMA_Open(PDMA, 1<<PDMA_ENCODE_CH2);
    PDMA_SetTransferMode(PDMA, PDMA_ENCODE_CH2, PDMA_MEM, TRUE, (uint32_t)&PDMA_TX2_DESC[0]);

    PDMA_TX2_DESC[0].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_BURST | PDMA_OP_SCATTER;
    PDMA_TX2_DESC[0].src = (uint32_t)&g_u32DacEnable;
    PDMA_TX2_DESC[0].dest = (uint32_t)&DAC0->ADGCTL;
    PDMA_TX2_DESC[0].offset = (uint32_t)&PDMA_TX2_DESC[1] - (PDMA->SCATBA);

    PDMA_TX2_DESC[1].ctl = ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    PDMA_TX2_DESC[1].src = (uint32_t)&g_u32PinLow;
    PDMA_TX2_DESC[1].dest = (uint32_t)&ENCODE_TXD;
    PDMA_TX2_DESC[1].offset = (uint32_t)&PDMA_TX2_DESC[0] - (PDMA->SCATBA);   //link to first description
#endif
}

void Timer_Encode_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 2000);
    TIMER_SetTriggerTarget(TIMER1, TIMER_TRG_TO_PDMA);
    TIMER_Start(TIMER1);
}

//Id:0/1
void Encode_Buf_Fill(uint8_t *u8ManchTxBuf, uint8_t u8Id)
{
    uint8_t i;
    uint8_t j;
    uint8_t u8Dat = 0;
    uint16_t u16BitCount = 0;

    for(i=0; i<FRAME_LENGTH; i++)
    {
        for(j=0; j<8; j++)
        {
            u8Dat = u8ManchTxBuf[i] >> (7-j);
            if(u8Dat & 0x1) /* 1 is encode to 10 */
            {
                g_u8EncodeBuf[u8Id][u16BitCount] = ENCODE_TX_HIGH;
                u16BitCount++;
                g_u8EncodeBuf[u8Id][u16BitCount] = ENCODE_TX_LOW;
                u16BitCount++;
            }
            else            /* 0 is encode to 01 */
            {
                g_u8EncodeBuf[u8Id][u16BitCount] = ENCODE_TX_LOW;
                u16BitCount++;
                g_u8EncodeBuf[u8Id][u16BitCount] = ENCODE_TX_HIGH;
                u16BitCount++;
            }
        }
    }
}

void DAC_Encode_Init(void)
{
    uint8_t i;
    uint32_t u32SampleNumPerSine;

    DAC_Open(DAC0, 0, DAC_WRITE_DAT_TRIGGER);
    DAC_ENABLE_RIGHT_ALIGN(DAC0);

    /* Set sample number per sine waveform */
    DAC_SetAutoSineSampleNum(DAC0, SAMPLE32_PER_SINE);
    u32SampleNumPerSine = 32;

    /* Set sine waveform frequency; sample number per sine waveform
        must be given in advance */
    DAC_SetAutoSineFreq(DAC0, SINE_FREQ_IN_HZ);

    /* set TX level with frequency modulation */
    DAC_AUTO_SINE_MANCH_TX_HIGH(DAC0);

    /* prepare and set sine waveform data to DAC0 ADCTL[] registers */
    for (i=0; i<SINE_SAMPLE; i++)
    {
        /* Add 1.0 to offset sine result from [-1, 1] to [0, 2],
           and divided with 2.0 to compress to [0, 1] */
        g_sineBuf[i] = (uint16_t)(((sin((double)(((i+1) * PI) / (SINE_SAMPLE/2))) + 1.0) / 2.0) * 0xFFF);
    }

    DAC_SetAutoSineSampleContent(DAC0, g_sineBuf, u32SampleNumPerSine);
}

void Manchester_Encode_Init(void)
{
    GPIO_SetMode(ENCODE_GPIO_PORT, ENCODE_GPIO_PIN, GPIO_MODE_OUTPUT);
    ENCODE_TXD = 0;
    DAC_Encode_Init();
    PDMA_Encode_Init();
    Timer_Encode_Init();
}

void add_crc(uint8_t* buf)
{
    int i;
#ifdef OPT_ADD_CRC_BY_PDMA
    uint8_t* ptr;
#endif
    uint32_t u32TimeOutCount = SystemCoreClock;

    /* Configure CRC, seed is X8+X5+X4+1 */
    CRC_Open(CRC_8, 0, SEED_X8_X5_X4_1, CRC_WDATA_8);

    /* given information lenght */
    buf[7] = MSG_LENGTH;

#ifdef OPT_ADD_CRC_BY_PDMA
    ptr = buf;

    /* Start to caluculate CRC-8 checksum */
    buf += PREAMBLE_LENGTH;
    for(i = 0; i < MSG_BEFORE_CRC; i++)
    {
        CRC_WRITE_DATA(*buf);
        buf++;
    }
    buf++;

    /* Open Channel PDMA_ENCODE_CRC_CH */
    PDMA_Open(PDMA,1 << PDMA_ENCODE_CRC_CH);
    /* Transfer count is MSG_LENGTH, transfer width is 8 bits(one word) */
    PDMA_SetTransferCnt(PDMA, PDMA_ENCODE_CRC_CH, PDMA_WIDTH_8, MSG_LENGTH);
    /* Set source address is u32Address, destination address is CRC->DAT, Source increment size is 32 bits(one word), Destination increment size is 0 */
    PDMA_SetTransferAddr(PDMA, PDMA_ENCODE_CRC_CH, (uint32_t)buf, PDMA_SAR_INC, (uint32_t)&CRC->DAT, PDMA_DAR_FIX);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA, PDMA_ENCODE_CRC_CH, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA, PDMA_ENCODE_CRC_CH, PDMA_REQ_BURST, PDMA_BURST_1);

    /* Generate a software request to trigger transfer with PDMA channel 1  */
    PDMA_Trigger(PDMA, PDMA_ENCODE_CRC_CH);

    /* Waiting for transfer done */
    while(!((PDMA_GET_TD_STS(PDMA)&(PDMA_TDSTS_TDIF0_Msk<<PDMA_ENCODE_CRC_CH))))
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nTimeout is happened, please check if something is wrong. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    /* Clear transfer done flag of channel 5 */
    PDMA_CLR_TD_FLAG(PDMA, (PDMA_TDSTS_TDIF0_Msk<<PDMA_ENCODE_CRC_CH));

    /* Set CRC checksum to buffer */
    ptr += PREAMBLE_LENGTH+MSG_BEFORE_CRC;
    *ptr = CRC_GetChecksum();

#else
    /* Start to caluculate CRC-8 checksum */
    for(i = 0; i < MSG_BEFORE_CRC; i++)
    {
        CRC_WRITE_DATA(buf[i+PREAMBLE_LENGTH]);
    }

    for(i = 0; i < MSG_LENGTH; i++)
    {
        CRC_WRITE_DATA(buf[i+PREAMBLE_LENGTH+MSG_BEFORE_CRC+1]);
    }

    /* Set CRC checksum to buffer */
    buf[PREAMBLE_LENGTH+MSG_BEFORE_CRC] = CRC_GetChecksum();
#endif

}

