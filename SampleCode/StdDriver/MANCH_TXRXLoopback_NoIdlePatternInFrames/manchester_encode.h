/**************************************************************************//**
 * @file     manchester_encode.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/06/10 2:23p $
 * @brief
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __MANCHESTER_ENCODE_H__
#define __MANCHESTER_ENCODE_H__

#include <stdio.h>
#include "NuMicro.h"
#include <math.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Constants                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

#define ENCODE_GPIO_PORT        PA
#define ENCODE_GPIO_PIN         BIT2
#define ENCODE_TXD              PA2

#define OPT_ADD_CRC_BY_PDMA
#define PDMA_ENCODE_CH0         0
#define PDMA_ENCODE_CH1         1
#define PDMA_ENCODE_CH2         2
#ifdef OPT_ADD_CRC_BY_PDMA
#define PDMA_ENCODE_CRC_CH      5
#endif

#define ENCODE_TX_HIGH          0x02
#define ENCODE_TX_LOW          0x04
#define OPT_AUTO_SINE_HIGH

/* Sample number per sine waveform for DAC0 */
#define SAMPLE0_PER_SINE        0x00
#define SAMPLE8_PER_SINE        0x01
#define SAMPLE16_PER_SINE       0x02
#define SAMPLE32_PER_SINE       0x03

/* DAC0 sine table */
#define SINE_FREQ_IN_HZ         500000
#define DAC_DATA                0x7FF
#define SINE_SAMPLE             0x20
#define PI                      3.1416

/* CRC checking */
#define SEED_X8_X5_X4_1         0x31
#define PREAMBLE                0x7E
#define PREAMBLE_LENGTH         0x4
#define MSG_BEFORE_CRC          0x4
#define MSG_LENGTH              0x36
#define FRAME_LENGTH            0x40

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

extern uint32_t g_u32PinHigh;
extern uint32_t g_u32PinLow;
extern uint8_t g_u8ManchRxDoneFlag;
extern uint8_t g_u8EncodeBuf[2][1024];

/*---------------------------------------------------------------------------------------------------------*/
/* Function declaration                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

extern void Encode_Buf_Fill(uint8_t *u8ManchTxBuf, uint8_t u8Id);
extern void Manchester_Encode_Init(void);
extern void add_crc(uint8_t* buf);

#endif

