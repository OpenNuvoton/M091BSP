/**************************************************************************//**
 * @file     manchester_decode.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/06/10 2:23p $
 * @brief
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __MANCHESTER_DECODE_H__
#define __MANCHESTER_DECODE_H__

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "manchester_encode.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Constants                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

#define TIME_1T             500
#define TIME_2T             1000
#define TIME_3T             1500
#define TIME_4T             2000
#define TIME_OFFSET         100

#define DECODE_GPIO_PORT    PB
#define DECODE_GPIO_PIN     BIT5
#define DECODE_TXD          PB5

#define OPT_CHECK_CRC_BY_PDMA
#define PDMA_DECODE_CH0     3
#define PDMA_DECODE_CH1     4
#ifdef OPT_CHECK_CRC_BY_PDMA
#define PDMA_DECODE_CRC_CH  6
#endif

//#define OPT_REARRANGE_RX

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

extern uint8_t g_u8ManchRxBuf[2][64];
extern uint8_t g_u8ManchRxBuf_ReArranged[2][64];
extern uint8_t g_u8ManchRxId;
extern uint8_t g_u8ManchTxDoneFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Function declaration                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

extern void Manchester_Decode_Init(void);
extern int check_crc(uint8_t* buf);
extern int rearrange_rx(uint8_t* bufSrc, uint8_t* bufDest);
#endif

