/******************************************************************************
 * @file     i2c_transfer.h
 * @brief    I2C ISP slave header file
 * @version  1.0.0
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __I2C_TRANS_H__
#define __I2C_TRANS_H__
#include <stdint.h>

extern volatile uint8_t bI2cDataReady;
extern volatile uint8_t bI2cSlvEndFlag;
extern uint8_t i2c_rcvbuf[];

/*-------------------------------------------------------------*/
void I2C_Init(void);
void I2C_SlaveTRx(I2C_T *i2c, uint32_t u32Status);

#endif  /* __I2C_TRANS_H__ */
