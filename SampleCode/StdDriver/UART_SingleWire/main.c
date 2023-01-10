/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Two Single-Wire Loopback data test.
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_96MHZ
#define BUFSIZE   128

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[BUFSIZE] = {0};
uint8_t g_u8TxData [BUFSIZE] = {0};
volatile uint32_t g_u32RecLen  =  0;
volatile int32_t  g_i32RecOK  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART0_TEST_HANDLE(void);
void UART_FunctionTest(void);
void SingleWire_FunctionTxTest(void);
void SingleWire_FunctionRxTest(void);


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

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);


    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));



    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_UART0_RXD;

    /* The RX pin needs to pull-high for single-wire */
    /* If the external circuit doesn't pull-high, set GPIO pin as Quasi-directional mode for this purpose here */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE12_Msk) | (GPIO_MODE_QUASI << GPIO_MODE_MODE12_Pos);

    /* Lock protected registers */
    SYS_LockReg();

}
/*---------------------------------------------------------------------------------------------------------*/
/*                                     Init UART0                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* Debug port control the Single wire 1(UART1) send data to Single wire 2(UART2) or Single wire 2(UART1)   */
/* send data to Single wire 1(UART1)                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                           SAMPLE CODE                                                   */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while (1);

}


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 2 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    UART0_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART1 Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_TEST_HANDLE()
{

    if (UART_GET_INT_FLAG(UART0,UART_INTSTS_SWBEIF_Msk))
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART0, UART_INTSTS_SWBEINT_Msk);

    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest()
{
    char cmmd ;

    printf("+-------------------------------------------------------------+\n");
    printf("|                     Pin Configure                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  ______                                      ______         |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |Master|--RXD0(PB.12)  <====>  RXD0(PB.12)--|Slave |        |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |______|                                    |______|        |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");

    /*
        Semi-host is set to debug port for message.
        The SingleWire sample code needs two module board to execute.
        Set the master board is SingleWire TX Mode and the other is SingleWire Rx mode.
        Inputting char on terminal will be sent to the UART0 of master.
        After the master receiving, the inputting char will send to UART0 again.
        At the same time, it also sends to UART0 TX pin by SingleWire mode.
        Slave will print received char after UART0 send out.
    */


    /* Enable UART0 RDA/Time-out/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART0_IRQn);
    UART_EnableInt(UART0, UART_INTEN_SWBEIEN_Msk);

    printf("\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     SingleWire Function Test                                |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and      |\n");
    printf("|    the other is slave.  Master will send data based on      |\n");
    printf("|    terminal input and Slave will printf received data on    |\n");
    printf("|    terminal screen.                                         |\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n\n");
    cmmd = getchar();

    if (cmmd == '0')
        SingleWire_FunctionTxTest();
    else
        SingleWire_FunctionRxTest();

    /* Disable UART0 RDA/Time-out interrupt */
    UART_DisableInt(UART0, UART_INTEN_SWBEIEN_Msk);
    printf("\nUART Sample Demo End.\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*                             SingleWire Function Transmit Test                                           */
/*---------------------------------------------------------------------------------------------------------*/
void SingleWire_FunctionTxTest(void)
{
    uint8_t u8OutChar;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     SingleWire Function Tx Mode Test                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Input char by UART0 terminal.                         |\n");
    printf("| 2). UART0 will send a char according to step 1.           |\n");
    printf("| 3). Return step 1. (Press '0' to exit)                    |\n");
    printf("+-----------------------------------------------------------+\n\n");

    printf("\nIRDA Sample Code Start. \n");

    /* Select Single Rx mode */
    UART_SelectSingleWireMode(UART0);

    /* Wait Terminal input to send data to UART0 TX pin */
    do
    {
        u8OutChar = getchar();
        printf(" Input: %c , Send %c out\n", u8OutChar, u8OutChar);
        UART_WRITE(UART0, u8OutChar);
    }
    while (u8OutChar != '0');

}
/*---------------------------------------------------------------------------------------------------------*/
/*                             SingleWire Function Receive Test                                            */
/*---------------------------------------------------------------------------------------------------------*/
void SingleWire_FunctionRxTest(void)
{
    uint8_t u8InChar = 0xFF;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     SingleWire Function Rx Mode Test                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Polling RDA_Flag to check data input though UART      |\n");
    printf("| 2). If received data is '0', the program will exit.       |\n");
    printf("|     Otherwise, print received data on terminal            |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* Select Single Rx mode */
    UART_SelectSingleWireMode(UART0);

    /* Reset Rx FIFO */
    UART0->FIFO |= UART_FIFO_RXRST_Msk;
    while(UART0->FIFO & UART_FIFO_RXRST_Msk);

    printf("Waiting...\n");

    /* Use polling method to wait master data */
    do
    {
        if (UART_IS_RX_READY(UART0))
        {
            u8InChar = UART_READ(UART0);
            printf("   Input: %c \n", u8InChar);
        }
    }
    while (u8InChar != '0');

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/


