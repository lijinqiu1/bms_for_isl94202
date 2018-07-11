#include <msp430.h> 
#include <stdio.h>
#include <string.h>
#include <iic.h>
#include "isl94202.h"
unsigned char IICRXByte;
unsigned int IICRXWord = 0x00;

unsigned char ret;
unsigned char CurrentVoltageBuffer[22];

#define UART_PRINTF

#ifdef UART_PRINTF
int fputc(int _c, register FILE *_fp);
int fputs(const char *_ptr, register FILE *_fp);
#endif

void uart_init(void)
{
    P1SEL = BIT1 + BIT2; // P1.1为 RXD， P1.2为TXD
    P1SEL2 = BIT1 + BIT2; // P1.1为 RXD， P1.2为TXD
    UCA0CTL1 |= UCSSEL_2; // 选择时钟BRCLK
    UCA0BR0 = 106; // 1MHz 9600
    UCA0BR1 = 0; // 1MHz 9600
    UCA0MCTL = UCBRS2 + UCBRS0; //波特率=BRCLK/(UBR+（M7+.。.0）/8)
    UCA0CTL1 &= ~UCSWRST;

    // 初始化顺序：SWRST=1设置串口，然后设置SWRST=0，最后设置相应中断

//    　　IE2 |= UCA0RXIE; // 使能接收中断
}

void UartPutchar(unsigned char c)
{
    while(!(IFG2 & UCA0TXIFG)); //待发送为空
    UCA0TXBUF=c;
    IFG2 &=~UCA0RXIFG;
}

unsigned long value;
/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    P2DIR |= BIT4;
    P2OUT &= ~BIT4;
//    P2OUT |= BIT4;

    BCSCTL1 = CALBC1_1MHZ;                    // Set range
    DCOCTL  = CALDCO_1MHZ;
    uart_init();
    // initialize Timer_A module
    TACCR0 = 12500;
    TACTL = TASSEL_2 + MC_1 + TACLR + ID_3;   // ACLK, up mode, clear TAR
    TACCTL0 |= CCIE;                          // CCR0 interrupt enabled
    _BIS_SR(GIE);              //开总中断

    IICRXWord = ISL94202_ReadOVThres();
    if (IICRXWord != ISL94202_milliVoltsToVScaleRaw(4500))
    {
        ISL94202_setOVThres(4500);
        printf("setOVThres\r\n");
    }
    else
    {
        printf("OVThres OK\r\n");
    }


    IICRXWord = ISL94202_ReadOVRecovery();
    if (IICRXWord != ISL94202_milliVoltsToVScaleRaw(4500))
    {
        ISL94202_setOVRecovery(4500);
        printf("setOVRecovery\r\n");
    }
    else
    {
        printf("OVRecovery OK\r\n");
    }


    IICRXWord = ISL94202_ReadUVThres();
    if (IICRXWord != ISL94202_milliVoltsToVScaleRaw(1500))
    {
        ISL94202_setUVThres(1500);
        printf("setUVThres\r\n");
    }
    else
    {
        printf("UVThres OK\r\n");
    }

    IICRXWord = ISL94202_ReadUVRecovery();
    if (IICRXWord != ISL94202_milliVoltsToVScaleRaw(1500))
    {
        ISL94202_setUVRecovery(1500);
        printf("setUVRecovery\r\n");
    }
    else
    {
        printf("UVThres OK\r\n");
    }

    IICRXWord = ISL94202_ReadOVLockout();
    if (IICRXWord != ISL94202_milliVoltsToVScaleRaw(4500))
    {
        ISL94202_setOVLockout(4500);
        printf("setOVLockout\r\n");
    }
    else
    {
        printf("OVLockout OK\r\n");
    }

    IICRXWord = ISL94202_ReadUVLockout();
    if (IICRXWord != ISL94202_milliVoltsToVScaleRaw(1500))
    {
        ISL94202_setUVLockout(1500);
        printf("setUVLockout\r\n");
    }
    else
    {
        printf("UVLockout OK\r\n");
    }

    IICRXWord = ISL94202_ReadEOCThreshold();
    if (IICRXWord != ISL94202_milliVoltsToVScaleRaw(4500))
    {
        ISL94202_setEOCThreshold(4500);
        printf("setEOCThreshold\r\n");
    }
    else
    {
        printf("EOCThreshold OK\r\n");
    }

    IICRXByte=ISL94202_readEEPROM(0x49);
    if (IICRXByte != 0xC3)
    {
        ISL94202_setCellCountSleepTimes(4,15,15);
        printf("setCellCount\r\n");
    }
    else
    {
        printf("CellCount OK\r\n");
    }

//    IICRXByte = ISL94202_readEEPROM(0x4B);
//    if ((IICRXByte & 0xc0) != 0x00)
//    {
//        ISL94202_writeEEPROM(0x4B,IICRXByte & 0x3f);
//        printf("set balance off\r\n");
//    }
//    else
//    {
//        printf("balance off\r\n");
//    }
//
//    Data = I2C_readReg8(0x50,0x17);
//    if (Data != 0x74)
//    {
//       Data = 0x74;
//       I2C_writeReg8(0x50, 0x17,Data);
//       Data = I2C_readReg8(0x50,0x17);
//    }

    while(1)
    {
        __bis_SR_register(LPM1_bits);       // Enter LPM3, enable interrupts
        ISL94202_updateReadings();                  //load in new I2C data
        ISL94202_updateStatus();
        printf("v1 %04d V2 %04d V3 %04d V4 %04d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n",ISL94202_getCellVoltageMV(0),
               ISL94202_getCellVoltageMV(1),
               ISL94202_getCellVoltageMV(6),
               ISL94202_getCellVoltageMV(7),
               ISL94202_getCurrentStatus(0),
               ISL94202_getCurrentStatus(1),
               ISL94202_getCurrentStatus(2),
               ISL94202_getCurrentStatus(3),
               ISL94202_getBalancingCells());
    }
}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    static unsigned char count=0;
    if (count ++ > 10)
    {
        __bic_SR_register_on_exit(LPM1_bits);
        count = 0;
    }
}

#ifdef UART_PRINTF
int fputc(int _c, register FILE *_fp)
{
    while(!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = (unsigned char) _c;

    return((unsigned char)_c);
}

int fputs(const char *_ptr, register FILE *_fp)
{
    unsigned int i, len;

    len = strlen(_ptr);

    for(i=0 ; i<len ; i++)
    {
        while(!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = (unsigned char) _ptr[i];
    }

    return len;
}
#endif
