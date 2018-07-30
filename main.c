#include <msp430.h> 
#include <stdio.h>
#include <string.h>
#include <iic.h>
#include "isl94202.h"
#include "app_trace.h"
#include "mcp2515.h"


can_t can_tx;                                                                    // CAN-Sendevariable
can_t can_rx;                                                                    // CAN-Empfangsvariable

unsigned char ret;
unsigned char CurrentVoltageBuffer[22];

void uart_init(void)
{
    P1SEL = BIT1 + BIT2; // P1.1Ϊ RXD�� P1.2ΪTXD
    P1SEL2 = BIT1 + BIT2; // P1.1Ϊ RXD�� P1.2ΪTXD
    UCA0CTL1 |= UCSSEL_2; // ѡ��ʱ��BRCLK
    UCA0BR0 = 106; // 1MHz 9600
    UCA0BR1 = 0; // 1MHz 9600
    UCA0MCTL = UCBRS2 + UCBRS0; //������=BRCLK/(UBR+��M7+.��.0��/8)
//    UCA0BR0 = 8; // 1MHz 115200
//    UCA0BR1 = 0; // 1MHz 115200
//    UCA0MCTL = UCBRS1 + UCBRS0; //������=BRCLK/(UBR+��M7+.��.0��/8)
    UCA0CTL1 &= ~UCSWRST;

    // ��ʼ��˳��SWRST=1���ô��ڣ�Ȼ������SWRST=0�����������Ӧ�ж�

//    ����IE2 |= UCA0RXIE; // ʹ�ܽ����ж�
}

void UartPutchar(unsigned char c)
{
    while(!(IFG2 & UCA0TXIFG)); //������Ϊ��
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

	P2DIR &= ~(BIT0|BIT1|BIT2|BIT3);
    P2DIR |= BIT4;
    P2OUT &= ~BIT4;
//    P2OUT |= BIT4;

    BCSCTL1 = CALBC1_1MHZ;                    // Set range
    DCOCTL  = CALDCO_1MHZ;
//    uart_init();
    MCP2515_SPI_init();
    // initialize Timer_A module
    TACCR0 = 12500;
    TACTL = TASSEL_2 + MC_1 + TACLR + ID_3;   // ACLK, up mode, clear TAR
    TACCTL0 |= CCIE;                          // CCR0 interrupt enabled
    _BIS_SR(GIE);              //�����ж�

//    ISL94202_Init();
    MCP2515_spi_test();
    MCP2515_init();                                                                // Initialisiert MCP2515
    MCP2515_CanVariable_init (&can_tx);                                            // Initialisiert Sendevariable

    while(1)
    {
        __bis_SR_register(LPM1_bits);       // Enter LPM3, enable interrupts
        MCP2515_can_tx0(&can_tx);                                                      // Sende das Empfangene zur�ck (Echo)
//        ISL94202_updateReadings();                  //load in new I2C data
//        ISL94202_updateStatus();
//        app_trace_log("v1 %04d V2 %04d V3 %04d V4 %04d I %04d X1 %04d X2 %04d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n",
//               ISL94202_getCellVoltageMV(0),
//               ISL94202_getCellVoltageMV(1),
//               ISL94202_getCellVoltageMV(6),
//               ISL94202_getCellVoltageMV(7),
//               ISL94202_getCurrentTemperature(0),
//               ISL94202_getCurrentTemperature(1),
//               ISL94202_getCurrentTemperature(2),
//               ISL94202_getCurrentStatus(0),
//               ISL94202_getCurrentStatus(1),
//               ISL94202_getCurrentStatus(2),
//               ISL94202_getCurrentStatus(3),
//               ISL94202_getBalancingCells(),
//               P2IN&0x0f);
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
