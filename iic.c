/*
 * iic.c
 *
 *  Created on: 2018Äê6ÔÂ5ÈÕ
 *      Author: Administrator
 */
#include <msp430.h>
#define  SlaveWriteAddress  0x50

#define  SlaveReadAddress   0x51

#define  OwnAddress         0xee

#define  I2CSDA             BIT7

#define  I2CSCL             BIT6

#define  I2CSDA_SET_1       P1OUT |=  I2CSDA

#define  I2CSDA_SET_0       P1OUT &=~ I2CSDA

#define  I2CSCL_SET_1       P1OUT |=  I2CSCL

#define  I2CSCL_SET_0       P1OUT &=~ I2CSCL

#define  I2CSDA_INPUT_IN    P1IN&I2CSDA



unsigned char READI2CBUF[22];
unsigned char READI2CWORD[2];
unsigned char READI2CCHAR;


void I2C_Pins_DIR_Setting ( unsigned char SDADIR )
{

    P1DIR  |= I2CSDA + I2CSCL;

    if(SDADIR==1)
    {

        P1DIR &=~ I2CSDA;

        P1OUT &=~ I2CSDA;

    }

}



void Engender_I2C_start_signal(void)
{

    I2CSCL_SET_1;  __delay_cycles(50);

    I2CSDA_SET_1;  __delay_cycles(50);

    I2CSDA_SET_0;  __delay_cycles(50);

}

void Engender_I2C_stop_signal(void)
{

    I2CSDA_SET_0;   __delay_cycles(50);

    I2CSCL_SET_1;   __delay_cycles(50);

    I2CSDA_SET_1;   __delay_cycles(50);

}

void Engender_I2C_ack_signal(void)
{

    I2CSCL_SET_0;   __delay_cycles(50);

    I2CSCL_SET_1;   __delay_cycles(50);

    I2CSCL_SET_0;   __delay_cycles(50);

}

void Engender_I2C_noack_signal(void)
{

    I2CSDA_SET_1;   __delay_cycles(50);

    I2CSCL_SET_0;   __delay_cycles(50);

    I2CSCL_SET_1;   __delay_cycles(50);

    I2CSCL_SET_0;   __delay_cycles(50);

}



void WRITE_BYTE( unsigned char BytEDAta )

{

    unsigned char count;

    for(count=0;count<8;count++)

    {

        I2CSCL_SET_0;

        __delay_cycles(50);

        if(BytEDAta&0x80)I2CSDA_SET_1;

        else             I2CSDA_SET_0;

        __delay_cycles(50);

        I2CSCL_SET_1;

        __delay_cycles(50);

        BytEDAta<<=1;

    }

}

unsigned char READ_BYTE( void )
{
    unsigned char count,readbyte=0;

    I2CSCL_SET_0;

    for(count=0;count<8;count++)

    {

        readbyte=readbyte<<1;

        I2CSCL_SET_1;

        __delay_cycles(50);

        if(I2CSDA_INPUT_IN)readbyte=readbyte|0x01;

        else               readbyte=readbyte&0xfe;

        __delay_cycles(50);

        I2CSCL_SET_0;

        __delay_cycles(50);

    }

    I2CSCL_SET_0;

    return readbyte;
}

void I2C_writeReg8(unsigned char slave, unsigned char add, unsigned char data)
{
    unsigned char slave_add = slave << 1;
    I2C_Pins_DIR_Setting(0);

    Engender_I2C_start_signal();

    WRITE_BYTE(slave_add);

    Engender_I2C_ack_signal();

    WRITE_BYTE(add);

    Engender_I2C_ack_signal();

    WRITE_BYTE(data);

    Engender_I2C_ack_signal();

    Engender_I2C_stop_signal();

    __delay_cycles(15000);

    I2CSCL_SET_0;

    I2CSDA_SET_0;
}

void I2C_writeRegs(unsigned char slave, unsigned char add, unsigned char *data, unsigned char count)
{
    unsigned char i = 0;
    unsigned char slave_add = slave << 1;

    I2C_Pins_DIR_Setting(0);

    Engender_I2C_start_signal();

    WRITE_BYTE(slave_add);

    Engender_I2C_ack_signal();

    WRITE_BYTE(add);

    Engender_I2C_ack_signal();

    for (i = 0; i < count - 1; i++)
    {
        WRITE_BYTE(data[i]);

        Engender_I2C_ack_signal();
        __delay_cycles(45000);
    }
    WRITE_BYTE(data[i]);

    Engender_I2C_stop_signal();

    __delay_cycles(15000);

    I2CSCL_SET_0;

    I2CSDA_SET_0;
}

unsigned char I2C_readReg8(unsigned char slave,  unsigned char add )
{

    unsigned char readdate;
    unsigned char slave_add = slave << 1;

    I2C_Pins_DIR_Setting(0);

    Engender_I2C_start_signal();

    WRITE_BYTE(slave_add);

    Engender_I2C_ack_signal();

    WRITE_BYTE(add);

    Engender_I2C_ack_signal();

    __delay_cycles(15000);

    Engender_I2C_start_signal();

    WRITE_BYTE(slave_add + 1);

    I2C_Pins_DIR_Setting(1);

    Engender_I2C_ack_signal();

    readdate=READ_BYTE();

    I2C_Pins_DIR_Setting(0);

    Engender_I2C_noack_signal();

    Engender_I2C_stop_signal();

    I2CSCL_SET_0;

    I2CSDA_SET_0;

    return (readdate);

}

unsigned int I2C_readReg16(unsigned char slave, unsigned char add)
{
    unsigned int value;
    unsigned char i;
    for (i = 0; i < 2 ; i++)
    {
        *((unsigned char *)&value+i) = I2C_readReg8(slave,add+i);
    }
    return value;
}
void I2C_readMany(unsigned char slave, unsigned char add, unsigned char count, unsigned char* buffer)
{
    unsigned char i=0;
    for (i = 0; i < count ; i++)
    {
        *(buffer+i) = I2C_readReg8(slave,add+i);
    }
}
