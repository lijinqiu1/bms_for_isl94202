/*
 * app_trace.c
 *
 *  Created on: 2018Äê7ÔÂ13ÈÕ
 *      Author: Administrator
 */
#include <msp430.h>
#include "app_trace.h"

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
