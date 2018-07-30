/*
 * app_trace.h
 *
 *  Created on: 2018Äê7ÔÂ13ÈÕ
 *      Author: Administrator
 */

#ifndef APP_TRACE_H_
#define APP_TRACE_H_
#include <stdio.h>
#include <string.h>

#define UART_PRINTF

#ifdef UART_PRINTF
int fputc(int _c, register FILE *_fp);
int fputs(const char *_ptr, register FILE *_fp);
#define app_trace_log       printf
#else
#define app_trace_log(...)
#endif

#endif /* APP_TRACE_H_ */
