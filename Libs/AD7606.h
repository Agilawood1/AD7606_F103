#ifndef _AD7606_H
#define _AD7606_H

#include "stdarg.h"
#include "stdint.h"

extern uint8_t need_send;
extern uint8_t tx_buf[36];

void ADInit();
void process_uart_transmit(void);

#endif
