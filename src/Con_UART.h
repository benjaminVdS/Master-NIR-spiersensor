/*
 * Con_UART.h
 *
 *  Created on: 3-mrt.-2020
 *      Author: Benjamin
 */

#ifndef SRC_CON_UART_H_
#define SRC_CON_UART_H_

/* Function prototypes */
void uartSetup(void);
void cmuSetup(void);
void uartPutData(uint8_t * dataPtr, uint32_t dataLen);
uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen);
void    uartPutChar(uint8_t charPtr);
uint8_t uartGetChar(void);

#endif /* SRC_CON_UART_H_ */
