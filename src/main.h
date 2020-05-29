/*
 * main.h
 *
 *  Created on: 10-apr.-2020
 *      Author: Benjamin
 */

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_


void delayMs(int ms);
void SysTick_Handler(void);
void RTC_setOut(void);
void RTC_setOn(void);
void RTC_setup(void);
void RTC_IRQHandler(void);
int main(void);
//void DMATxCallback(void);
void initDMA(void);
void startListening();
void stopListening();
void tx(const uint8_t *bytes, unsigned n);
void LEUART0_IRQHandler();
void initLEUART();
void setConversie();
void getTempData();



#endif /* SRC_MAIN_H_ */
