/*
 * PWM_LED.c
 *
 *  Created on: 17-apr.-2020
 *      Author: Benjamin
 */


#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "em_letimer.h"
#include <stdbool.h>

#define BUFFER_SIZE 1//10
// Note: change this to change the duty cycles used
static const uint16_t dutyCyclePercentages[BUFFER_SIZE] = {100}; //  {0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
// Define Port and number for PWM
#define PWM_PORT gpioPortB
#define PWM_PORTNUMBER 11
// PortB 11 is at LOCATION 3 , Channel2, Timer1.
// PortD 6 is at LOCATION 4 , Channel0, Timer1.

// GPIO 1 of motherboard set for Power control of sensors.
#define PORT_SENSORS gpioPortA
#define PORTNUMBER_SENSORS 0
// GPIO 3 of motherboard set for Power control of sensors.
#define PORT_BLE gpioPortE
#define PORTNUMBER_BLE 13

// GPIO 2 CONTROL LED
#define PORT_controlLED gpioPortF
#define PORTNUMBER_controlLED 2


bool ledAAN = false;
uint8_t counterPWM=0;

void setPortON(void){
	// Enable GPIO and clock
	CMU_ClockEnable(cmuClock_GPIO, true);

	// Configure PB11 as output
	GPIO_PinModeSet(PWM_PORT, PWM_PORTNUMBER, gpioModePushPull, 1);


	// disable GPIO and clock
	CMU_ClockEnable(cmuClock_GPIO, false);
}

void setPortOFF(void){
	// Enable GPIO and clock
	CMU_ClockEnable(cmuClock_GPIO, true);

	// Configure PB11 as output
	GPIO_PinModeSet(PWM_PORT, PWM_PORTNUMBER, gpioModePushPull, 0);


	// disable GPIO and clock
	CMU_ClockEnable(cmuClock_GPIO, false);
}

/*Bij RTC = 0.5 mS werken we op 200Hz.*/
void PWMSteering(void)
{
	if (dutyCyclePercentages[0]==100 ) { // Altijd aan
		// Na gpio initialisatie, niets nodig
	}
	else{
		if ( (counterPWM == 0) & (dutyCyclePercentages[0]!=0) ) {
			setPortON();
			counterPWM++;
		}
		else if (counterPWM == (dutyCyclePercentages[0]/10) ){
			setPortOFF();
			counterPWM++;
		}
		else if(counterPWM == 10){
			counterPWM=0;
		}else{
			counterPWM++;
		}
	}
}

bool isPWMNeeded(void){

	// Configure PB11 as output
	if(dutyCyclePercentages[0] != 0){
	  GPIO_PinModeSet(PWM_PORT, PWM_PORTNUMBER, gpioModePushPull, 1);
	} else {
	  GPIO_PinModeSet(PWM_PORT, PWM_PORTNUMBER, gpioModePushPull, 0);
	}

	if( (dutyCyclePercentages[0] != 100)& (dutyCyclePercentages[0] != 0)){
		return true;
	} else{
		return false;
	}
}
void SetPowerBLE(bool status) { // if status = true, set ON else set OFF
	if(status){
		// Enable GPIO and clock
		CMU_ClockEnable(cmuClock_GPIO, true);

		// Configure PE13_BLE as output
		GPIO_PinModeSet(PORT_BLE, PORTNUMBER_BLE, gpioModePushPull, 1);

		// disable GPIO and clock
		CMU_ClockEnable(cmuClock_GPIO, false);
	}else{
		// Enable GPIO and clock
		CMU_ClockEnable(cmuClock_GPIO, true);

		// Configure PE13_BLE as output
		GPIO_PinModeSet(PORT_BLE, PORTNUMBER_BLE, gpioModePushPull, 0);

		// disable GPIO and clock
		CMU_ClockEnable(cmuClock_GPIO, false);
	}
}

void SetPowerSensors(bool status){
	if(status){
		// Enable GPIO and clock
		CMU_ClockEnable(cmuClock_GPIO, true);

		// Configure PE13_BLE as output
		GPIO_PinModeSet(PORT_SENSORS, PORTNUMBER_SENSORS, gpioModePushPull, 1);

		// disable GPIO and clock
		CMU_ClockEnable(cmuClock_GPIO, false);
	}else{
		// Enable GPIO and clock
		CMU_ClockEnable(cmuClock_GPIO, true);

		// Configure PE13_BLE as output
		GPIO_PinModeSet(PORT_SENSORS, PORTNUMBER_SENSORS, gpioModePushPull, 0);

		// disable GPIO and clock
		CMU_ClockEnable(cmuClock_GPIO, false);
	}
}
