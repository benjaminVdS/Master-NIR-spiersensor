/*
 * PWM_LED.h
 *
 *  Created on: 17-apr.-2020
 *      Author: Benjamin
 */

#ifndef SRC_PWM_LED_H_
#define SRC_PWM_LED_H_

void PWMSteering(void);
bool isPWMNeeded(void);

void SetPowerBLE(bool status);
void SetPowerSensors(bool status);
#endif /* SRC_PWM_LED_H_ */
