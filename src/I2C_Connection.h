/*
 * I2C_Connection.h
 *
 *  Created on: 14-apr.-2020
 *      Author: Benjamin
 */

#ifndef SRC_I2C_CONNECTION_H_
#define SRC_I2C_CONNECTION_H_

#include <stdbool.h>
void setupI2C(void);
void IIC_Reset(void);
bool I2C_WriteBuffer(uint8_t i2cAddress, uint8_t * wBuffer, uint8_t wLength);
bool I2C_ReadBuffer(uint8_t i2cAddress, uint8_t regCommand, uint8_t * rBuffer, uint8_t rLength);
bool I2C_WriteReadBuffer(uint8_t i2cAddress, uint8_t * wBuffer, uint8_t wLength, uint8_t * rBuffer, uint8_t rLength);

#endif /* SRC_I2C_CONNECTION_H_ */
