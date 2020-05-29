/*
 * Ic2_Connection.c
 *
 *  Created on: 14-apr.-2020
 *      Author: Benjamin
 */
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_dma.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_int.h"
#include "em_i2c.h"
#include "i2cspm.h"
#include "dmactrl.h"
#include "bsp.h"
/* The I2C read bit is OR'ed with the address for a read operation */
#define I2C_READ_BIT 0x01
#define I2C_Write_BIT 0x01
/* The I2C address of the EEPROM */
#define EEPROM_I2C_ADDR 0xD0 // 0b1101000

/* DMA channels */ //0,1 used for LEUART
#define DMA_CHANNEL_I2C_TX 2
#define DMA_CHANNEL_I2C_RX 3

/* I2C configuration. Corresponds to the I2C bus on the EFM32GG-DK3750. */
#define I2C_SCL_PIN  1
#define I2C_PORT gpioPortC
#define I2C_SDA_PIN  0
#define I2C_LOCATION 4

/******************  IIC  **************************************/

I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;


void setupI2C(void){
	// settings i2c
	i2cInit.port = I2C0;
	i2cInit.sclPort = I2C_PORT;
	i2cInit.sclPin = I2C_SCL_PIN;
	i2cInit.sdaPort = I2C_PORT;
	i2cInit.sdaPin = I2C_SDA_PIN;
	i2cInit.i2cClhr = i2cClockHLRStandard;
	i2cInit.i2cRefFreq = 0 ;
	i2cInit.i2cMaxFreq = 100000; //lowerd standard frequency - I2C_FREQ_STANDARD_MAX;
	i2cInit.portLocation = I2C_LOCATION; //to be found in datasheet of controller

	I2CSPM_Init(&i2cInit);

}

void IIC_Reset(void){
	I2C_Reset(i2cInit.port);
}




bool I2C_WriteBuffer(uint8_t i2cAddress, uint8_t * wBuffer, uint8_t wLength){
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_read_data[0]; //leeg

	seq.addr = i2cAddress;
	seq.flags = I2C_FLAG_WRITE;

	seq.buf[0].data = wBuffer;
	seq.buf[0].len = wLength;

	seq.buf[1].data = i2c_read_data;
	seq.buf[1].len = 0;

	ret = I2CSPM_Transfer(i2cInit.port, &seq);
	if(ret != i2cTransferDone){
		return false;
	}
	return true;
}
bool I2C_ReadBuffer(uint8_t i2cAddress, uint8_t regCommand, uint8_t * rBuffer, uint8_t rLength){
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_write_data[1]; //empty array [0] and [1]

	seq.addr = i2cAddress;
	seq.flags = I2C_FLAG_READ;
	i2c_write_data[0] = regCommand;
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = 1;

	seq.buf[1].data = rBuffer;
	seq.buf[1].len = rLength;

	ret = I2CSPM_Transfer(i2cInit.port, &seq);
	if(ret != i2cTransferDone){
		*rBuffer=0;
		return false;
	}
	return true;
}
bool I2C_WriteReadBuffer(uint8_t i2cAddress, uint8_t * wBuffer, uint8_t wLength, uint8_t * rBuffer, uint8_t rLength){
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;

	seq.addr = i2cAddress;
	seq.flags = I2C_FLAG_WRITE_READ;

	seq.buf[0].data = wBuffer;
	seq.buf[0].len = wLength;

	seq.buf[1].data = rBuffer;
	seq.buf[1].len = rLength;

	ret = I2CSPM_Transfer(i2cInit.port, &seq);

	if(ret != i2cTransferDone){
		*rBuffer = 0;
		return false;
	}
	return true;
}
