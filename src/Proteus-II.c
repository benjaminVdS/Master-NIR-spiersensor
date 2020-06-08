/*
 * Proteus-II.c
 *
 *  Created on: 6-apr.-2020
 *      Author: Benjamin
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "PWM_LED.h"
#include "em_leuart.h"
#include "Proteus-II.h"       /* Corresponding header file */
#include "debug_dbprint.h" /* Enable or disable printing to UART for debugging */
#include "em_dma.h"
#include "main.h"
/*-------------INSTELBARE PARAMETERS!------------------------------------------*/
char Tablet_BTMAC[6]={0x55,0x49,0x4F,0xB4,0x00,0xFB}; // BTMAC van gebruikte device (wordt ingelezen en veranderd automatisch)
// pins for the LEDs:
const int ResetPin = 2;
bool endoftransmission = false;
bool Setup = true;
bool ConnectionEstablished = false;
bool Connection = false;
char parameter[16];
bool firstConnection = true;
#define CUSTOM_BOARD 0 // HAPPY GECKO
/*---------------------END-----------------------------------------------------*/
/*-------------DEFINE VASTE COMMANDO's BLE-module PROTEUS-II-------------------*/
//Startsignal
#define startS 				0x02
#define	DMA_CHANNEL 0
//Commando's
#define CMD_RESET_REQ 		0x00
#define CMD_GETSTATE_REQ 	0x01
#define CMD_SLEEP_REQ 		0x02
#define CMD_DATA_REQ 		0x04
#define CMD_CONNECT_REQ 	0x06
#define CMD_DISCONNECT_REQ 	0x07
#define CMD_DISCONNECT_REQ 	0x07
#define CMD_SCANSTART_REQ 	0x09 // Start scan
#define CMD_SCANSTOP_REQ 	0x0A // Stop scan
#define CMD_GETDEVICES_REQ 	0x0B // Request the scanned/detected devices
#define CMD_SETBEACON_REQ 	0x0C // Place data in scan response packet
#define CMD_PASSKEY_REQ 	0x0D // Respond to a pass key request
#define CMD_DELETEBONDS_REQ 0x0E // Delete bonding information
#define CMD_GETBONDS_REQ  	0x0F // Read the MACs of bonded devices
#define CMD_GET_REQ  		0x10
#define CMD_SET_REQ  		0x11
#define CMD_PHYUPDATE_REQ   0x1A // Update the PHY
#define CMD_UARTDISABLE_REQ 0x1B // Disable the UART
#define CMD_FACTORYRESET_REQ 0x1C// First, the default User Settings are restored, then the module is reset.
#define CMD_DTMSTART_REQ 	0x1D // Enable the direct test mode
#define CMD_DTM_REQ  		0x1E // Start/stop a test of the direct test mode
#define CMD_BOOTLOADER_REQ  0x1F // Switch to the bootloader
// PayloadLengths
//char LengthPayload0[2]={0x00,0x00};
char LengthPayload1[2]={0x01,0x00};
char LengthPayload2[2]={0x02,0x00};
char LengthPayload3[2]={0x03,0x00};
char LengthPayload6[2]={0x06,0x00};
char LengthPayload14[2] = {0x0E, 0x00};
char LengthPayload16[2]={0x10,0x00};
char LengthPayload242[2]={0xF2,0x00};
char LengthPayload243[2]={0xF3,0x00};
// Parameter_Settings
#define RF_DeviceName 		0x02 // Set name
#define FS_BTMAC 			0x04 // 0x04 in VB en tabel
#define UART_BaudrateIndex 	0x0B // 0x0B in VB ,0x11 in tabel
#define RF_SecFlags 		0x0C // 0x0C in Vb ; 0x12 in Tabel...
#define RF_ScanFlags 		0x0D // 0x0D in Vb , 0x13 in Tabel
#define RF_TxPower			0x11 // 0x11 in VB en 0x17 in tabel
#define RF_SPPBaseUUID  	0x1A // 0x1A in VB en 0x26 in Tabel;
#define RF_ScanTiming  		0x09 // Laat default
#define RF_ConnectionTiming 0x08 //
#define RF_AdvertisingTimeout 0x07 //


/*---------------------END-----------------------------------------------------*/
#define BUF_MAX        247
volatile char rxbuf[BUF_MAX];
volatile char tx_buffer[BUF_MAX];
uint32_t leuartif;
uint32_t len;
volatile uint32_t rx_data_ready = 0;

void initiateBLE(void){

//-------------------------------SET PARAMETERS----------------------------------------------//
/* set UART_Baudrateindex to 9600 (LEUART MAX)*/
#if DEBUG_DBPRINT == 1
	dbprintln("set UART_Baudrateindex");
#endif
	parameter[0] = 0x00;
	sendCommandtoDEV(startS, CMD_SET_REQ, LengthPayload2 , UART_BaudrateIndex, parameter);

/* Set RF_SecFlags Security protocol to generate random keys*/
#if DEBUG_DBPRINT == 1
	dbprintln("Set RF_SecFlags to 0x02: bij conn genereer random keys!, bonding_enable staat aan.");
#endif /* DEBUG_DBPRINT */
	parameter[0] = 0x0A;
	sendCommandtoDEV(startS, CMD_SET_REQ, LengthPayload2 , RF_SecFlags, parameter);

/* Set RF_TxPower to 0dBm */
#if DEBUG_DBPRINT == 1
	dbprintln("Set Tx_power: 0dBm");
#endif
	parameter[0] = 0x00;
	sendCommandtoDEV(startS, CMD_SET_REQ, LengthPayload2 , RF_TxPower, parameter);

/*Set RF_DeviceName: MUSCLE SENSOR */
#if DEBUG_DBPRINT == 1
	dbprintln("Set RF_DeviceName: MUSCLE SENSOR ");
#endif
	char parameter13[13] = {0x4d,0x75,0x73,0x63,0x6c,0x65,0x20,0x73,0x65,0x6e,0x73,0x6f,0x72}; // 13 lang
	sendCommandtoDEV(startS, CMD_SET_REQ, LengthPayload14 , RF_DeviceName, parameter13);

/*RF_ScanTiming to advertising window = 1 s with scan window = 1250ms. Connection setup timeout = 5 s*/
#if DEBUG_DBPRINT == 1
	dbprintln("Set RF_ScanTiming to 0x03");
#endif
	parameter[0] = 0x03;
	sendCommandtoDEV(startS, CMD_SET_REQ, LengthPayload2 , RF_ScanTiming, parameter);

/* Set RF_ConnectionTiming to 750-2250 mS */
#if DEBUG_DBPRINT == 1
	dbprintln("Set RF_ConnectionTiming to 0x04: 750-2250 mS");
#endif
	parameter[0] = 0x04;
	sendCommandtoDEV(startS, CMD_SET_REQ, LengthPayload2 , RF_ConnectionTiming, parameter);

//RF_AdvertisingTimeout 600 S
#if DEBUG_DBPRINT == 1
	dbprintln("Set RF_AdvertisingTimeout: 600 S");
#endif
	parameter[0] = 0x58;
	parameter[1] = 0x02;
	sendCommandtoDEV(startS, CMD_SET_REQ, LengthPayload3 , RF_AdvertisingTimeout, parameter);

//-------------------------------GET PARAMETERS----------------------------------------------//
/* Get UART_Baudrateindex */
#if DEBUG_DBPRINT == 1
	dbprintln("Get UART_Baudrateindex");
#endif
	sendCommandtoDEV(startS, CMD_GET_REQ, LengthPayload1 , UART_BaudrateIndex, parameter);

/* Ask for UUID */
#if DEBUG_DBPRINT == 1
	dbprintln("Ask for UUID");
#endif
	sendCommandtoDEV(startS, CMD_GET_REQ, LengthPayload1 , RF_SPPBaseUUID, parameter);

/* Get Tx_power: */
#if DEBUG_DBPRINT == 1
	dbprintln("Get Tx_power: ");
#endif
	parameter[0] = 0x00;
	sendCommandtoDEV(startS, CMD_GET_REQ, LengthPayload1 , RF_TxPower, parameter);

/* Get RF_SecFlags */
#if DEBUG_DBPRINT == 1
    dbprintln("Get RF_SecFlags");
#endif
    sendCommandtoDEV(startS, CMD_GET_REQ, LengthPayload1 , RF_SecFlags, parameter);

/* Get FS_BTMAC */
#if DEBUG_DBPRINT == 1
    dbprintln("Get FS_BTMAC");
#endif /* DEBUG_DBPRINT */
    sendCommandtoDEV(startS, CMD_GET_REQ, LengthPayload1 , FS_BTMAC, parameter);
}

void sendCommandtoDEV(char startsignal, char command, char Length[2], char SettingIndex, char* parameter){
	char CS = startsignal ^ command ^ Length[0] ^ Length[1];
	char message[5 + Length[0] + (Length[1]<<8)]; // <<8 Shift bit 8 positions left
	memset(message, 0, sizeof(message));
	message[0]= startsignal;
	message[1]= command;
	message[2]= Length[0];
	message[3]= Length[1];

	if(Length[0] == 0x00 && Length[1]== 0x00){
		message[4]= CS;
	}
	else if(Length[0] == 0x01 && Length[1]== 0x00){ //Send 0x10 command! //Setting index and CS toevoegen
		CS = CS ^ SettingIndex;
		message[4]= SettingIndex;
		message[5]= CS;
	}

	else if(Length[0] > 30 && Length[1]== 0x00){ //243 Sending full buffer of data! Geen parameter!
		for(int i=4; i <= (4 + (Length[0] + (Length[1]<<8))); i++ ){ // 4+243=247
			if( i <= 4 + ( Length[0] + (Length[1]<<8))-1 ){ // 4+243-1 = 246
				CS = CS ^ parameter[i-4];
				message[i] = parameter[i-4];
			}
			else { // 247de is CS = correct.
				message[i] = CS;
			}
		}
	}

	else{
	  message[4]= SettingIndex;
	  CS = CS ^ SettingIndex;

		for(int i=5; i <= (4 + (Length[0] + (Length[1]<<8))); i++ ){ // 4+6=10
			if( i <= 4 + ( Length[0] + (Length[1]<<8))-1 ){ // 4+6-1 = 9
				CS = CS ^ parameter[i-5];
				message[i] = parameter[i-5];
			}
			else { // 10de is CS = correct.
				message[i] = CS;
			}
		}
	}
	tx(&message, sizeof(message));

	//memset(&rxbuf, 0, sizeof(rxbuf));
	memset(&message, 0, sizeof(message));
	//LEUART_IntSet(LEUART0,LEUART_IF_TXC); // set flag to send data!
}


void sendTest(char* parameter){
	char CS = startS ^ CMD_DATA_REQ ^ 0xF3;
	char message[5 + 243]; // <<8 Shift bit 8 positions left
	memset(message, 0, sizeof(message));
	message[0]= startS;
	message[1]= CMD_DATA_REQ;
	message[2]= 0xF3;
	message[3]= 0x00;
	for(int i=4; i <= (4 + 243); i++ ){ // 4+243=247
		if( i <= 4 + 243 -1 ){ // 4+243-1 = 246
			CS = CS ^ parameter[i-4];
			message[i] = parameter[i-4];
		}
		else { // 247de is CS = correct.
			message[i] = CS;
		}
	}
	tx(&message, sizeof(message));
	memset(&message, 0, sizeof(message));
}

void receivedConnection(char *message){
	if(message[1] == 0x86){
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	dbprintln("Connection has been established");
#endif
		ConnectionEstablished = false;
		// Set power to sensors OFF
		SetPowerSensors(false);
	}
	else if(message[1] == 0x87){
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	dbprintln("Connection was broken");
#endif
		ConnectionEstablished = false;
		// Set power to sensors OFF
		SetPowerSensors(false);
	}
	else if(message[1] == 0x88){
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	dbprintln("CMD_SECURITY_IND received. Nw secure connection established.");
#endif
		ConnectionEstablished = false;
		// Set power to sensors OFF
		SetPowerSensors(false);
	}
	else if(message[1] == 0xC6){
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	dbprintln("Data can be transmitted now");
#endif
		ConnectionEstablished = true;
		// Set power to sensors ON
		SetPowerSensors(true);
	}
}


bool getConnectionEstablished(void){
	return ConnectionEstablished;
}
