#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_leuart.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_rtc.h"
#include "em_rtcc.h"
#include "em_timer.h"
#include "em_i2c.h"
#include "em_int.h"
#include "bsp.h"
#include "dmactrl.h"
#include "dmadrv.h"

#include "debug_dbprint.h" /* Enable or disable printing to UART for debugging */
#include "Proteus-II.h"
#include "main.h"
#include "I2C_Connection.h"
#include "PWM_LED.h"

/* DEFINES */
#define DMA_CHANNEL    0
#define BUF_MAX        247

/*RTC VRIABLES!!!*/
#define RTC_FREQ 32768
#define WAKEUP_INTERVAL_MS_PWM	0.5
#define WAKEUP_INTERVAL_MS	16.529
#define WAKEUP_INTERVAL_MS_IDLE	1000
#define RTC_COUNT_BETWEEN_WAKEUP_PWM	(((RTC_FREQ * WAKEUP_INTERVAL_MS_PWM) / 1000)-1)
#define RTC_COUNT_BETWEEN_WAKEUP	(((RTC_FREQ * WAKEUP_INTERVAL_MS) / 1000)-1)
#define RTC_COUNT_BETWEEN_IDLE	(((RTC_FREQ * WAKEUP_INTERVAL_MS_IDLE) / 1000)-1)
uint32_t counterI2C = 0;
bool StartConnection = true;
bool PWM_Enabled = false;
bool dataChannelOpen = false;
/* GLOBAL VARIABLES */
volatile uint32_t msTicks; /* counts 1ms timeTicks */
uint32_t leuartif;
uint32_t len;
char MCUstate;

/*-----------------Buffers------------------------*/
int32_t rxBufferSize = 64;
char rxBuffer [64];

// DMA globals
static unsigned LEUARTDmaRxChannel;
static unsigned LEUARTDmaTxChannel;
static DMA_CfgDescr_TypeDef LEUARTDmaRxCfgDescr, LEUARTDmaTxCfgDescr;
static DMA_CfgChannel_TypeDef LEUARTDmaRxCfgChannel, LEUARTDmaTxCfgChannel;
DMA_CB_TypeDef cb;

// RECEIVE GLOBALS
char receivedMessage1[64];



/*---------------I2C_NIR-SENSORMODULE--------------*/
/* I2C error flag */
static volatile bool i2cError = false;
/* The I2C address of the EEPROM */
#define EEPROM_I2C_ADDR 0b1101000
#define TSENSOR_I2C_ADDR 0b1001001
char txData;
/* Receive buffer */
static uint8_t rxData[64]; // Receive register,LSByte, MSByte for NIR-Sensormodule
char bufferCH0[243];
char bufferCH1[243];
bool oneShotConversion=false;
bool bit12= true; // bit 12 & 14 = false => 16 bit Conversion!
bool bit14 = false;
uint8_t counterBuffer = 1;
uint8_t counterCH0 = 1;
uint8_t counterCH1 = 1;
uint8_t channel = 0; // 0, 1
uint8_t gain = 1; // 1,2,4
char Configuration = 0b00000000;
bool setCH = true;
/*-----------------------END-----------------------*/


/* Energy mode enumerations */
typedef enum {
  SLEEP,
  SendDataBLE,
  SendDataI2C,
  SendDataTemp,
} Sensor_Mode_T;
static Sensor_Mode_T mode = SLEEP;

LEUART_Init_TypeDef leuart0Init =
{
        .enable   = leuartEnable,       /* Activate data reception on LEUn_TX pin. */
        .refFreq  = 0,                    /* Inherit the clock frequenzy from the LEUART clock source */
        .baudrate = 9600,                 /* Baudrate = 9600 bps */
        .databits = leuartDatabits8,      /* Each LEUART frame containes 8 databits */
        .parity   = leuartNoParity,       /* No parity bits in use */
        .stopbits = leuartStopbits1,      /* Setting the number of stop bits in a frame to 2 bitperiods */
};



/**********************************************************
 * Delay a number of milliseconds
 **********************************************************/
void delayMs(int ms)
{
  uint32_t endValue = ms * RTC_FREQ / 1000;
  RTC->CNT = 0;

  RTC->CTRL |= RTC_CTRL_EN;

  while ( RTC->CNT < endValue );

  RTC->CTRL &= ~RTC_CTRL_EN;
}


/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
    msTicks++;       /* increment counter necessary in Delay()*/
}

void RTC_setOut(void) {
	RTC_FreezeEnable(true);
}
void RTC_setOn(void) {
	RTC_FreezeEnable(false);
}
void RTC_setup(void) {
	// Set RTC compare value for RTC 0
	RTC_CompareSet(0,RTC_COUNT_BETWEEN_IDLE );//RTC_COUNT_BETWEEN_WAKEUP
	// Allow channel 0 to cause an interrupt
	RTC_IntEnable(RTC_IFC_COMP0); // RTC_IEN_COMP1 voor channel 1?
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_EnableIRQ(RTC_IRQn);

	// Allow channel 1 to cause an interrupt
	// Configure the RTC settings
	RTC_Init_TypeDef rtc = RTC_INIT_DEFAULT;

	// Initialise RTC with pre-defined settings
	RTC_Init(&rtc);
}

void RTC_IRQHandler(void){ // Interrupt handler om de 8µS om data van I2C in te lezen. Enkel aan indien DataCHOpen commando is ontvangen.
	mode = SLEEP;
	//dataChannelOpen = getConnectionEstablished(); // Place in LEUART_IRQ to check on interrupt after reading in the commands.

	// Als er een connectie is met dataChannelOpen.
	// Zet de Temperatuur en NIR-SENSORMODulES AAN.
	if(dataChannelOpen){
		if(StartConnection){
			RTC_CompareSet(0,RTC_COUNT_BETWEEN_WAKEUP );//RTC_COUNT_BETWEEN_WAKEUP = 16.459mS
			PWM_Enabled = isPWMNeeded(); // Check if PWM is needed or not. Set RTC_Clock depending on it!
			if (PWM_Enabled) RTC_CompareSet(0,RTC_COUNT_BETWEEN_WAKEUP_PWM ); // 0.5mS
			// Set power to sensors
			// .....
			StartConnection=false;
		}

		if (PWM_Enabled){
			PWMSteering();
		}


		// Zet alle GPIO pinnen aan/uit voor voeding!

		/// if sensorson = false ... Zet modules aan.
		//.....
		//Switch channel
		if(PWM_Enabled){ // Snellere clock van 0.5mS
			if(counterI2C >= 33){
				if(setCH){
					setCH = false;
				}else{
					setCH = true;
				}
				counterI2C = 0;
				mode = SendDataI2C;
			}else{
				counterI2C++;
			}
		} else { // Clock van normale snelheid 16.5mS
			if(setCH){
				setCH = false;
			}else{
				setCH = true;
			}
			counterI2C = 0;
			mode = SendDataI2C;
		}
	} else {
		if (!StartConnection){
			RTC_CompareSet(0,RTC_COUNT_BETWEEN_IDLE );//RTC_COUNT_BETWEEN_WAKEUP
			// Set power of sensor off!
			// ....
			StartConnection=true;
		}
		// NO dataChannelOpen has been received.
	}

	/* Clear interrupt source */
	RTC_IntClear(RTC_IFC_COMP0);
}

int main(void)
{
    /* Chip errata */
    CHIP_Init();
	/* Enable access to the I2C bus on the DK */
	BSP_Init(BSP_INIT_DEFAULT);
    // PB7 and PB8 are the LFXO pins, LEUART
	// disable them so GPIO can't mess with them
	GPIO_PinModeSet(gpioPortB, 7, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 8, gpioModeDisabled, 0);

    /* Setup SysTick Timer for 1 msec interrupts  */
    //if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;
    // Start LFXO
	CMU->CTRL |= CMU_CTRL_LFXOBOOST_100PCENT;
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

	 // Turn on the clock for Low Energy clocks
    CMU_ClockEnable(cmuClock_HFLE, true);
    // LFB is needed by LEUART
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	//CMU_ClockEnable(cmuClock_LFB, true);

    /********Enable Clock to MCU core ****/
    CMU_ClockEnable(cmuClock_CORELE, true);
    /* Enable DMA clock */
    CMU_ClockEnable(cmuClock_DMA, true);
    /*********Enable clocks to GPIO**********/
    //CMU_ClockEnable(cmuClock_GPIO, true);
    /* Enable LEUART0 clock */
    CMU_ClockEnable(cmuClock_LEUART0, true);
    //Enable RTC clock
	CMU_ClockEnable(cmuClock_RTC, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
 	//Setup-rtc voor elke 4s data door te sturen.
	RTC_setup();
    //Initialize the low-energy-UART
	initLEUART();
	//Initialize the DMA for LEUART
    initDMA();
//Initialize DBUG PRINT
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	dbprint_INIT(USART1, 4, true, false); /* Initialize dbprint for use with VCOM */
	dbprintln("Initialisatie:");
#endif /* DEBUG_DBPRINT */

	startListening(); // Set-on LEUART
	// Initialise I2C
	setupI2C();
 	initiateBLE();
 	SetPowerBLE(true);
 	SetPowerSensors(false);
/* Infinite loop */
//char LengthPayload243[2] = {0xF3,0x00} ;
while (1) {
	switch(mode){
		case SLEEP:
			//do {
				EMU_EnterEM2(true);
			//}while(mode == SLEEP);

			break;
		case SendDataBLE:
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	dbprint(bufferCH0 );
	dbprint(bufferCH1 );
#endif /* DEBUG_DBPRINT */
			sendTest(bufferCH0);
			sendTest(bufferCH1);
			mode =SLEEP;
			break;

		case SendDataI2C:
			/* Clear error flag. Will be set on any error during transmission. */
			i2cError = false;
			uint8_t wBufferTemp[0];
			uint8_t rBufferTemp[3];
			if(counterCH0 <= 239) { // place 241,242 reserved for battery Voltage or T-sensor. Depending on the channel.
				if(setCH){ // Read CH0
					/*Read data in from NIR-sensor*/ // CH0
					i2cError=I2C_WriteReadBuffer(EEPROM_I2C_ADDR <<1 , wBufferTemp, 1, rBufferTemp, 3);//R CH0
					if(i2cError){
						bufferCH0[counterCH0]=rBufferTemp[1];   // LSB
						bufferCH0[counterCH0+1]=rBufferTemp[0]; // MSB
						counterCH0 = counterCH0+ 2;

					}else{
						/*Something went wrong -> Good connection?*/
					}
					/*Set configuration CH1*/
					channel = 1;
					setConversie();
					wBufferTemp[0] = Configuration;
					/*set register*/
					i2cError=I2C_WriteBuffer( EEPROM_I2C_ADDR <<1 ,wBufferTemp, 1); // W CH1
					if(i2cError){

					}else{
						/*Something went wrong -> Good connection?*/
					}

				}else{ // Read CH1
					/*Read data in from NIR-sensor*/
					i2cError=I2C_WriteReadBuffer(EEPROM_I2C_ADDR <<1 , wBufferTemp, 1, rBufferTemp, 3); // R CH1
					if(i2cError){
						bufferCH1[counterCH1]=rBufferTemp[1];   // LSB
						bufferCH1[counterCH1+1]=rBufferTemp[0]; // MSB
						counterCH1 = counterCH1 + 2;
					}else{
						/*Something went wrong -> Good connection?*/
					}
					/*Set configuration CH0*/
					channel = 0;
					setConversie();
					wBufferTemp[0] = Configuration;
					i2cError=I2C_WriteBuffer( EEPROM_I2C_ADDR <<1 ,wBufferTemp, 1); // W CH0
					if(i2cError){

					}else{
						/*Something went wrong -> Good connection?*/
					}
				}

				mode = SLEEP;
			}
			else{
				// Read temperature
				// Reset counters
				counterCH1= 1;
				counterCH0=1;
				// Set mode to send
				mode = SendDataTemp; //SendDataBLE
			}
			break;

		case SendDataTemp: // 241 & 242 fill with data
			getTempData();
			mode = SendDataBLE;
			break;
		//default:
	}
}


}
void getTempData(){
//TSENSOR_I2C_ADDR
	uint8_t wBuff[2];
	uint8_t rBuff[3];
	wBuff[0] = 0;
	/*Read data in from NIR-sensor*/
	i2cError=I2C_WriteReadBuffer(TSENSOR_I2C_ADDR <<1 , wBuff, 1, rBuff, 2); // R CH1
	bufferCH0[241]=rBuff[0];//MSB
	bufferCH0[242]=rBuff[1];//LSB
	float Temperature = ( (rBuff[1]>>5) + (rBuff[0]<<3) )*0.125; // °C (nauwkeurigheid
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	dbprintln("T-sensor: ");
	dbprintInt((int)Temperature);
#endif /* DEBUG_DBPRINT */
	//wBuff[0] = 1;
	/*Read data in from NIR-sensor*/
	/*i2cError=I2C_WriteReadBuffer(TSENSOR_I2C_ADDR <<1 , wBuff, 1, rBuff, 1);
	wBuff[0] = 4;*/
	/*Read data in from NIR-sensor*/
	/*i2cError=I2C_WriteReadBuffer(TSENSOR_I2C_ADDR <<1 , wBuff, 1, rBuff, 1);
	wBuff[1]=0b00011111;
	i2cError=I2C_WriteBuffer(TSENSOR_I2C_ADDR <<1 , wBuff, 2);*/

}
void setConversie(){
	if (oneShotConversion){
	  Configuration |= 1UL << 7; // set bit 7!
	  Configuration &= ~(1UL << 4); // clear bit 4
	} else {
		Configuration |= 1UL << 4; // set bit 4!
	}// set Continueous conversion true

	// Gain bit 0,1! 00 = x1 | 01 = x2 | 10= x4 | 11 = x8
	if(gain==1){
	Configuration &= ~(1UL << 0); // clear bit 0
	Configuration &= ~(1UL << 1); // clear bit 1
	} else if (gain ==2){
	  Configuration |= 1UL << 0; // set bit 0!
	  Configuration &= ~(1UL << 1); // clear bit 1
	} else{ // set gain to x4.
	  Configuration |= 1UL << 1; // set bit 1!
	  Configuration &= ~(1UL << 0); // clear bit 0
	}

	if (bit12){
	  Configuration &= ~(1UL << 2); // clear bit 2
	  Configuration &= ~(1UL << 3); // clear bit 3
	}
	else if(bit14){
	Configuration |= 1UL << 2; // set bit 2
	Configuration &= ~(1UL << 3); // clear bit 3
	}
	else{ //16bit!
	  Configuration &= ~(1UL << 2); // clear bit 2
	  Configuration |= 1UL << 3; // set bit 3
	}

	if (channel == 0){
	  Configuration &= ~(1UL << 5); // clear bit 2
	  Configuration &= ~(1UL << 6); // clear bit 3
	}
	else {
	  Configuration |= 1UL << 5; // set bit 5
	  Configuration &= ~(1UL << 6); // clear bit 6
	}

	if (channel == 0){
	 bufferCH0[0] = Configuration;
	}
	if (channel == 1){
	 bufferCH1[0] = Configuration;
	}
}
/**************************************************************************//**
 * @brief  Tx DMA Callback function
 *****************************************************************************/
void DMATxCallback(void){
#if DEBUG_DBPRINT == 1 /* DEBUG_DBPRINT */
	dbprintln("iitts");
#endif /* DEBUG_DBPRINT */

}
/**************************************************************************//***
 * @brief  DMA init function
 ******************************************************************************/
void initDMA(void){
    // Set up 2 DMA channels!!! to send and receive
    CMU_ClockEnable(cmuClock_DMA, true);
	DMADRV_Init();

	// Allocate DMA channels
	/*if (ECODE_EMDRV_DMADRV_OK != DMADRV_AllocateChannel(&LEUARTDmaRxChannel, NULL)) {
		printf("DMA channel allocation failure for LEUART RX!");
	}
	if (ECODE_EMDRV_DMADRV_OK != DMADRV_AllocateChannel(&LEUARTDmaTxChannel, NULL)) {
		printf("DMA channel allocation failure for LEUART TX!");
	}*/
	DMADRV_AllocateChannel(&LEUARTDmaRxChannel, NULL);
	DMADRV_AllocateChannel(&LEUARTDmaTxChannel, NULL);
	// DMA RX configuration descriptor
	LEUARTDmaRxCfgDescr.arbRate = dmaArbitrate1;
	LEUARTDmaRxCfgDescr.dstInc = dmaDataInc1;
	LEUARTDmaRxCfgDescr.hprot = 0;
	LEUARTDmaRxCfgDescr.size = dmaDataSize1;
	LEUARTDmaRxCfgDescr.srcInc = dmaDataIncNone;

	// DMA RX channel configuration
	LEUARTDmaRxCfgChannel.cb = NULL;
	LEUARTDmaRxCfgChannel.enableInt = true;
	LEUARTDmaRxCfgChannel.highPri = false;
	LEUARTDmaRxCfgChannel.select = DMAREQ_LEUART0_RXDATAV;

	// Configure RX channel
	DMA_CfgChannel(LEUARTDmaRxChannel, &LEUARTDmaRxCfgChannel);
	// Configure RX channel primary descriptor
	DMA_CfgDescr(LEUARTDmaRxChannel, true, &LEUARTDmaRxCfgDescr);
	// Configure RX channel secondary descriptor
	DMA_CfgDescr(LEUARTDmaRxChannel, false, &LEUARTDmaRxCfgDescr);

	// DMA TX configuration descriptor
	LEUARTDmaTxCfgDescr.arbRate = dmaArbitrate1;
	LEUARTDmaTxCfgDescr.dstInc = dmaDataIncNone;
	LEUARTDmaTxCfgDescr.hprot = 0;
	LEUARTDmaTxCfgDescr.size = dmaDataSize1;
	LEUARTDmaTxCfgDescr.srcInc = dmaDataInc1;

	// DMA TX channel configuration
	LEUARTDmaTxCfgChannel.enableInt = true;
	LEUARTDmaTxCfgChannel.highPri = false;
	LEUARTDmaTxCfgChannel.select = DMAREQ_LEUART0_TXBL;
	LEUARTDmaTxCfgChannel.cb = NULL;
	/*
	LEUARTDmaTxCfgChannel.cb = &cb;
	LEUARTDmaTxCfgChannel.cb->primary = true;
	LEUARTDmaTxCfgChannel.cb->userPtr = NULL;
	LEUARTDmaTxCfgChannel.cb->cbFunc = (void *)DMATxCallback;*/

	// Configure RX channel
	DMA_CfgChannel(LEUARTDmaTxChannel, &LEUARTDmaTxCfgChannel);
	// Configure RX channel primary descriptor
	DMA_CfgDescr(LEUARTDmaTxChannel, true, &LEUARTDmaTxCfgDescr);
}


void startListening() {
    LEUART0->FREEZE = 1;

    // Enable LEUART0 RX
    LEUART0->CMD |= LEUART_CMD_RXEN;
    LEUART0->CTRL |= LEUART_CTRL_RXDMAWU;
    LEUART0->ROUTE |= LEUART_ROUTE_RXPEN;

    LEUART0->FREEZE = 0;

    while (LEUART0->SYNCBUSY) { }
}

void stopListening() {
    LEUART0->FREEZE = 1;

    // Disable LEUART0 RX
    LEUART0->CMD |= LEUART_CMD_RXDIS;
    LEUART0->CTRL &= ~LEUART_CTRL_RXDMAWU;
    LEUART0->ROUTE &= ~LEUART_ROUTE_RXPEN;

    LEUART0->FREEZE = 0;

    while (LEUART0->SYNCBUSY) { }
}

void tx(const uint8_t *bytes, unsigned n) {
    // Check number of bytes to transfer
    if (n == 0) {
        return;
    }

    // Enable TX
    LEUART0->FREEZE = 1;
    LEUART0->ROUTE |= LEUART_ROUTE_TXPEN;
    LEUART0->CMD |= LEUART_CMD_TXEN;
    LEUART0->FREEZE = 0;

    // Wait for sync
    while (LEUART0->SYNCBUSY) { }

    // Start DMA transfer to the LEUART TX
    DMA_ActivateBasic(
		LEUARTDmaTxChannel,
        true,
        false,
        (void*) (&(LEUART0->TXDATA)),
        (void*) bytes,
        n - 1);

    // Enter EM1 sleep while the DMA sends out the data
    // The DMA will wake up the device when it is done
    	EMU_EnterEM1();
        // Just to be sure, wait until the DMA is done
        while (DMA_ChannelEnabled(LEUARTDmaTxChannel)) { }

}

/**************************************************************************//***
 * @brief  IRQ handler for LEUART0
 * Probleem!!!!!
 * Eindbit veranderd afhankelijk van het verstuurde commando. Moet een methode maken en zien dat eindbit ontdekt wordt.
 ******************************************************************************/
void LEUART0_IRQHandler() {
	//dbprintln("Ontvangen:");
	memset(receivedMessage1, 0, sizeof(receivedMessage1));
    unsigned leuartif = LEUART_IntGet(LEUART0);
    // Start frame detection
    if (leuartif & LEUART_IF_STARTF) {
        DMA_ActivateBasic(
            LEUARTDmaRxChannel,
            true,
            false,
            (void*) (rxBuffer),
            (void*) (&(LEUART0->RXDATA)),
            rxBufferSize);


        //// Hooray! You can now do something with the data!!!
		// Doorloop de hele buffer en sla ontvangen antwoorden op om later uit te drukken.
		bool commandoOntvangen = false;
		bool firstcommando = true; // Eerste commando kan zijn dat geen 0x02 meer bij is doordat deze van de vorige keer is afgesneden.
		int startInt = 0;
		int setMarkerstart=0;
		char CS;
		for (int i = 0; i < sizeof(rxBuffer); i++){
			// Doorloop de rxBuffer tot 2 ontvangen, start received.
			if( (rxBuffer[i] == 2) & (!commandoOntvangen) ){
				startInt = 0;
				CS = 0;
				receivedMessage1[startInt] = rxBuffer[i] & 0xFF;
				setMarkerstart = i;
				commandoOntvangen = true;
				firstcommando = false;
				startInt++;
			}

			else if( (rxBuffer[i] != 0 ) & (rxBuffer[i] != 2 ) & (!commandoOntvangen) & (firstcommando) ){ // Stel dat 0x02 ontvangen was op het einde. en het commando ergens random wordt ontvangen.
				startInt = 0;
				CS = 0;
				receivedMessage1[startInt] = 0x02;
				setMarkerstart = i;
				commandoOntvangen = true;
				firstcommando = false;
				startInt++;
				i--;
			}

			else if(commandoOntvangen){
				if(startInt <= 3){ // Commando , Length[0], Length[1] || startInt = 3
					receivedMessage1[startInt] = rxBuffer[i] & 0xFF;
					startInt++;
				}
				else if( ((receivedMessage1[2] + (receivedMessage1[3]<<8)) == 0) ){ // Length[0], Length[1] checken voor hoelang inkomend bericht is.
					// Check Sum berekenen en vgl. (geldig commando ontvangen?
					signed char CS = receivedMessage1[0] ^receivedMessage1[1]^receivedMessage1[2]^receivedMessage1[3];
					if (CS == (rxBuffer[i] & 0xFF) ){
						receivedMessage1[startInt] = CS;
						//dbprint("Commando ontvange; ");
						//dbprintln(receivedMessage1);
						receivedConnection(receivedMessage1);
						// Reset parameters om mogelijk volgende commando te ontvangen
						commandoOntvangen = false;
						memset(rxBuffer + setMarkerstart, 0x00, startInt);
						memset(receivedMessage1, 0, sizeof(receivedMessage1));

					} else{
						startInt++;
						//dbprintln("Slecht ontvangen; delete start 0x02 ");
						//memset(rxBuffer + setMarkerstart, 0x00, 1);
						i = setMarkerstart+1; // Reset en doorloop hele buffer opnieuw.
						commandoOntvangen = false;

					}
				}

				else if( ((receivedMessage1[2] + (receivedMessage1[3]<<8)) > 0 ) ){ // Length[0], Length[1] checken voor hoelang inkomend bericht is.
					// Check Sum berekenen en vgl. (geldig commando ontvangen?
					if (startInt <= (receivedMessage1[2] + (receivedMessage1[3]<<8))+4) {
						if (startInt < (receivedMessage1[2] + (receivedMessage1[3]<<8))+4) {
							receivedMessage1[startInt] = (rxBuffer[i] & 0xFF);
							startInt++;
						}
						else if(startInt == (receivedMessage1[2] + (receivedMessage1[3]<<8))+4){ // CS ontvangen
							CS = 0;
							for (int k =0; k < startInt; k++){ // Calculate CS
								CS = CS ^ (receivedMessage1[k] & 0xFF);
							}
							if (CS == (rxBuffer[i] & 0xFF) ){
								receivedMessage1[startInt] = CS;
								startInt++;
								//dbprint("Commando ontvange; ");
								//dbprintln(receivedMessage1);
								receivedConnection(receivedMessage1);
								// Reset parameters om mogelijk volgende commando te ontvangen
								commandoOntvangen = false;
								memset(rxBuffer + setMarkerstart, 0, startInt);
								memset(receivedMessage1, 0x00, sizeof(receivedMessage1));
							} else{
								//dbprintln("Slecht ontvangen; delete start 0x02 ");
								//memset(rxBuffer + setMarkerstart, 0, 1);
								i = setMarkerstart+1; // Reset en doorloop hele buffer opnieuw.
								commandoOntvangen = false;
							}
						}
					}

				}
			}

		}
    }


    memset(&rxBuffer, 0, sizeof(rxBuffer));
    // Signal frame detection
    if (leuartif & LEUART_IF_SIGF) {
        // Stop DMA transfer
        if (DMA_ChannelEnabled(LEUARTDmaRxChannel)) {
            DMA_ChannelEnable(LEUARTDmaRxChannel, false);
            // Enable RX block - the LEUART will unblock it when the start frame is detected
            LEUART0->CMD |= LEUART_CMD_RXBLOCKEN;
        }
    }
    // Check for connection open command.
    dataChannelOpen = getConnectionEstablished();
    //EMU_EnterEM2(true);
    LEUART_IntClear(LEUART0, 0xffffffff);
}

/**************************************************************************//***
 * @brief  LEUART1 init function
 ******************************************************************************/
void initLEUART(){
	/*-------------------------------------------SET-UP LEUART!!!!---------------------------------------------------------*/
	    /*****Configure GPIO ports as required for LEUART****/
	//C , 14 , 15
		GPIO_PinModeSet(gpioPortD,                /* GPIO port */
				4,                        /* GPIO port number */
				gpioModePushPull,         /* Pin mode is set to push pull */
				1);                       /* High idle state */

		GPIO_PinModeSet(gpioPortD,            /* Port */
				5,                    /* Port number */
				gpioModeInput,    /* Pin mode is set to input only, with pull direction given bellow */
				0);                   /* Pull direction is set to pull-up */

	    /* Reset LEUART */
	    LEUART_Reset(LEUART0);
	    /****Initialise LEUART*********/
	    LEUART_Init(LEUART0, &leuart0Init);
	    LEUART0->FREEZE = 1;

	    // write to the LEUART registers manually.
	    /* Route LEUART0 to LOCATION 0 --> PD4,PD5*/
	    /* Route LEUART0 to LOCATION 5 --> PC14,PC15*/
	    LEUART0->ROUTE = LEUART_ROUTE_LOCATION_LOC0;
	    LEUART0->CTRL = LEUART_CTRL_SFUBRX | LEUART_CTRL_TXDMAWU | LEUART_CTRL_RXDMAWU | LEUART_CTRL_AUTOTRI;
	    LEUART0->CMD |= LEUART_CMD_RXBLOCKEN;
	    LEUART0->SIGFRAME = '\r'; // (uint8_t) IF end of transmission detected = TRUE
	    LEUART0->STARTFRAME = (uint8_t)0x02; // If startframe detected = TRUE

	    LEUART0->FREEZE = 0;
	    //Wait until the SYNCBUSY register is clear. --> if doesnt happen, forgot a clock.
	    while (LEUART0->SYNCBUSY) { }
	    // Enable interrupts!
	    LEUART_IntDisable(LEUART0, 0xffffffff);
	    LEUART_IntEnable(LEUART0, LEUART_IEN_STARTF | LEUART_IEN_SIGF);
	    LEUART_IntClear(LEUART0, 0xffffffff);
	    NVIC_EnableIRQ(LEUART0_IRQn);
	    // The above initialisation leaves the LEUART RX/TX disabled by default. Will need to be put on manually further down in the code.
}

