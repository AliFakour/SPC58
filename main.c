/****************************************************************************
*
* Copyright © 2015-2021 STMicroelectronics - All Rights Reserved
*
* This software is licensed under SLA0098 terms that can be found in the
* DM00779817_1_0.pdf file in the licenses directory of this software product.
* 
* THIS SOFTWARE IS DISTRIBUTED "AS IS," AND ALL WARRANTIES ARE DISCLAIMED, 
* INCLUDING MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
*****************************************************************************/

/* Inclusion of the main header files of all the imported components in the
   order specified in the application wizard. The file is generated
   automatically.*/

#include "components.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "saradc_lld_cfg.h"
#include "serial_lld_cfg.h"
#include "spi_lld_cfg.h"
#include "can_lld_cfg.h"
#include <inttypes.h>
#include "FreeRTOS.h"
#include "task.h"



//#define PLOT_ACCELERATOR_DATA
//#define CAN_DEBUG
#define SMOOTH_ADC
#define SEND_DATA
#define UPDATE_DAC
#define READ_UART
//#define DIRECT_DAC
#define PRINT_LOG


/* Definition of MCP4922 bits*/
#define  AB				15
#define BUF				14
#define GAIN			13
#define SHDN			12
#define DAC_B			(1 << AB)
#define DAC_A			(0 << AB)
#define BUFFERED		(1 << BUF)
#define UNBUFFERED		(0 << BUF)
#define GAIN1			(1 << GAIN)
#define GAIN2			(0 << GAIN)
#define DAC_ACTIVE		(1 << SHDN)
#define DAC_SHDN		(0 << SHDN)
#define DAC_DEFAULT_VAL 300



#define MSG_PREFIX1		"Mcfly = "
#define MSG_PREFIX2		" Map = "
#define MSG_PREFIX3		"D1"
#define MSG_PREFIX4		"D2"
#define SEPARATOR		","
#define NEW_LINE		"\n\r"


uint16_t dacA , dacB ;
uint8_t prnd = 0;
uint8_t rpm= 0;
uint8_t msgBuf[100];
uint8_t msgUpdated=0;
uint8_t dacUpdated=0;
uint16_t mapped_vout=0;
const char delimiter[] = ",";
uint8_t vout = 0;
char myString[50];
uint8_t message[50];


/* define a vector containing the channels to be converted */
#define NUMOFCHANNELS 2U
extern uint8_t anp[NUMOFCHANNELS];
extern uint16_t value[NUMOFCHANNELS];

/* set channels to convert */
uint8_t anp[NUMOFCHANNELS] = {27U, 42U};
uint16_t value[NUMOFCHANNELS] = {0U ,0U};
#ifdef SMOOTH_ADC
	#define SMOOTH_WINDOW_SIZE			10
	uint16_t value0[SMOOTH_WINDOW_SIZE];
	uint16_t value1[SMOOTH_WINDOW_SIZE];
#endif

uint8_t strValue0[10];
uint8_t strValue1[10];
uint8_t strDac[15];

uint8_t message_error_rtos[15]= "Out of RTOS\r\n";
uint8_t message_log_port[15]= "LOG PORT OPEN\r\n";

void updateDAC(uint16_t DACvalue1, uint16_t DACvalue2){
	if(dacUpdated > 2){
		DACvalue1 = DAC_DEFAULT_VAL;
		DACvalue2 = DAC_DEFAULT_VAL;
		dacUpdated = 100;
	}
	pal_lld_togglepad(PORT_F,LED3);
	uint16_t dac1,dac2;
	dac1 = (DACvalue1) & 0x0FFF;
	dac2 = (DACvalue2) & 0x0FFF;

	dacA = (DAC_A | BUFFERED | GAIN2 | DAC_ACTIVE | dac1);
	spi_lld_send(&SPID1, 1, &dacA);

	dacB = (DAC_B | BUFFERED | GAIN1 | DAC_ACTIVE | dac2);
	spi_lld_send(&SPID1, 1, &dacB);
	dacUpdated++;
}

uint16_t map(uint8_t In_L, uint8_t In_H,uint16_t Out_L, uint16_t Out_H,uint8_t input){
	uint16_t mappedValue;
	uint8_t InRange = In_H - In_L;
	uint16_t OutRange = Out_H - Out_L;
	float Coefficiente = OutRange / InRange;
	mappedValue = (int)(Coefficiente *input);
 	return mappedValue;
}

void canReadBuf(CANRxFrame crfp) {
	if( can_lld_receive(&CAND2, CAN_ANY_RXBUFFER, &crfp) == 0){

		#ifdef CAN_DEBUG
			uint16_t CANid = crfp.ID;
			printf("CAN,%d",CANid);
			for(int i=0; i<crfp.DLC ; i++){
				printf(",%d",crfp.data8[i]);
			}
			printf("\r\n");
		#endif

		if(crfp.ID == 0x118){
			prnd= crfp.data8[7];
		}else if(crfp.ID == 0x108){
			rpm= crfp.data8[7];
		}
	}
}


void mcanconf_can1sub0rxcb(uint32_t msgbuf, CANRxFrame crfp) {

		if(crfp.ID == 0x108){
			//rpm= crfp.data8[7];
		}else if(crfp.ID == 0x118){
			prnd= crfp.data8[7];
		}
		(void)msgbuf;
}


void mcanconf_errorcb(CANDriver *canp, uint32_t psr){
  /* write error management code here */
  (void)canp;
  (void)psr;
}



/* conversion callback */
void cfg_saradc1_adc_callback27(SARADCDriver *saradcp) {

	/* Read converted channels */
	#ifndef SMOOTH_ADC
		value[0] = saradc_lld_readchannel(saradcp, anp[0]);
	#endif
	#ifdef SMOOTH_ADC
		static int n=0;
		value0[n] = saradc_lld_readchannel(saradcp, anp[0]);
		if(n < SMOOTH_WINDOW_SIZE-1 ){
			n++;
		}else{
			n=0;
		}
		value[0] = 0;
		for(int j=0; j<SMOOTH_WINDOW_SIZE ; j++){
			value[0] += value0[j];
		}
		value[0] =value[0] /SMOOTH_WINDOW_SIZE;
	#endif
}

void cfg_saradc0_adc_callback42(SARADCDriver *saradcp) {

	/* Read converted channels */
	#ifndef SMOOTH_ADC
		value[1] = saradc_lld_readchannel(saradcp, anp[1]);
	#endif
	#ifdef SMOOTH_ADC
		static int i=0;
		value1[i] = saradc_lld_readchannel(saradcp, anp[1]);
		if(i < SMOOTH_WINDOW_SIZE-1 ){
			i++;
		}else{
			i=0;
		}
		value[1] = 0;
		for(int m=0; m<SMOOTH_WINDOW_SIZE ; m++){
			value[1] += value1[m];
		}
		value[1] =value[1] /SMOOTH_WINDOW_SIZE;
	#endif
}


void rx0cb(SerialDriver *sdp){
	pal_lld_togglepad(PORT_F,LED1);
	(void) sdp;
	msgUpdated = 1;
	dacUpdated = 0;
}








/* Tasks */

#ifdef UPDATE_DAC
	portTASK_FUNCTION( vTaskUpdateDac, pvParameters )
	{
	  ( void ) pvParameters;
	  TickType_t xLastWakeTime = xTaskGetTickCount();
	  for ( ;; ) {
		vTaskSuspendAll();

#ifndef DIRECT_DAC
		dacA = vout*10;
		dacB = vout*10;
#endif
#ifdef DIRECT_DAC
		dacA = value[0]*5;
		dacB = value[0]*5;
#endif

		updateDAC(dacA,dacB);

		xTaskResumeAll();
		vTaskDelayUntil( &xLastWakeTime, 40 );
	  }
	}
#endif

#ifdef READ_UART
	portTASK_FUNCTION( vTaskReadUart, pvParameters )
	{
	  ( void ) pvParameters;
	  TickType_t xLastWakeTime = xTaskGetTickCount();
	  for ( ;; ) {
		vTaskSuspendAll();
		sd_lld_read(&SD1, &msgBuf, 5);
		if(msgUpdated !=0 ){
			char *token = strtok(msgBuf, delimiter);
			vout = atoi(token);
			if(vout > 160 ) vout = 160;
			if(vout < 30 ) vout = 30;
			msgUpdated = 0;
			pal_lld_togglepad(PORT_F,LED2);
		}
		xTaskResumeAll();
		vTaskDelayUntil( &xLastWakeTime, 40 );
	  }
	}
#endif

#ifdef SEND_DATA
	portTASK_FUNCTION( vTaskSendData, pvParameters )
	{
	  ( void ) pvParameters;
	  TickType_t xLastWakeTime = xTaskGetTickCount();
	  for ( ;; ) {
		vTaskSuspendAll();
		//prnd = 2;

		if(value[0]> 200){
			if(rpm<200){
				rpm++;
			}
		}else{
			if(rpm>0){
				rpm--;
			}

		}

		printf("A,%d,G,0,C,0,x0108,%d,0,x0118,%d,0,\r\n",value[0],rpm * 100,prnd);
		xTaskResumeAll();
		vTaskDelayUntil( &xLastWakeTime, 40 );
	  }
	}
#endif


#ifdef PRINT_LOG
	portTASK_FUNCTION( vTaskPrintLog, pvParameters )
	{
	  ( void ) pvParameters;
	  TickType_t xLastWakeTime = xTaskGetTickCount();
	  for ( ;; ) {
		vTaskSuspendAll();

		mapped_vout = map(0,200,0,1023,vout);
		snprintf(myString,sizeof(myString), "McFly = %d, Map = %d, PNDR = %d\r\n",vout,mapped_vout,prnd);
		sd_lld_write(&SD7,myString,(uint16_t)(sizeof(myString)/sizeof(myString[0])));
		myString[0] = NULL;

		xTaskResumeAll();
		vTaskDelayUntil( &xLastWakeTime, 500 );
	  }
	}
#endif


/* Application entry point. */
int main(void) {



	/* Initialization of all the imported components in the order specified in
	 the application wizard. The function is generated automatically.*/
	componentsInit();

	/* Enable Interrupts */
	irqIsrEnable();

	/* Activates the serial driver 1 using the driver default configuration. */
	msgUpdated = 0;
	sd_lld_start(&SD1, &serial_config_uart_cfg);
	sd_lld_start(&SD7, &serial_config_uart_cfg_logport);

	/* Configure CAN */
	can_lld_init();
	can_lld_start(&CAND2, &can_config_mcanconf);




	/* Create instances of SARADC drivers. */


	SARADCDriver* sarADCdriver1;
	SARADCDriver* sarADCdriver2;
	sarADCdriver1 = &SARADC12D1;
	sarADCdriver2 = &SARADC12D2;

	saradc_lld_start(sarADCdriver1, &saradc_config_cfg_saradc0);
	saradc_lld_start(sarADCdriver2, &saradc_config_cfg_saradc1);



	saradc_lld_start_conversion(sarADCdriver1);
	saradc_lld_start_conversion(sarADCdriver2);



	/* Application main loop, two SPI configuration are used alternating them.*/
	spi_lld_start(&SPID1, &spi_config_low_speed_16);  /* Setup parameters.        */




#ifdef UPDATE_DAC
	/* Creating task to update DAC outputs */
	xTaskCreate( vTaskUpdateDac,
				 (const char * const)"task Update DAC",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 NULL );
#endif

#ifdef READ_UART
	/* Creating task to read uart */
	xTaskCreate( vTaskReadUart,
				 (const char * const)"task read uart",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 NULL );
#endif

#ifdef SEND_DATA
	/* Creating task to read uart */
	xTaskCreate( vTaskSendData,
				 (const char * const)"task send data",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 NULL );
#endif

#ifdef PRINT_LOG
	/* Creating task to print log */
	xTaskCreate( vTaskPrintLog,
				 (const char * const)"task print log",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 NULL );
#endif


	/* Start the FreeRTOS scheduler */

	//printf("Start RTOS tasks\r\n");
	//sd_lld_write(&SD7,message_log_port,(uint16_t)(sizeof(message_log_port)/sizeof(message_log_port[0])));
	//osalThreadDelayMilliseconds(50);


	vTaskStartScheduler();

	for(;;){
		printf("Out of RTOS\r\n");
		osalThreadDelayMilliseconds(500);
	}

	return 0;

}









