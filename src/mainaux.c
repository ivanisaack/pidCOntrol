/*
 FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
 All rights reserved

 VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

 This file is part of the FreeRTOS distribution.

 FreeRTOS is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License (version 2) as published by the
 Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

 ***************************************************************************
 >>!   NOTE: The modification to the GPL is included to allow you to     !<<
 >>!   distribute a combined work that includes FreeRTOS without being   !<<
 >>!   obliged to provide the source code for proprietary components     !<<
 >>!   outside of the FreeRTOS kernel.                                   !<<
 ***************************************************************************

 FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  Full license text is available on the following
 link: http://www.freertos.org/a00114.html

 http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
 the FAQ page "My application does not run, what could be wrong?".  Have you
 defined configASSERT()?

 http://www.FreeRTOS.org/support - In return for receiving this top quality
 embedded software for free we request you assist our global community by
 participating in the support forum.

 http://www.FreeRTOS.org/training - Investing in training allows your team to
 be as productive as possible as early as possible.  Now you can receive
 FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
 Ltd, and the world's leading authority on the world's leading RTOS.

 http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
 including FreeRTOS+Trace - an indispensable productivity tool, a DOS
 compatible FAT file system, and our tiny thread aware UDP/IP stack.

 http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
 Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

 http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
 Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
 licenses offer ticketed support, indemnification and commercial middleware.

 http://www.SafeRTOS.com - High Integrity Systems also provide a safety
 engineered and independently SIL3 certified version for use in safety and
 mission critical applications that require provable dependability.

 1 tab == 4 spaces!
 */

/* FreeRTOS.org includes. */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "sapi.h"

#include "filter.h"
#include "signal.h"
#include "utils.h"

#define lpf_order		(1)

float vecB[(lpf_order + 1)];
float vecA[(lpf_order + 1)];
filter_type_t vecX1[(lpf_order + 1)];
filter_type_t vecY1[(lpf_order + 1)];
filter_type_t vecX2[(lpf_order + 1)];
filter_type_t vecY2[(lpf_order + 1)];
filter_type_t vecX3[(lpf_order + 1)];
filter_type_t vecY3[(lpf_order + 1)];
filter_t lpf1;
filter_t lpf2;
filter_t lpf3;
char str[20];

/* Demo includes. */
/*	#include "supporting_functions.h"
 Not available in the freeRTOS version included in the firmware_v2 modules */

/* The DEBUG* functions are sAPI debug print functions.
 Code that uses the DEBUG* functions will have their I/O routed to
 the sAPI DEBUG UART. */
DEBUG_PRINT_ENABLE
;
#define	vPrintString(str) debugPrintString(str)

const char *pcTextForMain = "\r\n PIDControl \r\n";

/* Sets up system hardware */
static void prvSetupHardware(void);

/* The tasks to be created. */
static void vFilterTask(void *pvParameters);
static void vMainProgramTask(void *pvParameters);

/*-----------------------------------------------------------*/

/* Declare a variable of type QueueHandle_t.  This is used to store the queue
 that is accessed by all three tasks. */
QueueHandle_t xFiltredSignalQueue;

/*-----------------------------------------------------------*/

/* Sets up system hardware */
static void prvSetupHardware(void) {
	/* Sets up system hardware */
	boardConfig();
	debugPrintConfigUart(UART_USB, 115200);

	/* Initial LED state is on, keep a live */
	gpioWrite(LED1, ON);
	gpioWrite(LED2, ON);
	gpioWrite(LED3, ON);
}/*-----------------------------------------------------------*/

int main(void) {
	/* Sets up system hardware */
	prvSetupHardware();

	/* Print out the name of this program. */
	vPrintString(pcTextForMain);

	/* The queue is created to hold a maximum of 100 uint16_t values. */
	xFiltredSignalQueue = xQueueCreate(1, sizeof(uint16_t));

	if (xFiltredSignalQueue != NULL) {

		xTaskCreate(vFilterTask, "FilterTask", 1000, NULL, 1, NULL);
		xTaskCreate(vMainProgramTask, "MainProgramTask", 1000, NULL, 1, NULL);

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	} else {
		/* The queue could not be created. */
	}

	/* The following line should never be reached because vTaskStartScheduler()
	 will only return if there was not enough FreeRTOS heap memory available to
	 create the Idle and (if configured) Timer tasks.  Heap management, and
	 techniques for trapping heap exhaustion, are described in the book text. */

	for (;;)
		;
	return 0;
}
/*-----------------------------------------------------------*/

static void vMainProgramTask(void *pvParameters) {

	BaseType_t xStatus;
	uint16_t muestra;

	Signal_Init();

	const TickType_t xTicksToWait = 10UL / portTICK_RATE_MS;

	/* As per most tasks, this task is implemented within an infinite loop. */
	for (;;) {
		/* LED state is toggled, keep a live */
		gpioToggle(LED1);
		xStatus = xQueueReceive(xFiltredSignalQueue, &muestra,0);

		if (xStatus == pdPASS) {
			/* Data was successfully received from the queue, print out the received
			 value. */
			//vPrintStringAndNumber("Received = ", lReceivedValue);

			stdioPrintf(UART_USB, "%d\r\n", muestra);
		}

		/* The first parameter is the queue to which data is being sent.  The
		 queue was created before the scheduler was started, so before this task
		 started to execute.

		 The second parameter is the address of the data to be sent.

		 The third parameter is the Block time – the time the task should be kept
		 in the Blocked state to wait for space to become available on the queue
		 should the queue already be full.  In this case we don’t specify a block
		 time because there should always be space in the queue. */
		//	xStatus = xQueueSendToBack(xFiltredSignalQueue, &lValueToSend, 0);
		//if (xStatus != pdPASS) {
		/* We could not write to the queue because it was full – this must
		 be an error as the queue should never contain more than one item! */
		//vPrintString("Could not send to the queue.\r\n");
		//	}
	}
}
/*-----------------------------------------------------------*/

static void vFilterTask(void *pvParameters) {
	/* Declare the variable that will hold the values received from the queue. */
	BaseType_t xStatus;
	const TickType_t xTicksToWait = 100UL / portTICK_RATE_MS;

	float R = 3200.0;
	float C = 0.000001;
	float T = 0.001;
	vecB[0] = 1;
	vecB[1] = 1;
	vecA[0] = 1 + (R * C * 2) / T;
	vecA[1] = 1 - (R * C * 2) / T;

	Filter_Init(&lpf1, vecB, vecA, vecX1, vecY1, lpf_order);
	Filter_Init(&lpf2, vecB, vecA, vecX2, vecY2, lpf_order);
	Filter_Init(&lpf3, vecB, vecA, vecX3, vecY3, lpf_order);
	Filter_Reset(&lpf1);
	Filter_Reset(&lpf2);
	Filter_Reset(&lpf3);

	filter_type_t x, y;
	uint32_t n, N;
	N = 2 * sig_len;
	float adcAux = 0;
	uint16_t muestra;

	adcConfig(ADC_ENABLE); /* ADC */

	dacConfig(DAC_ENABLE); /* DAC */

	/* This task is also defined within an infinite loop. */
	for (;;) {
		/* LED state is toggled, keep a live */
		gpioToggle(LED2);

		for (n = 0; n < N; n++) {
			x = Signal_Sample(n * 10);
			y = Filter_Process(&lpf1, x);

			dacWrite(DAC, (uint16_t) ((float) ((float) (y + 1)) * 1023) / 2);
			//dacWrite(DAC, 200);
			//adcAux =(float)((float)(adcRead(CH1)/1023)) * 2 - 1;
			//uartWriteString(UART_USB, str);
			muestra = adcRead(CH1);
//			stdioPrintf(UART_USB, "%d\r\n", muestra);
			xStatus = xQueueSendToBack(xFiltredSignalQueue, &muestra, 0);

		/*	if (uxQueueMessagesWaiting(xFiltredSignalQueue) != 0) {
				vPrintString("Queue should have been empty!\r\n");
			}*/
		}
		/* As this task unblocks immediately that data is written to the queue this
		 call should always find the queue empty. */
		/*	if (uxQueueMessagesWaiting(xFiltredSignalQueue) != 0) {
		 vPrintString("Queue should have been empty!\r\n");
		 }*/

		/* The first parameter is the queue from which data is to be received.  The
		 queue is created before the scheduler is started, and therefore before this
		 task runs for the first time.

		 The second parameter is the buffer into which the received data will be
		 placed.  In this case the buffer is simply the address of a variable that
		 has the required size to hold the received data.

		 the last parameter is the block time – the maximum amount of time that the
		 task should remain in the Blocked state to wait for data to be available should
		 the queue already be empty. */
		/*	xStatus = xQueueReceive(xFiltredSignalQueue, &lReceivedValue,
		 xTicksToWait);
		 */
		//	if (xStatus == pdPASS) {
		/* Data was successfully received from the queue, print out the received
		 value. */
		/* vPrintStringAndNumber( "Received = ", lReceivedValue );
		 Not available in the freeRTOS version included in the firmware_v2 modules */
		//		stdioPrintf(UART_USB, "Received = %d\r\n", lReceivedValue);
//		} else {
		/* We did not receive anything from the queue even after waiting for 100ms.
		 This must be an error as the sending tasks are free running and will be
		 continuously writing to the queue. */
		//		vPrintString("Could not receive from the queue.\r\n");
		//	}
	}
}

