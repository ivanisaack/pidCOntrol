#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "sapi.h"

#include "pid.h"
#include "filter.h"
#include "utils.h"

DEBUG_PRINT_ENABLE
;
#define	vPrintString(str) debugPrintString(str)

static float tempTarget = 0.0;
control_pid_t pid;

const char *pcTextForMain = "\r\n PIDControl \r\n";

/* Sets up system hardware */
static void prvSetupHardware(void);

/* The tasks to be created. */
static void vAlarmTask(void *pvParameters);
static void vControlTask(void *pvParameters);
static void vMainProgramTask(void *pvParameters);

/*-----------------------------------------------------------*/

/* Declare a variable of type QueueHandle_t.  This is used to store the queue
 that is accessed by all three tasks. */
QueueHandle_t xTempPresuQueue;

/* Sets up system hardware */
static void prvSetupHardware(void) {
	/* Sets up system hardware */
	boardConfig();
	debugPrintConfigUart(UART_USB, 115200);

	adcConfig(ADC_ENABLE); /* ADC */

	dacConfig(DAC_ENABLE); /* DAC */
	/* Initial LED state is on, keep a live */
	gpioWrite(LED1, ON);
	gpioWrite(LED2, ON);
	gpioWrite(LED3, ON);
}/*-----------------------------------------------------------*/

int main(void) {

	prvSetupHardware();

	/* Print out the name of this program. */
	vPrintString(pcTextForMain);

	/* The queue is created to hold a maximum of 100 uint16_t values. */
	xTempPresuQueue = xQueueCreate(1, sizeof(float));

	xTaskCreate(vControlTask, "ControlTask", 1000, NULL, 2, NULL);
	xTaskCreate(vMainProgramTask, "MainProgramTask", 1000, NULL, 1, NULL);
	xTaskCreate(vAlarmTask, "AlarmTask", 1000, NULL, 3, NULL);
	/* Start the scheduler so the created tasks start executing. */
	vTaskStartScheduler();

	/* The following line should never be reached because vTaskStartScheduler()
	 will only return if there was not enough FreeRTOS heap memory available to
	 create the Idle and (if configured) Timer tasks.  Heap management, and
	 techniques for trapping heap exhaustion, are described in the book text. */

	for (;;)
		;
	return 0;

}

static void vMainProgramTask(void *pvParameters) {

	const TickType_t xDelay1ms = 1UL / portTICK_RATE_MS;

	BaseType_t xStatus;
	char str[20];
	char auxFloat[4];
	float tempTarg = 0.0;
	uint8_t dataUart;
	float muestra = 0;
	size_t i = 0;
	/* As per most tasks, this task is implemented within an infinite loop. */
	for (;;) {
		/* LED state is toggled, keep a live */
		gpioToggle(LED1);

		i = 0;
		while (uartReadByte(UART_USB, &dataUart) && i < 5) {
			auxFloat[i] = dataUart;
			i++;
			//vPrintString(itoa(i,str,10));
			//vPrintString(" es i \r\n");
			vTaskDelay(xDelay1ms);

		}
		if (i == 5) {
			tempTarg = atof(auxFloat);
			i = 0;
			ftoa(tempTarg, auxFloat, 2);
			//vPrintString(auxFloat);
			//vPrintString("\r\n");
			tempTarget = tempTarg;
		}
		//stdioPrintf(UART_USB, "data: %c \r\n", dataUart);

		xStatus = xQueueReceive(xTempPresuQueue, &muestra, 0);

		if (xStatus == pdPASS) {

			ftoa(muestra, str, 4);
			vPrintString(str);
			vPrintString("\r\n");
			/* Data was successfully received from the queue, print out the received
			 value. */
			//vPrintStringAndNumber("Received = ", lReceivedValue);
			//stdioPrintf(UART_USB, "%f\r\n", 1.0);
		}
	}
}
/*-----------------------------------------------------------*/

static void vControlTask(void *pvParameters) {

	/* Declare the variable that will hold the values received from the queue. */

	BaseType_t xStatus;

	TickType_t xLastWakeTime, xLastUartSend;
	/*	const TickType_t xDelay5ms = pdMS_TO_TICKS( 3UL );
	 Not available in the freeRTOS version included in the firmware_v2 modules */
	const TickType_t xDelay1ms = 1UL / portTICK_RATE_MS;
	const TickType_t xDelay500ms = 500UL / portTICK_RATE_MS;
//delayTick_t delayUart;

	float B, E, X;

	float Kp, Ki, Kd, Tloop;

	Kp = 1.0;
	Ki = 1000.0;
	Kd = 0.0;
	Tloop = 0.001;

	PID_Init(&pid, Kp, Ki, Kd, Tloop);

	B = 0.0;

	xLastWakeTime = xTaskGetTickCount();
	xLastUartSend = xLastWakeTime;
//delayConfigII(&delayUart, 1000);

	/* This task is also defined within an infinite loop. */
	for (;;) {
		/* LED state is toggled, keep a live */
		gpioToggle(LED2);

		E = tempTarget - B;
//		stdioPrintf(UART_USB, "error: %f\r\n",(uint16_t)E );

		X = PID_Process(&pid, E);
		X = (3.3 < X) ? 3.3 : X;
		//	stdioPrintf(UART_USB, "X: %d\r\n",(uint16_t) X );

		dacWrite(DAC, (uint16_t) (X * (1023 / 3.3)));

		vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

		B = ((float) (adcRead(CH1)) * 3.3) / 1023;
		//stdioPrintf(UART_USB, "B: %f\r\n",(uint16_t)B );

		//if (delayReadII(&delayUart)) {
		if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {
			xStatus = xQueueSendToBack(xTempPresuQueue, &B, 0);
			xLastUartSend = xTaskGetTickCount();
		}
	}
}

static void vAlarmTask(void *pvParameters) {


	TickType_t xLastWakeTime;
	const TickType_t xDelay1s = 1000UL / portTICK_RATE_MS;

	float tempPresuControl = 0.0;

	xLastWakeTime = xTaskGetTickCount();


	for (;;) {

		tempPresuControl = ((float) (adcRead(CH1)) * 3.3) / 1023;

		if (tempPresuControl > MAX_TEMP) {
			dacWrite(DAC, 0);
			tempTarget = 0.0;
			vPrintString(" Maxima temperatura");
			vPrintString("\r\n");
			if (tempPresuControl > MAX_CRITICAL_TEMP) {
				while (1) {
					tempTarget = 0.0;
					vPrintString(" temperatura critica, reiniciar ");
					vPrintString("\r\n");

				}
			}
		}
		vTaskDelayUntil(&xLastWakeTime, xDelay1s);

	}

}

