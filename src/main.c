#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "sapi.h"

#include "pid.h"
#include "filter.h"
#include "utils.h"
#include "mainProgramUtils.h"
#include "nectar.h"

//DEBUG_PRINT_ENABLE
//;

//#define	vPrintString(str) debugPrintString(str)

float tempTarget = 0.0;
control_pid_t pid;
static nectar_target_param_t nectarTarget;

const char *pcTextForMain = "\r\n PIDControl \r\n";

/* Sets up system hardware */
static void prvSetupHardware(void);

static void vControlTask(void *pvParameters);
static void vAlarmTask(void *pvParameters);
static void vMainProgramTask(void *pvParameters);

QueueHandle_t xTempPresuQueue;

/* Sets up system hardware */
static void prvSetupHardware(void) {
	/* Sets up system hardware */
	boardConfig();
	uartConfig(UART_USB, 115200);

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

	xTaskCreate(vMainProgramTask, "MainProgramTask", 1000, NULL, 1, NULL);
	xTaskCreate(vControlTask, "ControlTask", 1000, NULL, 2, NULL);
	xTaskCreate(vAlarmTask, "AlarmTask", 1000, NULL, 3, NULL);

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

	BaseType_t xStatus;

	char auxFloat[4];
	float tempTarg = 0.0;
	uint8_t dataUart;
	char strQueque[20];
	float muestra = 0;
	static size_t i = 0;

	/* As per most tasks, this task is implemented within an infinite loop. */
	for (;;) {
		gpioToggle(LED1);

		i = 0;
		if (processSerialPort(&nectarTarget)) {

			/*vPrintNumber(nectarTarget.tempExt);
			vPrintString("\r\n");
			vPrintString("preison");
			vPrintNumber(nectarTarget.pExt);
			vPrintString("\r\n");*/

		}

		/*	while (uartReadByte(UART_USB, &dataUart) && i < 5) {
		 auxFloat[i] = dataUart;
		 i++;
		 //vPrintString(itoa(i,str,10));
		 //vPrintString(" es i \r\n");
		 vTaskDelay(xDelayUart1ms);

		 }*/
		if (i == 5) {
			tempTarg = atof(auxFloat);
			i = 0;
			ftoa(tempTarg, auxFloat, 2);
			vPrintString(auxFloat);
			vPrintString("\r\n");
			tempTarget = tempTarg;
		}
		//stdioPrintf(UART_USB, "data: %c \r\n", dataUart);

		xStatus = xQueueReceive(xTempPresuQueue, &muestra, 0);

		if (xStatus == pdPASS) {

			ftoa(muestra, strQueque, 4);
			vPrintString(strQueque);
			vPrintString("\r\n");

		}
	}
}

static void vControlTask(void *pvParameters) {

	BaseType_t xStatus;

	TickType_t xLastWakeTime, xLastUartSend;

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

	/* This task is also defined within an infinite loop. */
	for (;;) {

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

/*-----------------------------------------------------------*/

