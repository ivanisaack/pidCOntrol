#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "sapi.h"

#include "pid.h"
#include "filter.h"
#include "utils.h"
#include "mainProgramUtils.h"
#include "controlUtils.h"
#include "nectar.h"

//DEBUG_PRINT_ENABLE
//;

//#define	vPrintString(str) debugPrintString(str)

float tempTarget = 0.0;
static nectar_target_param_t nectarTarget;
static bool_t startProgram = false;
static nectar_actual_state_t nectarActualState;

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
	//xNectarActualStateQueque = xQueueCreate(1, sizeof(nectarActualState));

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
	char strQueque[20];
	float muestra = 0;
	bool_t processSerialResult;

	/* As per most tasks, this task is implemented within an infinite loop. */
	for (;;) {

		processSerialResult = processSerialPort(&nectarTarget, &startProgram);

		xStatus = xQueueReceive(xTempPresuQueue, &muestra, 0);

		if (xStatus == pdPASS) {


			nectarActualState.tempExt = scaledTempToRealTemp(
					nectarActualState.tempExt);

			ftoa(nectarActualState.tempExt, strQueque, 2);
			vPrintString("tempExtrActualState = ");
			vPrintString(strQueque);
			vPrintString("\r\n");

		}
	}
}

static void vControlTask(void *pvParameters) {

	BaseType_t xStatus;
	TickType_t xLastWakeTime, xLastUartSend;

	/*--------------INIT CONTROL TEMPERATURA DE EXTRACCION-------------*/
	control_variable_t tempExtrControlVar;
	control_pid_t pidTempExtr;
	float KpTempExtr, KiTempExtr, KdTempExtr, TloopTempExtr;
	nectarInit(&nectarTarget, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	KpTempExtr = 1.0;
	KiTempExtr = 1000.0;
	KdTempExtr = 0.0;
	TloopTempExtr = 0.001;

	PID_Init(&pidTempExtr, KpTempExtr, KiTempExtr, KdTempExtr, TloopTempExtr);
	initControlVariable(&tempExtrControlVar, 0, 0, 0);
	/*-----------------------------------------------------------------*/

	xLastWakeTime = xTaskGetTickCount();
	xLastUartSend = xLastWakeTime;

	/* This task is also defined within an infinite loop. */
	for (;;) {

		if (startProgram == true) {
			tempExtrControl(&pidTempExtr, &tempExtrControlVar,
					nectarTarget.tempExt);

			if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

				//setNectarActualState(&nectarActualState, nectarTarget);
				nectarActualState.tempExt = tempExtrControlVar.B;

				xStatus = xQueueSendToBack(xTempPresuQueue,
						&(tempExtrControlVar.B), 0);
				xLastUartSend = xTaskGetTickCount();
			}
		}

		vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

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

