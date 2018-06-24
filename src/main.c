#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "sapi.h"

#include "pid.h"
#include "filter.h"
#include "utils.h"
#include "mainProgramUtils.h"
#include "controlUtils.h"
#include "nectar.h"

nectar_target_param_t nectarTarget;
bool_t startProgram = false;
nectar_actual_state_t nectarActualState;
rtc_t rtc;
bool_t rtcVal = false;
bool_t isCamExtrReady = false;
bool_t isCamPresuReady = false;

const char *pcTextForMain = "\r\n PIDControl \r\n";

/* Sets up system hardware */
static void prvSetupHardware(void);

static void vControlTask(void *pvParameters);
static void vAlarmTask(void *pvParameters);
static void vMainProgramTask(void *pvParameters);

static QueueHandle_t xTempPresuQueue;
static SemaphoreHandle_t xUartDatoToPrintSemaphore;

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
	vSemaphoreCreateBinary(xUartDatoToPrintSemaphore);

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

	BaseType_t xStatus, xSemStatus;
	char strDataToUart[20];
	float muestra = 0;
	bool_t processSerialResult;
	/* Inicializar RTC */
	rtcVal = rtcConfig(&rtc);
	vTaskDelay(2 * xDelay1s);

	/* As per most tasks, this task is implemented within an infinite loop. */
	for (;;) {

		processSerialResult = processSerialPort(&nectarTarget, &startProgram);

		vTaskDelay(5 * xDelay1ms);
		xSemStatus = xSemaphoreTake(xUartDatoToPrintSemaphore, 0);

		//xStatus = xQueueReceive(xTempPresuQueue, &muestra, 0);

		if (xSemStatus == 1) {

			nectarActualState.tempExt = scaledTempToRealTemp(
					nectarActualState.tempExt);

			ftoa(nectarActualState.tempExt, strDataToUart, 2);
			vPrintString("tempExtrActualState = ");
			vPrintString(strDataToUart);
			vPrintString("\r\n");

		}
	}
}

static void vControlTask(void *pvParameters) {

	//BaseType_t xStatus;
	TickType_t xLastWakeTime, xLastUartSend;
	size_t cyclesIterator = 0;
	/*--------------INIT CONTROL TEMPERATURA DE EXTRACCION-------------*/
	control_variable_t tempExtrControlVar, tempPresuControlVar;
	control_pid_t pidTempExtr, pidTempPresu;

	max_min_control_t pExtrControlVar, pPresuControlVar;

	float KpTempExtr, KiTempExtr, KdTempExtr, TloopTempExtr;
	float KpTempPresu, KiTempPresu, KdTempPresu, TloopTempPresu;

	float maxPExtr, minPExtr;
	float maxPPresu, minPPresu;

	vTaskDelay(2 * xDelay1s);

	nectarInit(&nectarTarget, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	KpTempExtr = 1.0;
	KiTempExtr = 1000.0;
	KdTempExtr = 0.0;
	TloopTempExtr = 0.001;

	KpTempPresu = 1.0;
	KiTempPresu = 1000.0;
	KdTempPresu = 0.0;
	TloopTempPresu = 0.001;

	PID_Init(&pidTempExtr, KpTempExtr, KiTempExtr, KdTempExtr, TloopTempExtr);
	initControlVariable(&tempExtrControlVar, 0, 0, 0);

	PID_Init(&pidTempPresu, KpTempPresu, KiTempPresu, KdTempPresu,
			TloopTempPresu);
	initControlVariable(&tempPresuControlVar, 0, 0, 0);

	maxPExtr = 2.0;
	minPExtr = 2.0;

	maxPPresu = 1.0;
	minPPresu = 1.0;

	initMaxMinControl(&pExtrControlVar, maxPExtr, minPExtr);
	initMaxMinControl(&pPresuControlVar, maxPPresu, minPPresu);

	/*-----------------------------------------------------------------*/

	xLastWakeTime = xTaskGetTickCount();
	xLastUartSend = xLastWakeTime;

	/* This task is also defined within an infinite loop. */
	for (;;) {

		if (startProgram == true) {
			while (!isCamExtrReady) {

				tempExtrControl(&pidTempExtr, &tempExtrControlVar,
						nectarTarget.tempExt);

				maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);
//TODO: HACER UNA FUNCION PARA QUE ACA TAMBIEEN SE IMPRIMA CADA 500MS.
				if (pExtrControlVar.E < 2.0 && tempExtrControlVar.E < 2.0) {
					isCamExtrReady = true;
				}
				vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

			}
			while (!isCamPresuReady) {

				//TODO: HACER UNA FUNCION PARA QUE ACA TAMBIEEN SE IMPRIMA CADA 500MS.

				tempPresuControl(&pidTempPresu, &tempPresuControlVar,
						nectarTarget.tempExt);

				maxMinPPresuControl(&pPresuControlVar, nectarTarget.pExt);

				tempExtrControl(&pidTempExtr, &tempExtrControlVar,
						nectarTarget.tempExt);

				maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

				if (pPresuControlVar.E < 2.0 && tempPresuControlVar.E < 2.0) {
					isCamPresuReady = true;
				}

				vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

			}
			//TODO:HACER CONTROL DE LA CAMPARA DE PRESURIZACION SIMILAR A LA EXTREACCION PERO DISTINTO.
			if (isCamExtrReady && isCamPresuReady) {
				for (cyclesIterator = 0; cyclesIterator < nectarTarget.nCiclos;
						cyclesIterator++) {
					rtc.min = 0;
					rtcVal = rtcWrite(&rtc);

					while (rtc.min < nectarTarget.tPasoEstatico) {

						tempExtrControl(&pidTempExtr, &tempExtrControlVar,
								nectarTarget.tempExt);

						maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

						if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

							//setNectarActualState(&nectarActualState, nectarTarget);
							nectarActualState.tempExt = tempExtrControlVar.B;
							xSemaphoreGive(xUartDatoToPrintSemaphore);
							//	xStatus = xQueueSendToBack(xTempPresuQueue,
							//		&(tempExtrControlVar.B), 0);
							xLastUartSend = xTaskGetTickCount();
							rtcVal = rtcRead(&rtc);
							vPrintNumber(rtc.min);
							vPrintString("\r\n");

						}

						vTaskDelayUntil(&xLastWakeTime, xDelay1ms);
					}
					rtc.min = 0;
					rtcVal = rtcWrite(&rtc);
					while (rtc.min < nectarTarget.tPasoDinamico) {
						rtcVal = rtcRead(&rtc);
						//TODO:ESTO FALTA PENSARLO TODAVIA.
						vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

					}
				}
			}
		}
		vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

	}
}

static void vAlarmTask(void *pvParameters) {

	TickType_t xLastWakeTime;

	float tempPresuControl = 0.0;

	vTaskDelay(2 * xDelay1s);

	xLastWakeTime = xTaskGetTickCount();

	for (;;) {

		tempPresuControl = ((float) (adcRead(CH1)) * 3.3) / 1023;

		if (tempPresuControl > MAX_TEMP) {
			dacWrite(DAC, 0);
			nectarTarget.tempExt = 0.0;
			vPrintString(" Maxima temperatura");
			vPrintString("\r\n");
			if (tempPresuControl > MAX_CRITICAL_TEMP) {
				while (1) {
					nectarTarget.tempExt = 0.0;
					vPrintString(" temperatura critica, reiniciar ");
					vPrintString("\r\n");

				}
			}
		}
		vTaskDelayUntil(&xLastWakeTime, xDelay1s);

	}

}

/*-----------------------------------------------------------*/

