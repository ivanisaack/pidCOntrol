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
/*
 startProgram = false;
 rtcVal = false;
 isCamExtrReady = false;
 isCamPresuReady = false;
 */

static bool_t startProgram = false;

static nectar_target_param_t nectarTarget;
static nectar_actual_param_t nectarActualState;

static bool_t isCamExtrReady = false;
static bool_t isCamPresuReady = false;

const char *pcTextForMain = "\r\n PIDControl \r\n";

/* Sets up system hardware */
static void prvSetupHardware(void);

static void vControlTask(void *pvParameters);
static void vAlarmTask(void *pvParameters);
static void vMainProgramTask(void *pvParameters);

static QueueHandle_t xTempPresuQueue;
SemaphoreHandle_t xUartDatoToPrintSemaphore;

/* Sets up system hardware */
static void prvSetupHardware(void) {
	/* Sets up system hardware */

	boardConfig();
	uartConfig(UART_USB, 115200);

	adcConfig(ADC_ENABLE); /* ADC */
	dacConfig(DAC_ENABLE); /* DAC */

	/*......Inicialización calentadores..........*/
	uint8_t pwmVal = 0; /* 0 a 255 */

	/* Configurar PWM */
	//pwmConfig(0, PWM_ENABLE);
	//pwmConfig(PIN_CALENTADOR_PRES, PWM_ENABLE_OUTPUT);
	//pwmConfig(PIN_CALENTADOR_EXTR, PWM_ENABLE_OUTPUT);
	/*......Inicialización valvulas..........*/

	gpioWrite(PIN_VALVULA_SALIDA, OFF);
	gpioWrite(PIN_VALVULA_PRESU, OFF);

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

	BaseType_t xSemStatus;
	char strDataToUart[20];
	bool_t processSerialResult;

	/* Inicializar RTC */
	vTaskDelay(2 * xDelay1s);

	/* As per most tasks, this task is implemented within an infinite loop. */
	for (;;) {

		processSerialResult = processSerialPort(&nectarTarget, &startProgram);

		vTaskDelay(5 * xDelay1ms);
		xSemStatus = xSemaphoreTake(xUartDatoToPrintSemaphore, 0);

		if (xSemStatus == 1) {

			vPrintString("Actual state: ");
			vPrintNumber(nectarActualState.actualState);
			vPrintString("\r\n");

			nectarActualState.tempExt = scaledTempToRealTemp(
					nectarActualState.tempExt);
			ftoa(nectarActualState.tempExt, strDataToUart, 2);
			vPrintString("tempExtrActualState = ");
			vPrintString(strDataToUart);
			vPrintString("\r\n");

			nectarActualState.pExt = scaledPresToRealPres(
					nectarActualState.pExt);
			ftoa(nectarActualState.pExt, strDataToUart, 2);
			vPrintString("pExtrActualState = ");
			vPrintString(strDataToUart);
			vPrintString("\r\n");

		}
	}
}

static void vControlTask(void *pvParameters) {

	/*--------------Inicializacion-------------*/
	static size_t cyclesIterator = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	/*TickType_t xLastWakeTime, xLastUartSend, xLastOut;
	 size_t cyclesIterator = 0;
	 control_variable_t tempExtrControlVar, tempPresuControlVar,
	 tempOutControlVar;
	 control_pid_t pidTempExtr, pidTempPresu, pidTempOut;

	 max_min_control_t pExtrControlVar, pPresuControlVar;

	 float KpTempExtr, KiTempExtr, KdTempExtr, TloopTempExtr;
	 float KpTempPresu, KiTempPresu, KdTempPresu, TloopTempPresu;
	 float KpTempOut, KiTempOut, KdTempOut, TloopTempOut;

	 float maxPExtr, minPExtr;
	 float maxPPresu, minPPresu;

	 bool_t isOutValveOpen = false;*/

	/*vTaskDelay(2 * xDelay1s);


	 nectarInit(&nectarTarget, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	 KpTempExtr = 1.0;
	 KiTempExtr = 1000.0;
	 KdTempExtr = 0.0;
	 TloopTempExtr = 0.001;

	 KpTempPresu = 1.0;
	 KiTempPresu = 1000.0;
	 KdTempPresu = 0.0;
	 TloopTempPresu = 0.001;

	 KpTempOut = 1.0;
	 KiTempOut = 1000.0;
	 KdTempOut = 0.0;
	 TloopTempOut = 0.001;

	 PID_Init(&pidTempExtr, KpTempExtr, KiTempExtr, KdTempExtr, TloopTempExtr);
	 initControlVariable(&tempExtrControlVar, 0, 0, 0);

	 PID_Init(&pidTempPresu, KpTempPresu, KiTempPresu, KdTempPresu,
	 TloopTempPresu);
	 initControlVariable(&tempPresuControlVar, 0, 0, 0);

	 PID_Init(&pidTempOut, KpTempOut, KiTempOut, KdTempOut, TloopTempOut);
	 initControlVariable(&tempOutControlVar, 0, 0, 0);

	 maxPExtr = 2.0;
	 minPExtr = 2.0;

	 maxPPresu = 1.0;
	 minPPresu = 1.0;

	 initMaxMinControl(&pExtrControlVar, maxPExtr, minPExtr);
	 initMaxMinControl(&pPresuControlVar, maxPPresu, minPPresu);

	 xLastWakeTime = xTaskGetTickCount();
	 xLastUartSend = xLastWakeTime;
	 xLastOut = xLastWakeTime;*/
	initControlTask(&nectarTarget);
	/*----------------------Iteracion-----------------------------------------------*/

	/* This task is also defined within an infinite loop. */
	for (;;) {

		if (startProgram == true) {

			isCamExtrReady = prepareCamExtr(nectarTarget, &nectarActualState);
			isCamPresuReady = prepareCamPres(nectarTarget, &nectarActualState);

			/*			nectarActualState.actualState = preparandoCamaraExtr;

			 while (!isCamExtrReady) {

			 tempExtrControl(&pidTempExtr, &tempExtrControlVar,
			 nectarTarget.tempExt);

			 maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

			 if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

			 nectarActualState.tempExt = tempExtrControlVar.B;
			 nectarActualState.pExt = pExtrControlVar.acutalP;

			 xSemaphoreGive(xUartDatoToPrintSemaphore);
			 xLastUartSend = xTaskGetTickCount();
			 }
			 }
			 vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

			 }*/

			/*			nectarActualState.actualState = preparandoCamaraPres;

			 while (!isCamPresuReady) {

			 //TODO: Podria hacer un vector CON VARIABLES DE CONTROL DE T y OTRO DE P

			 tempPresuControl(&pidTempPresu, &tempPresuControlVar,
			 nectarTarget.tempPresu);

			 maxMinPPresuControl(&pPresuControlVar, nectarTarget.pPresu);

			 tempExtrControl(&pidTempExtr, &tempExtrControlVar,
			 nectarTarget.tempExt);

			 maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

			 if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

			 nectarActualState.tempPresu = tempPresuControlVar.B;
			 nectarActualState.pPresu = pPresuControlVar.acutalP;
			 nectarActualState.tempExt = tempExtrControlVar.B;
			 nectarActualState.pExt = pExtrControlVar.acutalP;

			 xSemaphoreGive(xUartDatoToPrintSemaphore);
			 xLastUartSend = xTaskGetTickCount();
			 }

			 if (pPresuControlVar.E < 2.0 && tempPresuControlVar.E < 2.0) {
			 isCamPresuReady = true;
			 }

			 vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

			 }
			 */

			if (isCamExtrReady && isCamPresuReady) {

				for (cyclesIterator = 0; cyclesIterator < nectarTarget.nCiclos;
						cyclesIterator++) {

					nectarActualState.Ciclo = cyclesIterator + 1;

					maserar(nectarTarget, &nectarActualState);
				/*	nectarActualState.actualState = maserando;

					rtc.min = 0;
					rtcVal = rtcWrite(&rtc);

					while (rtc.min < nectarTarget.tPasoEstatico) {

						tempPresuControl(&pidTempPresu, &tempPresuControlVar,
								nectarTarget.tempPresu);

						maxMinPPresuControl(&pPresuControlVar,
								nectarTarget.pPresu);

						tempExtrControl(&pidTempExtr, &tempExtrControlVar,
								nectarTarget.tempExt);

						maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

						if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

							nectarActualState.tempPresu = tempPresuControlVar.B;
							nectarActualState.pPresu = pPresuControlVar.acutalP;
							nectarActualState.tempExt = tempExtrControlVar.B;
							nectarActualState.pExt = pExtrControlVar.acutalP;

							xSemaphoreGive(xUartDatoToPrintSemaphore);
							xLastUartSend = xTaskGetTickCount();
						}

						vTaskDelayUntil(&xLastWakeTime, xDelay1ms);
						rtcVal = rtcRead(&rtc);
					}
					*/

					/*rtc.min = 0;
					rtcVal = rtcWrite(&rtc);

					nectarActualState.actualState = extrayendo;

					while (rtc.min < nectarTarget.tPasoDinamico) {

						tempOutControl(&pidTempOut, &tempOutControlVar,
								nectarTarget.tempSalida);

						tempPresuControl(&pidTempPresu, &tempPresuControlVar,
								nectarTarget.tempPresu);

						maxMinPPresuControl(&pPresuControlVar,
								nectarTarget.pPresu);

						tempExtrControl(&pidTempExtr, &tempExtrControlVar,
								nectarTarget.tempExt);

						maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

						if (xTaskGetTickCount() - xLastOut
								> nectarTarget.flujoSalida) {

							if (isOutValveOpen) {
								closeOutValve();
							} else {
								openOutValve();
							}
							xLastOut = xTaskGetTickCount();
						}

						if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

							nectarActualState.tempSalida = tempOutControlVar.B;
							nectarActualState.tempPresu = tempPresuControlVar.B;
							nectarActualState.pPresu = pPresuControlVar.acutalP;
							nectarActualState.tempExt = tempExtrControlVar.B;
							nectarActualState.pExt = pExtrControlVar.acutalP;

							xSemaphoreGive(xUartDatoToPrintSemaphore);
							xLastUartSend = xTaskGetTickCount();
						}

						vTaskDelayUntil(&xLastWakeTime, xDelay1ms);
						rtcVal = rtcRead(&rtc);
					}*/
					extraer(nectarTarget, &nectarActualState);

				}
				startProgram = false;
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

