/*
 * controlUtils.c
 *
 *  Created on: 20 de jun. de 2018
 *      Author: ivan
 */

#include "controlUtils.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

static bool_t isValvesForExtrOpen = false;
static bool_t isValvePresuOpen = false;
static bool_t isValveOutOpen = false;
static TickType_t xLastWakeTime, xLastUartSend, xLastOut;
static control_variable_t tempExtrControlVar, tempPresuControlVar,
		tempOutControlVar;
static control_pid_t pidTempExtr, pidTempPresu, pidTempOut;

static max_min_control_t pExtrControlVar, pPresuControlVar;

static float KpTempExtr, KiTempExtr, KdTempExtr, TloopTempExtr;
static float KpTempPresu, KiTempPresu, KdTempPresu, TloopTempPresu;
static float KpTempOut, KiTempOut, KdTempOut, TloopTempOut;

static float maxPExtr, minPExtr;
static float maxPPresu, minPPresu;

static bool_t isOutValveOpen = false;

static rtc_t rtc;
static bool_t rtcVal = false;

void initControlTask(nectar_target_param_t *nectarTarget) {

	nectarInit(nectarTarget, 0, 0, 0, 0, 0, 0, 0, 0, 0);

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

	rtcVal = rtcConfig(&rtc);

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
	xLastOut = xLastWakeTime;

	vTaskDelay(2 * xDelay1s);

}

bool_t prepareCamExtr(nectar_target_param_t nectarTarget,
		nectar_actual_param_t *nectarActualState) {

	bool_t isCamExtrReady = false;

	nectarActualState->actualState = preparandoCamaraExtr;
//TODO: SACAR EL WHILE Y PONERLO EN MAIN
	while (!isCamExtrReady) {

		tempExtrControl(&pidTempExtr, &tempExtrControlVar,
				nectarTarget.tempExt);

		maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

		if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

			nectarActualState->tempExt = tempExtrControlVar.B;
			nectarActualState->pExt = pExtrControlVar.acutalP;

			xSemaphoreGive(xUartDatoToPrintSemaphore);
			xLastUartSend = xTaskGetTickCount();
		}

		vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

		/*if (pExtrControlVar.E < 2.0 && tempExtrControlVar.E < 2.0) {
		 return true;
		 }
		 else{
		 return false;}*/

	}
	return false; //todo:SOLO PARA TEST. BORRAR

}

bool_t prepareCamPres(nectar_target_param_t nectarTarget,
		nectar_actual_param_t *nectarActualState) {

	bool_t isCamPresuReady = false;

	nectarActualState->actualState = preparandoCamaraPres;

	while (!isCamPresuReady) {

		//TODO: Podria hacer un vector CON VARIABLES DE CONTROL DE T y OTRO DE P

		tempPresuControl(&pidTempPresu, &tempPresuControlVar,
				nectarTarget.tempPresu);

		maxMinPPresuControl(&pPresuControlVar, nectarTarget.pPresu);

		tempExtrControl(&pidTempExtr, &tempExtrControlVar,
				nectarTarget.tempExt);

		maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

		if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

			nectarActualState->tempPresu = tempPresuControlVar.B;
			nectarActualState->pPresu = pPresuControlVar.acutalP;
			nectarActualState->tempExt = tempExtrControlVar.B;
			nectarActualState->pExt = pExtrControlVar.acutalP;

			xSemaphoreGive(xUartDatoToPrintSemaphore);
			xLastUartSend = xTaskGetTickCount();
		}

		if (pPresuControlVar.E < 2.0 && tempPresuControlVar.E < 2.0) {
			isCamPresuReady = true;
		}

		vTaskDelayUntil(&xLastWakeTime, xDelay1ms);

	}
	return true;
}

void maserar(nectar_target_param_t nectarTarget,
		nectar_actual_param_t *nectarActualState) {

	nectarActualState->actualState = maserando;

	rtc.min = 0;
	rtcVal = rtcWrite(&rtc);

	while (rtc.min < nectarTarget.tPasoEstatico) {

		tempPresuControl(&pidTempPresu, &tempPresuControlVar,
				nectarTarget.tempPresu);

		maxMinPPresuControl(&pPresuControlVar, nectarTarget.pPresu);

		tempExtrControl(&pidTempExtr, &tempExtrControlVar,
				nectarTarget.tempExt);

		maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

		if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

			nectarActualState->tempPresu = tempPresuControlVar.B;
			nectarActualState->pPresu = pPresuControlVar.acutalP;
			nectarActualState->tempExt = tempExtrControlVar.B;
			nectarActualState->pExt = pExtrControlVar.acutalP;

			xSemaphoreGive(xUartDatoToPrintSemaphore);
			xLastUartSend = xTaskGetTickCount();
		}

		vTaskDelayUntil(&xLastWakeTime, xDelay1ms);
		rtcVal = rtcRead(&rtc);
	}

}

void extraer(nectar_target_param_t nectarTarget,
		nectar_actual_param_t *nectarActualState) {

	rtc.min = 0;
	rtcVal = rtcWrite(&rtc);

	nectarActualState->actualState = extrayendo;

	while (rtc.min < nectarTarget.tPasoDinamico) {

		tempOutControl(&pidTempOut, &tempOutControlVar,
				nectarTarget.tempSalida);

		tempPresuControl(&pidTempPresu, &tempPresuControlVar,
				nectarTarget.tempPresu);

		maxMinPPresuControl(&pPresuControlVar, nectarTarget.pPresu);

		tempExtrControl(&pidTempExtr, &tempExtrControlVar,
				nectarTarget.tempExt);

		maxMinPExtrControl(&pExtrControlVar, nectarTarget.pExt);

		if (xTaskGetTickCount() - xLastOut > nectarTarget.flujoSalida) {

			if (isOutValveOpen) {
				closeOutValve();
			} else {
				openOutValve();
			}
			xLastOut = xTaskGetTickCount();
		}

		if ((xTaskGetTickCount() - xLastUartSend) > xDelay500ms) {

			nectarActualState->tempSalida = tempOutControlVar.B;
			nectarActualState->tempPresu = tempPresuControlVar.B;
			nectarActualState->pPresu = pPresuControlVar.acutalP;
			nectarActualState->tempExt = tempExtrControlVar.B;
			nectarActualState->pExt = pExtrControlVar.acutalP;

			xSemaphoreGive(xUartDatoToPrintSemaphore);
			xLastUartSend = xTaskGetTickCount();
		}

		vTaskDelayUntil(&xLastWakeTime, xDelay1ms);
		rtcVal = rtcRead(&rtc);
	}

}

void openPresuValve() {

	gpioWrite(PIN_VALVULA_PRESU, ON);

}

void closePresuValve() {

	gpioWrite(PIN_VALVULA_PRESU, OFF);

}

void openOutValve() {

	gpioWrite(PIN_VALVULA_SALIDA, ON);

}

void closeOutValve() {

	gpioWrite(PIN_VALVULA_SALIDA, OFF);

}

void initMaxMinControl(max_min_control_t *maxMinControlVar, float max,
		float min) {

	maxMinControlVar->max = max;
	maxMinControlVar->min = min;
	maxMinControlVar->acutalP = 0.0;
}

void maxMinPExtrControl(max_min_control_t *maxMinControlVar, float pTarget) {

	maxMinControlVar->acutalP = ((float) (adcRead(CH1)) * 3.3) / 1024;
	maxMinControlVar->E = abs(maxMinControlVar->acutalP - pTarget);
	/*if ((maxMinControlVar->acutalP >= maxMinControlVar->max)
	 && (isValvesForExtrOpen == false)) {

	 openPresuValve();
	 openOutValve();
	 isValvesForExtrOpen = true;
	 isValveOutOpen = true;

	 } else if ((maxMinControlVar->acutalP <= maxMinControlVar->min)
	 && (isValvesForExtrOpen == true)) {

	 closePresuValve();
	 closeOutValve();
	 isValvesForExtrOpen = false;
	 isValveOutOpen = false;
	 }
	 vTaskDelay(xDelay100ms);
	 */
}

void maxMinPPresuControl(max_min_control_t *maxMinControlVar, float pTarget) {

	maxMinControlVar->acutalP = ((float) (adcRead(PIN_SEN_PRESI_PRES)) * 3.3)
			/ 1024;
	maxMinControlVar->E = abs(maxMinControlVar->acutalP - pTarget);

	if ((maxMinControlVar->acutalP <= maxMinControlVar->min)
			&& (isValvePresuOpen == false)) {

		if (isValveOutOpen) {
			closeOutValve();
			isValveOutOpen = false;
		}
		openPresuValve();
		isValvePresuOpen = true;

	} else if ((maxMinControlVar->acutalP >= maxMinControlVar->max)
			&& (isValvePresuOpen == true) && (isValveOutOpen == false)) {

		closePresuValve();
		openOutValve();
		isValvePresuOpen = false;
		isValveOutOpen = true;
	}

	vTaskDelay(xDelay100ms);

}

void initControlVariable(control_variable_t *controlVariable, float B, float E,
		float X) {

	controlVariable->B = B;
	controlVariable->E = E;
	controlVariable->X = X;

}

void tempExtrControl(control_pid_t *pid, control_variable_t *tempExtrControlVar,
		float tempTarget) {

	tempExtrControlVar->E = tempTarget - tempExtrControlVar->B;

	tempExtrControlVar->X = PID_Process(pid, tempExtrControlVar->E);
	tempExtrControlVar->X =
			(3.3 < tempExtrControlVar->X) ? 3.3 : tempExtrControlVar->X;

//TODO: ESTA SERIA LA FUNCION DE PRENDER CALENTADOR
	dacWrite(DAC, (uint16_t) (tempExtrControlVar->X * (1024 / 3.3)));

//TODO: ESTA SERIA LA FUNCION DE LEER TEMPERATURA;
	tempExtrControlVar->B = ((float) (adcRead(CH1)) * 3.3) / 1024;

}

void tempPresuControl(control_pid_t *pid,
		control_variable_t *tempPresuControlVar, float tempTarget) {

	bool_t valor;

	tempPresuControlVar->E = tempTarget - tempPresuControlVar->B;

	tempPresuControlVar->X = PID_Process(pid, tempPresuControlVar->E);
	tempPresuControlVar->X =
			(3.3 < tempPresuControlVar->X) ? 3.3 : tempPresuControlVar->X;

//TODO: ESTA SERIA LA FUNCION DE PRENDER CALENTADOR
//TODO: CAMBIAR FRECUENCIA PWM A 20HZ.
//pwmWrite(PIN_CALENTADOR_PRES, (uint8_t) (tempPresuControlVar->X * 255 / 3.3));

//TODO: ESTA SERIA LA FUNCION DE LEER TEMPERATURA;

	tempPresuControlVar->B = ((float) (adcRead(PIN_SEN_TEMP_PRES)) * 3.3)
			/ 1024;

}

void tempOutControl(control_pid_t *pid, control_variable_t *tempOutControlVar,
		float tempTarget) {

	bool_t valor;

	tempOutControlVar->E = tempTarget - tempOutControlVar->B;

	tempOutControlVar->X = PID_Process(pid, tempOutControlVar->E);
	tempOutControlVar->X =
			(3.3 < tempOutControlVar->X) ? 3.3 : tempOutControlVar->X;

//TODO: ESTA SERIA LA FUNCION DE PRENDER CALENTADOR
//TODO: CAMBIAR FRECUENCIA PWM A 20HZ.
//	pwmWrite(PIN_CALENTADOR_SALIDA, (uint8_t) (tempOutControlVar->X * 255 / 3.3));

//TODO: ESTA SERIA LA FUNCION DE LEER TEMPERATURA;

	tempOutControlVar->B = ((float) (adcRead(CH1)) * 3.3) / 1024;

}

