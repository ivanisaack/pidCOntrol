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

	maxMinControlVar->acutalP = ((float) (adcRead(PIN_SEN_PRESI_EXTR)) * 3.3)
			/ 1024;
	maxMinControlVar->E = abs(maxMinControlVar->acutalP - pTarget);
	if ((maxMinControlVar->acutalP >= maxMinControlVar->max)
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
	pwmWrite(PWM7, (uint8_t) (tempPresuControlVar->X * 255 / 3.3));

	//TODO: ESTA SERIA LA FUNCION DE LEER TEMPERATURA;

	tempPresuControlVar->B = ((float) (adcRead(CH1)) * 3.3) / 1024;

}
