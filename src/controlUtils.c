/*
 * controlUtils.c
 *
 *  Created on: 20 de jun. de 2018
 *      Author: ivan
 */

#include "controlUtils.h"

void tempExtrControl(control_pid_t *pid, control_variable_t *tempExtrControlVar,
		float tempTarget) {

	tempExtrControlVar->E = tempTarget - tempExtrControlVar->B;

	tempExtrControlVar->X = PID_Process(pid, tempExtrControlVar->E);
	tempExtrControlVar->X =
			(3.3 < tempExtrControlVar->X) ? 3.3 : tempExtrControlVar->X;

	dacWrite(DAC, (uint16_t) (tempExtrControlVar->X * (1023 / 3.3)));

	tempExtrControlVar->B = ((float) (adcRead(CH1)) * 3.3) / 1023;

}

void tempPresuControl(control_pid_t *pid,
		control_variable_t *tempPresuControlVar, float tempTarget) {

	bool_t valor;

	tempPresuControlVar->E = tempTarget - tempPresuControlVar->B;

	tempPresuControlVar->X = PID_Process(pid, tempPresuControlVar->E);
	tempPresuControlVar->X =
			(3.3 < tempPresuControlVar->X) ? 3.3 : tempPresuControlVar->X;

//	dacWrite(DAC, (uint16_t) (tempPresuControlVar->X * (1023 / 3.3)));
	   valor = pwmWrite( PWM0, (uint8_t)tempPresuControlVar->X*(255/3.3) );

	tempPresuControlVar->B = (float) ((adcRead(PIN_SEN_TEMP_PRES) - 204.6) * (200 / 819.4) - 50);

}
