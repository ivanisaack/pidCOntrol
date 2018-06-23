/*
 * controlUtils.h
 *
 *  Created on: 20 de jun. de 2018
 *      Author: ivan
 */

#ifndef PROJECTS_CONTROLPID_INC_CONTROLUTILS_H_
#define PROJECTS_CONTROLPID_INC_CONTROLUTILS_H_

#include "sapi.h"
#include "utils.h"
#include "pid.h"

typedef struct control_variable_s {

	float B;
	float E;
	float X;

} control_variable_t;

void initControlVariable(control_variable_t *controlVariable, float B, float E, float X);
void tempExtrControl(control_pid_t *pid, control_variable_t *tempExtrControlVar,
		float tempTarget);
void tempPresuControl(control_pid_t *pid, control_variable_t *tempPresuControlVar,
		float tempTarget);
void tempSalidaControl(control_pid_t *pid, control_variable_t *tempSalidaControlVar,
		float tempTarget);
void presExtrControl(control_pid_t *pid, control_variable_t *presExtrControlVar,
		float presTarget);
void presPresuControl(control_pid_t *pid, control_variable_t *presPresuControlVar,
		float presTarget);

//TODO:Agregar PWM para control de apertura de valvulas y para control te encendidos de resistencias.
#endif /* PROJECTS_CONTROLPID_INC_CONTROLUTILS_H_ */
