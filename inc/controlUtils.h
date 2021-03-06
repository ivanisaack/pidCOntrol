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
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "nectar.h"

typedef struct control_variable_s {

	float B;
	float E;
	float X;

} control_variable_t;

typedef struct max_min_control_s {

	float max;
	float min;
	float acutalP;
	float E;

} max_min_control_t;

void initControlTask(nectar_target_param_t *nectarTarget);

bool_t prepareCamExtr(nectar_target_param_t nectarTarget,
		nectar_actual_param_t *nectarActualState);

bool_t prepareCamPres(nectar_target_param_t nectarTarget,
		nectar_actual_param_t *nectarActualState);

void maserar(nectar_target_param_t nectarTarget,
		nectar_actual_param_t *nectarActualState);

void extraer(nectar_target_param_t nectarTarget,
		nectar_actual_param_t *nectarActualState);

void openPresuValve();
void closePresuValve();
void openOutValve();
void closeOutValve();

void initMaxMinControl(max_min_control_t *maxMinControlVar, float max,
		float min);
void maxMinPPresuControl(max_min_control_t *maxMinControlVar, float pTarget);
void maxMinPExtrControl(max_min_control_t *maxMinControlVar, float pTarget);
void initControlVariable(control_variable_t *controlVariable, float B, float E,
		float X);
void tempExtrControl(control_pid_t *pid, control_variable_t *tempExtrControlVar,
		float tempTarget);
void tempPresuControl(control_pid_t *pid,
		control_variable_t *tempPresuControlVar, float tempTarget);
void tempSalidaControl(control_pid_t *pid,
		control_variable_t *tempSalidaControlVar, float tempTarget);
void presExtrControl(control_pid_t *pid, control_variable_t *presExtrControlVar,
		float presTarget);
void presPresuControl(control_pid_t *pid,
		control_variable_t *presPresuControlVar, float presTarget);

void tempOutControl(control_pid_t *pid, control_variable_t *tempOutControlVar,
		float tempTarget);
//TODO:Agregar PWM para control de apertura de valvulas y para control te encendidos de resistencias.
#endif /* PROJECTS_CONTROLPID_INC_CONTROLUTILS_H_ */
