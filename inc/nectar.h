/*
 * nectar.h
 *
 *  Created on: 19 de jun. de 2018
 *      Author: ivan
 */

#ifndef PROJECTS_CONTROLPID_INC_NECTAR_H_
#define PROJECTS_CONTROLPID_INC_NECTAR_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct nectar_target_param_s{

	float tempExt;
	float pExt;
	float tempPresu;
	float pPresu;
	float tempSalida;
	uint32_t tPasoEstatico;
	uint32_t tPasoDinamico;
	uint16_t nCiclos;
	uint16_t flujoSalida;

}nectar_target_param_t;

typedef struct nectar_actual_state_s{

	float tempExt;
	float pExt;
	float tempPresu;
	float pPresu;
	float tempSalida;
	uint32_t tPasoEstatico;
	uint32_t tPasoDinamico;
	uint16_t Ciclo;

}nectar_actual_state_t;

void initNectarActualState(nectar_actual_state_t *nectarActualState);

#endif /* PROJECTS_CONTROLPID_INC_NECTAR_H_ */
