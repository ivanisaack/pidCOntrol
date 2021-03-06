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

typedef enum {

	preparandoCamaraExtr = 1,
	preparandoCamaraPres = 2,
	maserando = 3,
	extrayendo = 4
	//TODO: AGREGAR ACTIVAR CONTROL, ELEGIR PID, KP,KI,KD

} nectar_state_t;


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

typedef struct nectar_actual_param_s{

	float tempExt;
	float pExt;
	float tempPresu;
	float pPresu;
	float tempSalida;
	uint32_t tPasoEstatico;
	uint32_t tPasoDinamico;
	uint16_t Ciclo;
	nectar_state_t actualState;

}nectar_actual_param_t;

uint8_t nectarInit(nectar_target_param_t *nectarTarget, uint16_t tempExt,
		uint16_t pExt, uint16_t tempPresu, uint16_t pPresu, uint16_t tempSalida,
		uint32_t tPasoEstatico, uint32_t tPasoDinamico, uint16_t nCiclos,
		uint16_t flujoSalida);
void initNectarActualState(nectar_actual_param_t *nectarActualState);

#endif /* PROJECTS_CONTROLPID_INC_NECTAR_H_ */
