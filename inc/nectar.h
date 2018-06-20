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

	uint16_t tempExt;
	uint16_t pExt;
	uint16_t tempPresu;
	uint16_t pPresu;
	uint16_t tempSalida;
	uint32_t tPasoEstatico;
	uint32_t tPasoDinamico;
	uint16_t nCiclos;
	uint16_t flujoSalida;

}nectar_target_param_t;




#endif /* PROJECTS_CONTROLPID_INC_NECTAR_H_ */
