/*
 * nectar.c
 *
 *  Created on: 19 de jun. de 2018
 *      Author: ivan
 */

#include "nectar.h"

uint8_t nectarInit(nectar_target_param_t *nectarTarget, uint16_t tempExt,
		uint16_t pExt, uint16_t tempPresu, uint16_t pPresu, uint16_t tempSalida,
		uint32_t tPasoEstatico, uint32_t tPasoDinamico, uint16_t nCiclos,
		uint16_t flujoSalida) {

	nectarTarget->tempExt = tempExt;
	nectarTarget->pExt = pExt;
	nectarTarget->tempPresu = tempPresu;
	nectarTarget->pPresu = pPresu;
	nectarTarget->tempSalida = tempSalida;
	nectarTarget->tPasoEstatico = tPasoEstatico;
	nectarTarget->tPasoDinamico = tPasoDinamico;
	nectarTarget->nCiclos = nCiclos;
	nectarTarget->flujoSalida = flujoSalida;
}


