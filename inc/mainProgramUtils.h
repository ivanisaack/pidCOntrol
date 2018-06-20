/*
 * mainProgramUtils.h
 *
 *  Created on: 18 de jun. de 2018
 *      Author: ivan
 */

#ifndef PROJECTS_CONTROLPID_INC_MAINPROGRAMUTILS_H_
#define PROJECTS_CONTROLPID_INC_MAINPROGRAMUTILS_H_

#include "sapi.h"
#include "utils.h"
#include "nectar.h"
#include "string.h"

static const char startToken = '$';
static const char printAllToken = '$';
static const char resetAllToken = 'R';
static const char assignToken = '=';
static const char separatorToken = '\n';
static const char responseConfirm[] = "ok";
static const char responseError[] = "err";
static const char identifyToken = '?';
static const char responseIndentify[] = "Nectar";

typedef enum {

	tempExt = 1,
	pExt = 2,
	tempPresu = 3,
	pPresu = 4,
	tempSalida = 5,
	tPasoEstatico = 6,
	tPasoDinamico = 7,
	nCiclos = 8,
	flujoSalida = 9
	//TODO: AGREGAR ACTIVAR CONTROL, ELEGIR PID, KP,KI,KD

} comProtocolVar_t;

static const size_t comProtocolVarSize = 9;

void vPrintString( char * string);
void vPrintNumber( int32_t number);

bool_t processSerialPort(nectar_target_param_t * nectarTarget);
bool_t parseCommand(nectar_target_param_t *nectarTarget);
bool_t printCommand(nectar_target_param_t *nectarTarget);
#endif /* PROJECTS_CONTROLPID_INC_MAINPROGRAMUTILS_H_ */
