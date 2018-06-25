/*
 * mainProgramUtils.c
 *
 *  Created on: 18 de jun. de 2018
 *      Author: ivan
 */

#include "mainProgramUtils.h"
char command[5];
char value[10];

void vPrintString(char * string) {
	uartWriteString(UART_USB, string);
}

void vPrintNumber(int32_t number) {
	uint8_t uartBuff[10];
	/* Conversión de number entero a ascii con base decimal */
	itoa(number, uartBuff, 10); /* 10 significa decimal */
	/* Enviar number */
	uartWriteString(UART_USB, uartBuff);
}

bool_t processSerialPort(nectar_target_param_t *nectarTarget,
		bool_t *startProgram) {

	char str[20];

	bool_t result = false;
	uint8_t dataUart;
	char *pAssignToken, *pSeparatorToken;
	size_t commandLen, valueLen;

	size_t uartDataIterator = 0;

	while (uartReadByte(UART_USB, &dataUart)) {

		str[uartDataIterator] = dataUart;
		uartDataIterator++;
		result = true;
		vTaskDelay(xDelayUart1ms);

	}
	if (result) {
		str[uartDataIterator] = '\n';
		if (!strncmp(str, "$", 1)) {

			pAssignToken = strchr(str, assignToken);

			if (pAssignToken != NULL) {

				commandLen = pAssignToken - str - 1;
				strncpy(command, &str[1], commandLen);
				command[commandLen] = '\0';

				pSeparatorToken = strchr(pAssignToken + 1, separatorToken);

				if (pSeparatorToken != NULL) {

					valueLen = pSeparatorToken - str - 2 - commandLen;

					strncpy(value, &str[commandLen + 2], valueLen);
					value[valueLen] = '\0';

					if (parseCommand(nectarTarget, startProgram)){
						vPrintString(responseConfirm);
						vPrintString("\r\n");
					}
					else{
						vPrintString(responseError);
						vPrintString("\r\n");

					}

					return true;
				}
			} else if (uartDataIterator < 5) {
				strncpy(command, &str[1], uartDataIterator - 2);
				command[uartDataIterator - 1] = '\0';

				printCommand(nectarTarget, startProgram);

				return true;
			}
		}
		return false;
	}

	return result;
}

bool_t parseCommand(nectar_target_param_t *nectarTarget, bool_t *startProgram) {

	comProtocolVar_t commandVar;
	uint32_t comValue;
	uint32_t auxLong;
	char *ptr;

	auxLong = strtol(command, &ptr, 10);

	if (auxLong != 0) {

		commandVar = auxLong;

	} else
		return false;

	comValue = strtol(value, &ptr, 10);
	if (!(*ptr == '\0')){
		vPrintString(responseError);
		vPrintString("\r\n");
		return false;
	}
	//vPrintNumber(comValue);
	//vPrintString(" es eso\n");

	switch (commandVar) {

	case tempExt: {

		nectarTarget->tempExt = realTempToScaledTemp((float) comValue);
	//	vPrintNumber((int32_t)nectarTarget.tempExt);
		return true;
	}
	case pExt: {

		nectarTarget->pExt =  realPresToScaledPres((float) comValue);
		;
		return true;
	}
	case tempPresu: {

		nectarTarget->tempPresu = realTempToScaledTemp((float) comValue);
		return true;
	}
	case pPresu: {

		nectarTarget->pPresu = realPresToScaledPres((float) comValue);
		return true;
	}
	case tempSalida: {

		nectarTarget->tempSalida = comValue;
		return true;
	}
	case tPasoEstatico: {

		nectarTarget->tPasoEstatico = comValue;
		return true;
	}
	case tPasoDinamico: {

		nectarTarget->tPasoDinamico = comValue;
		return true;
	}
	case nCiclos: {

		nectarTarget->nCiclos = comValue;
		return true;
	}
	case flujoSalida: {

		nectarTarget->flujoSalida = comValue;
		return true;
	}
	case comenzarCiclo: {

		*startProgram = (bool_t) !(comValue == 0);
		return true;
	}
	default: {
		return false;
	}
	}
	return true;
}

bool_t printCommand(nectar_target_param_t *nectarTarget, bool_t *startProgram) {

	comProtocolVar_t commandVar;
	uint32_t auxLong;
	char *ptr;

	auxLong = strtol(command, &ptr, 10);

	if (auxLong != 0) {

		commandVar = auxLong;
	} else
		return false;

	switch (commandVar) {

	case tempExt: {

		vPrintNumber((int16_t) ((200 / 3.3) * nectarTarget->tempExt) - 50);
		vPrintString("\r\n");
		return true;
	}
	case pExt: {

		vPrintNumber((int16_t) ((100 / 3.3) * nectarTarget->pExt));
		vPrintString("\r\n");
		return true;
	}
	case tempPresu: {

		vPrintNumber((int16_t) ((200 / 3.3) * nectarTarget->tempPresu) - 50);
		vPrintString("\r\n");
		return true;
	}
	case pPresu: {

		vPrintNumber((int16_t) ((100 / 3.3) * nectarTarget->pPresu));
		vPrintString("\r\n");
		return true;
	}
	case tempSalida: {

		vPrintNumber(nectarTarget->tempSalida);
		vPrintString("\r\n");
		return true;
	}
	case tPasoEstatico: {

		vPrintNumber(nectarTarget->tPasoEstatico);
		vPrintString("\r\n");
		return true;
	}
	case tPasoDinamico: {

		vPrintNumber(nectarTarget->tPasoDinamico);
		vPrintString("\r\n");
		return true;
	}
	case nCiclos: {

		vPrintNumber(nectarTarget->nCiclos);
		vPrintString("\r\n");
		return true;
	}
	case flujoSalida: {

		vPrintNumber(nectarTarget->flujoSalida);
		vPrintString("\r\n");
		return true;
	}
	/*case comenzarCiclo: {

		vPrintNumber(*startProgram);
		vPrintString("\r\n");
	}*/

	default: {
		return false;
	}
	}
	return true;

}
