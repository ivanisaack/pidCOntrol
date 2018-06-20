/*
 * mainProgramUtils.c
 *
 *  Created on: 18 de jun. de 2018
 *      Author: ivan
 */

#include "mainProgramUtils.h"
char command[5];
char value[10];

void vPrintString( char * string){
   uartWriteString( UART_USB, string );
}

void vPrintNumber( int32_t number){
   uint8_t uartBuff[10];
   /* Conversión de number entero a ascii con base decimal */
   itoa( number, uartBuff, 10 ); /* 10 significa decimal */
   /* Enviar number */
   uartWriteString( UART_USB, uartBuff );
}

bool_t processSerialPort(nectar_target_param_t *nectarTarget) {

	char str[20];

	bool_t result = false;
	uint8_t dataUart;
	char *pAssignToken, *pSeparatorToken;
	size_t commandLen, valueLen;

	size_t uartDataIterator = 0;
	nectarTarget->pExt = 3;

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

					parseCommand(	nectarTarget);

					return true;
				}
			} else if (uartDataIterator < 5) {
				strncpy(command, &str[1], uartDataIterator - 2);
				command[uartDataIterator - 1] = '\0';

				printCommand(nectarTarget);

				return true;
			}
		}
		return false;
	}

	return result;
}

bool_t parseCommand(nectar_target_param_t *nectarTarget) {

	comProtocolVar_t commandVar;
	uint32_t comValue;
	uint32_t auxLong;
	char *ptr;

	auxLong = strtol(command, &ptr, 10);

	if (auxLong != 0) {

		commandVar = auxLong;
		vPrintString("command var:");
		vPrintNumber(commandVar);
		vPrintString("\n");
	}
	else
		return false;

	comValue = strtol(value, &ptr, 10);

	//vPrintNumber(comValue);
	//vPrintString(" es eso\n");

	switch (commandVar) {

	case tempExt: {

		nectarTarget->tempExt = comValue;
		break;
	}
	case pExt: {

		nectarTarget->pExt = comValue;
		break;
	}
	case tempPresu: {

		nectarTarget->tempPresu = comValue;
		break;
	}
	case pPresu: {

		//nectarTarget->pPresu = comValue;
		break;
	}
	case tempSalida: {

		//nectarTarget->tempSalida = comValue;
		break;
	}
	case tPasoEstatico: {

		//nectarTarget->tPasoEstatico = comValue;
		break;
	}
	case tPasoDinamico: {

		//nectarTarget->tPasoDinamico = comValue;
		break;
	}
	case nCiclos: {

		//nectarTarget->nCiclos = comValue;
		break;
	}
	case flujoSalida: {

		nectarTarget->flujoSalida = comValue;
		break;
	}
	default: {
		return false;
	}
	}
	return true;
}

bool_t printCommand(nectar_target_param_t *nectarTarget){

	comProtocolVar_t commandVar;
		uint32_t auxLong;
		char *ptr;

		auxLong = strtol(command, &ptr, 10);

		if (!auxLong) {

			commandVar = auxLong;
		} else
			return false;



		switch (commandVar) {

		case tempExt: {

			vPrintNumber(nectarTarget->tempExt);
			break;
		}
		case pExt: {

			vPrintNumber(nectarTarget->pExt);
			break;
		}
		case tempPresu: {

			vPrintNumber(nectarTarget->tempPresu);
			break;
		}
		case pPresu: {

			vPrintNumber(nectarTarget->pPresu);
			break;
		}
		case tempSalida: {

			vPrintNumber(nectarTarget->tempSalida);
			break;
		}
		case tPasoEstatico: {

			vPrintNumber(nectarTarget->tPasoEstatico);
			break;
		}
		case tPasoDinamico: {

			vPrintNumber(nectarTarget->tPasoDinamico);
			break;
		}
		case nCiclos: {

			vPrintNumber(nectarTarget->nCiclos);
			break;
		}
		case flujoSalida: {

			vPrintNumber(nectarTarget->flujoSalida);
			break;
		}
		default: {
			return false;
		}
		}
		return true;


}
