/*
 * mainProgramUtils.c
 *
 *  Created on: 18 de jun. de 2018
 *      Author: ivan
 */

#include "mainProgramUtils.h"

bool_t processSerialPort(char* str) {

	bool_t result = false;
	uint8_t dataUart;
	char command[3];
	int i = 0;
	while (uartReadByte(UART_USB, &dataUart)) {

		str[i] = dataUart;
		i++;
		result = true;
	}
	if (result) {
		str[i] = '\n';
	}

	return result;
}
