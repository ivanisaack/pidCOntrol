/*
 * utils.c
 *
 *  Created on: 18 de jun. de 2018
 *      Author: ivan
 */

#include "utils.h"

char* itoa(int value, char* result, int base) {
	// check that the base if valid
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ =
				"zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35
						+ (tmp_value - value * base)];
	} while (value);

	// Apply negative sign
	if (tmp_value < 0)
		*ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

void reverse(char *str, int len) {
	int i = 0, j = len - 1, temp;
	while (i < j) {
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d) {
	int i = 0;
	while (x) {
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint) {
	// Extract integer part
	int ipart = (int) n;

	if (ipart == 0) {
		itoa(ipart, res, 10);
	} else {
		// Extract floating part
		float fpart = n - (float) ipart;

		// convert integer part to string
		int i = intToStr(ipart, res, 0);

		// check for display option after point
		if (afterpoint != 0) {
			res[i] = '.';  // add dot

			// Get the value of fraction part upto given no.
			// of points after dot. The third parameter is needed
			// to handle cases like 233.007
			fpart = fpart * pow(10, afterpoint);

			intToStr((int) fpart, res + i + 1, afterpoint);
		}
	}
}

float scaledTempToRealTemp(float scaledTemp) {

	return ((200 / 3.3) * scaledTemp) - 50;

}
float realTempToScaledTemp(float realTemp) {

	return realTemp * (3.3 / 200) + 0.825;
}
float scaledPresToRealPres(float scaledPres) {

	return ((100 / 3.3) * scaledPres);

}
float realPresToScaledPres(float realPres) {

	return realPres * (3.3 / 100);
}
