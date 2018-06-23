#ifndef UTILS_H
#define UTILS_H

#include "sapi.h"
#include "FreeRTOS.h"

#define PIN_CALENTADOR_PRES GPIO0
#define PIN_CALENTADOR_EXTR GPIO3

#define PIN_SEN_PRESI_EXTR AI0
#define PIN_SEN_TEMP_PRES AI1
#define PIN_SEN_TEMP_EXTR AI2
#define PIN_SEN_PRESI_PRES AI3

#define MAX_TEMP 3
#define MAX_CRITICAL_TEMP 3.2
#define temperatureDataLen 1

#define xDelayUart1ms  ( (TickType_t) 1UL / portTICK_RATE_MS)
#define xDelay1ms  ( (TickType_t) 1UL / portTICK_RATE_MS)
#define xDelay500ms  ( (TickType_t) 500UL / portTICK_RATE_MS)
#define xDelay1s  ( (TickType_t) 1000UL / portTICK_RATE_MS)

typedef struct {
	tick_t startTime;
	tick_t duration;
	bool_t running;
} delayTick_t;

float scaledTempToRealTemp(float scaledTemp);
float realTempToScaledTemp(float realTemp);
float scaledPresToRealPres(float scaledPres);
float realPresToScaledPres(float realPres);
char* itoa(int value, char* result, int base);
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);

#endif //UTILS.H
