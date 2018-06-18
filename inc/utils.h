#ifndef UTILS_H
#define UTILS_H

#include "sapi.h"
#include "FreeRTOS.h"
#define MAX_TEMP 3
#define MAX_CRITICAL_TEMP 3.2
#define temperatureDataLen 1

typedef struct {
	tick_t startTime;
	tick_t duration;
	bool_t running;
} delayTick_t;

void delayConfigII(delayTick_t * delay, tick_t duration);
bool_t delayReadII(delayTick_t * delay);
char* itoa(int value, char* result, int base);
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);

#endif //UTILS.H
