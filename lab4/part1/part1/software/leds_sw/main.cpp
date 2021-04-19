/*
 * main.cpp
 *
 *  Created on: Mar 6, 2021
 *      Author: Davendra
 */
#include "../leds_sw_bsp/system.h"

int main(){

	// pointer to led base address in RAM
	int* led = (int*) LEDS_BASE;

	// pointer to switch base address in RAM
	int * switches = (int*) SWITCHES_BASE;

	// continuous loop reading switches to leds
	while(1){
		*led = *switches;
	}

	return 0;
}
