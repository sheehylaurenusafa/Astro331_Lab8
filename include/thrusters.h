#pragma once

#include <Arduino.h>

/*---------------------------------------------------------------------------------------------*/
// Function Prototypes (see definitions in .cpp file):
/*---------------------------------------------------------------------------------------------*/
void thrusters_init();
void thrusters_set(float cmd);
void thrusters_set_each(float plusZ, float minusZ);
void thrusters_off();
float thrusters_get_plusZ();
float thrusters_get_minusZ();
