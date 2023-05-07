#pragma once
#include "Arduino.h"


void run_hal_initialization();

void hal_disable_output();
void hal_enable_output();
void hal_force_disable();
void hal_brownout_output_start();
void hal_brownout_output_stop();