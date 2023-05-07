#include "hal.h"
#include "esp32-hal-ledc.h"


struct PinSettings{

};

struct PinAbilities{
  bool d_input = false;
  bool d_output = false;
  bool pwm_output = false;
  bool analog_in = false;
  bool dac_out = false;
  bool capacitive_touch = false;
  bool inturrupt = false;
  bool input_pullup = false;
  bool input_pulldown = false;
};


PinSettings pins[40] = {};
PinAbilities pin_abilities[40] = {};

void run_hal_initialization() {}



