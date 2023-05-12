#pragma once
#include "Arduino.h"



void run_hal_initialization();



void hal_disable_output();
void hal_enable_output();
void hal_force_disable();
void hal_brownout_output_start();
void hal_brownout_output_stop();


struct HandleCreation{
  /*no peaking ;)*/
};

class PinHandle{
 private:
  uint8_t pin = 255;

 public:
 uint8_t physicalPinNumber();
 PinHandle();
 PinHandle(uint8_t pin, HandleCreation* ctx);
};

PinHandle create_analog_input(uint8_t pin);
uint16_t analog_input_read(PinHandle& handle);
void delete_analog_input(PinHandle handle);


enum class InputPinMode{
  Floating,
  PullUp,
  PullDown,
};

PinHandle create_digital_input(uint8_t pin, InputPinMode pullup_mode);
bool digital_input_read(PinHandle& handle);
void delete_digital_input(PinHandle handle);

PinHandle create_capacitive_input(uint8_t pin);
uint16_t capacitive_input_read(PinHandle& handle);
void delete_capacitive_input(PinHandle handle);


PinHandle create_inturrupt_pin(uint8_t pin);
bool inturrupt_pin_read(PinHandle& handle);
void delete_inturrupt_pin(PinHandle handle);

PinHandle create_digital_output(uint8_t pin, bool disabled_value = false, bool brownout_val = false,
                                bool should_disable_output = true, bool should_brown_out = true);
bool digital_output_read(PinHandle& handle);
void digital_output_write(PinHandle& handle, bool value);
void delete_digital_output(PinHandle handle);


struct PwmHandle{
 private:
  uint8_t _channel = 255;
 public:
  uint8_t channel();
  PwmHandle();
  PwmHandle(uint8_t channel, HandleCreation* ctx);
};

PwmHandle create_pwm_generator(uint8_t resolution, uint32_t frequecy, uint32_t phase);
void update_pwm_duty(PwmHandle& handle, uint32_t phase);
void update_pwm_frequency(PwmHandle& handle, uint32_t frequency);
void update_pwm_resolution(PwmHandle& handle, uint8_t resolution);
void delete_pwm_generator(PwmHandle handle);

PinHandle create_pwn_output(uint8_t pin, PwmHandle& pwm_handle, bool disabled_value = false, bool brownout_val = false,
                                bool should_disable_output = true, bool should_brown_out = true);
void delete_pwm_output(PinHandle handle);