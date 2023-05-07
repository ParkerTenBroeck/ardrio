#include "hal.h"

#include <atomic>

#include "esp32-hal-ledc.h"

//------------------ pin handle

uint8_t PinHandle::physicalPinNumber() { return this->pin; }

PinHandle::PinHandle() { this->pin = 255; }

PinHandle::PinHandle(uint8_t pin, PinHandleCreation* ctx) { this->pin = pin; }

//------------------ pin handle

PinHandleCreation* create_pin_handle_creation() { return NULL; }

enum PinSettingVarient {
  StrappingPin,
  DInputFloating,
  DInputPullUp,
  DInputPullDown,
  DOutput,
  PwmOutput,
  AnalogIn,
  DacOutput,
  CapacitiveTouchIn,
  InturruptInput,

  // i2c
  SclPin,
  SdaPin,

  // uart
  U2_TXD,
  U2_RXD,

  // spi
  // hspi
  HSPI_CLK,   // 14
  HSPI_MOSO,  // 12
  HSPI_MOSI,  // 13
  HSPI_CS,    // 15
  // vspi
  VSPI_MOSI,  // 23
  VSPI_MOSO,  // 19
  VSPI_CLK,   // 18
  VSPI_CS,    // 5

  RTC,

  // don't use this
  NUM_PIN_SETTINGS,

  None,
};

struct DOutputSetting {
  bool current_val : 1;
  bool disable_val : 1;
  bool should_stop_output : 1;
};

struct PwmOutputSetting {};

struct DacOutputSetting {
  uint16_t val;
};

union PinSettingVarients {
  DOutputSetting DOutput;
  DacOutputSetting DacOutput;
  PwmOutputSetting PwmOutput;
};

struct PinSetting {
  PinSettingVarient varient;
  PinSettingVarients varients;
};

struct PinAbilities {
  bool abilities[NUM_PIN_SETTINGS];

  template <typename... Args>
  PinAbilities(Args... args) {
    for (int i = 0; i < NUM_PIN_SETTINGS; i++) {
      this->abilities[i] = false;
    }
    this->add_abilities(args...);
  }

  template <typename T, typename... Args>
  void add_abilities(T t, Args... args) {
    this->abilities[t] = true;
    add_abilities(args...);
  }

  template <typename T>
  void add_abilities(T t) {
    this->abilities[t] = true;
  }

  void add_abilities() {}
};

#define NUM_PINS 40
std::atomic<bool> pin_settings_lock(false);
PinSetting pin_settings[NUM_PINS] = {};

const PinAbilities pin_abilities[NUM_PINS] = {
    PinAbilities(),  // 00-D0
    PinAbilities(),  // 01-TX0 Used for
    PinAbilities(StrappingPin, DInputPullDown, DInputPullUp, DInputPullUp,
                 DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn,
                 PwmOutput, RTC),  // 02-D2
    PinAbilities(),                // 03-RX0
    PinAbilities(DInputPullDown, DInputPullUp, DInputPullUp, DOutput,
                 InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput,
                 RTC),  // 04-D4
    PinAbilities(StrappingPin, DInputPullDown, DInputPullUp, DInputPullUp,
                 DOutput, InturruptInput, PwmOutput, VSPI_CS),  // 05-D5
    PinAbilities(),                                             // 06-D6
    PinAbilities(),                                             // 07-D7
    PinAbilities(),                                             // 08-D8
    PinAbilities(),                                             // 09-D9
    PinAbilities(),                                             // 10-D10
    PinAbilities(),                                             // 11-D11
    PinAbilities(StrappingPin, DInputPullDown, DInputPullUp, DOutput,
                 InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC,
                 HSPI_MOSO),  // 12-D12
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 AnalogIn, CapacitiveTouchIn, PwmOutput, RTC,
                 HSPI_MOSI),  // 13-D13
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 AnalogIn, CapacitiveTouchIn, PwmOutput, RTC,
                 HSPI_CLK),  // 14-D14
    PinAbilities(StrappingPin, DInputPullDown, DInputPullUp, DOutput,
                 InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC,
                 HSPI_CS),  // 15-D15
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput, U2_RXD,
                 PwmOutput),  // 16-RX2
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput, U2_TXD,
                 PwmOutput),  // 17-TX2
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 PwmOutput, VSPI_CLK),  // 18-D18
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 PwmOutput, VSPI_MOSO),  // 19-D19
    PinAbilities(),                      // 20-XX
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput, SclPin,
                 PwmOutput),  // 21-D21
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput, SdaPin,
                 PwmOutput),  // 22-D22
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 PwmOutput, VSPI_MOSI),  // 23-D23
    PinAbilities(),                      // 24-XX
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 AnalogIn, DacOutput, PwmOutput, RTC),  // 25-D25
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 AnalogIn, DacOutput, PwmOutput, RTC),  // 26-D26
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 AnalogIn, CapacitiveTouchIn, PwmOutput, RTC),  // 27-D27
    PinAbilities(),                                             // 28-XX
    PinAbilities(),                                             // 29-XX
    PinAbilities(),                                             // 30-XX
    PinAbilities(),                                             // 31-XX
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 AnalogIn, CapacitiveTouchIn, PwmOutput, RTC),  // 32-D32
    PinAbilities(DInputPullDown, DInputPullUp, DOutput, InturruptInput,
                 AnalogIn, CapacitiveTouchIn, PwmOutput, RTC),    // 33-D33
    PinAbilities(DInputFloating, InturruptInput, AnalogIn, RTC),  // 34-D34
    PinAbilities(DInputFloating, InturruptInput, AnalogIn, RTC),  // 35-D35
    PinAbilities(DInputFloating, InturruptInput, AnalogIn, RTC),  // 36-VP
    PinAbilities(),                                               // 37-XX
    PinAbilities(),                                               // 38-XX
    PinAbilities(DInputFloating, InturruptInput, AnalogIn, RTC),  // 39-VN
};

void run_hal_initialization() {
  for (int i = 0; i < NUM_PINS; i++) {
    pin_settings[i].varient = None;
  }
}

void hal_disable_output() {}

void hal_enable_output() {}

void hal_force_disable() {}

void hal_brownout_output_start() {}

void hal_brownout_output_stop() {}

PinHandle create_pin_handle(uint8_t pin, PinSettingVarient setting,
                            PinSettingVarients data = {}) {
  while (pin_settings_lock.exchange(true, std::memory_order_acquire)) {
  }
  assert(pin < NUM_PINS);
  assert(pin_settings[pin].varient == None);
  assert(pin_abilities[pin].abilities[setting]);
  pin_settings[pin].varient = setting;
  pin_settings[pin].varients = data;
  pin_settings_lock.store(false, std::memory_order_release);
  return PinHandle(pin, create_pin_handle_creation());
}

void delete_pin_handle(PinHandle handle) {
  while (pin_settings_lock.exchange(true, std::memory_order_acquire)) {
  }
  assert(pin_settings[handle.physicalPinNumber()].varient != None);
  pin_settings[handle.physicalPinNumber()].varient = None;
  pin_settings_lock.store(false, std::memory_order_release);
}

//---------------- analog read pin
PinHandle create_analog_input(uint8_t pin) {
  auto handle = create_pin_handle(pin, AnalogIn);
  pinMode(pin, INPUT);
  return handle;
}

uint16_t analog_input_read(PinHandle& handle) {
  assert(pin_settings[handle.physicalPinNumber()].varient == AnalogIn);
  return analogRead(handle.physicalPinNumber());
}

void delete_analog_input(PinHandle handle) {
  assert(pin_settings[handle.physicalPinNumber()].varient == AnalogIn);
  delete_pin_handle(handle);
}

//---------------- digital read pin
PinHandle create_digital_input(uint8_t pin, InputPinMode pullup_mode) {
  PinSettingVarient mode;
  switch (pullup_mode) {
    case InputPinMode::Floating:
      mode = DInputFloating;
      break;
    case InputPinMode::PullDown:
      mode = DInputPullDown;
      break;
    case InputPinMode::PullUp:
      mode = DInputPullUp;
      break;
    default:
      assert(false);
      break;
  }
  auto handle = create_pin_handle(pin, mode);
  switch (pullup_mode) {
    case InputPinMode::Floating:
      pinMode(pin, INPUT);
      break;
    case InputPinMode::PullDown:
      pinMode(pin, INPUT_PULLDOWN);
      break;
    case InputPinMode::PullUp:
      pinMode(pin, INPUT_PULLUP);
      break;
  }
  return handle;
}

bool digital_input_read(PinHandle& handle) {
  auto mode = pin_settings[handle.physicalPinNumber()].varient;
  assert(mode == DInputFloating || mode == DInputPullDown ||
         mode == DInputPullUp);
  return digitalRead(handle.physicalPinNumber());
}

void delete_digital_input(PinHandle handle) {
  auto mode = pin_settings[handle.physicalPinNumber()].varient;
  assert(mode == DInputFloating || mode == DInputPullDown ||
         mode == DInputPullUp);
  delete_pin_handle(handle);
}

//---------------- capactive read pin
PinHandle create_capacitive_input(uint8_t pin) {
  auto handle = create_pin_handle(pin, CapacitiveTouchIn);
  pinMode(pin, INPUT);
  return handle;
}

uint16_t capacitive_input_read(PinHandle& handle) {
  assert(pin_settings[handle.physicalPinNumber()].varient == CapacitiveTouchIn);
  return touchRead(handle.physicalPinNumber());
}

void delete_capacitive_input(PinHandle handle) {
  assert(pin_settings[handle.physicalPinNumber()].varient == CapacitiveTouchIn);
  delete_pin_handle(handle);
}

//---------------- inturrupt pin
PinHandle create_inturrupt_pin(uint8_t pin);
bool inturrupt_pin_read(PinHandle& handle);
void delete_inturrupt_pin(PinHandle handle);

//---------------- digital output pin
PinHandle create_digital_output(uint8_t pin, bool disabled_value,
                                bool should_disable_output = true) {
  auto handle = create_pin_handle(pin, DOutput,
                                  PinSettingVarients{.DOutput{
                                      .current_val = disabled_value,
                                      .disable_val = disabled_value,
                                      .should_stop_output = disabled_value,
                                  }});
  pinMode(pin, OUTPUT);
  digitalWrite(handle.physicalPinNumber(), disabled_value);
  return handle;
}

bool digital_output_read(PinHandle& handle) {
  assert(pin_settings[handle.physicalPinNumber()].varient == DOutput);
  return digitalRead(handle.physicalPinNumber());
}

void digital_output_write(PinHandle& handle, bool value) {
  assert(pin_settings[handle.physicalPinNumber()].varient == DOutput);
  digitalWrite(handle.physicalPinNumber(), value);
}

void delete_digital_output(PinHandle handle) {
  assert(pin_settings[handle.physicalPinNumber()].varient == DOutput);
  digitalWrite(handle.physicalPinNumber(), false);
  pinMode(handle.physicalPinNumber(), INPUT);
  delete_pin_handle(handle);
}