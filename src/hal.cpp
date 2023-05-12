#include "hal.h"

#include <atomic>
#include "esp32-hal-ledc.h"

//------------------ pin handle

uint8_t PinHandle::physicalPinNumber() { return this->pin; }

PinHandle::PinHandle() { this->pin = 255; }

PinHandle::PinHandle(uint8_t pin, HandleCreation* ctx) { this->pin = pin; }

//------------------ pin handle

HandleCreation* create_pin_handle_creation() { return NULL; }

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
  bool safety_critical_val : 1;
  bool high_power_val : 1;
  // whether or not to switch to disable_val when output is disabled
  bool safety_critical : 1;
  // whether or not to switch to disable_val when a brown out occurs
  bool high_power_consumption : 1;
};

struct PwnTimerSetting {
  uint8_t resolution;
  bool exists : 1;
  bool safety_critical : 1;
  bool high_power_consuption : 1;
  uint32_t frequency;
  uint32_t phase;
  uint32_t disabled_frequency;
  uint32_t disabled_phase;
  uint32_t brownout_frequency;
  uint32_t brownout_phase;
};

struct PwmOutputSetting {
  uint8_t channel : 4;  // 16 channels max

  // whether or not to switch to disable_val when output is disabled
  bool safety_critical : 1;
  // if this device is safety critical and saftey critical devices are disabled
  // this is the value the pin will be set to
  bool safety_critical_disabled_val : 1;

  // whether or not to switch to disable_val when a brown out occurs
  bool high_power_consumption : 1;
  // if this device is high power and high power devices are disable
  // what is the value of the pin()
  bool high_power_disabled_val : 1;
};

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
#ifdef SOC_LEDC_SUPPORT_HS_MODE
#define NUM_PWM_CHANNELS           (SOC_LEDC_CHANNEL_NUM<<1)
#else
#ifndef SOC_LEDC_CHANNEL_NUM
#define SOC_LEDC_CHANNEL_NUM 16
#endif
#define NUM_PWM_CHANNELS           (SOC_LEDC_CHANNEL_NUM)
#endif

PwnTimerSetting pwm_config[NUM_PWM_CHANNELS] = {};

std::atomic<bool> pin_settings_lock(false);

void lock_pin_settings_critical() {
  while (pin_settings_lock.exchange(true, std::memory_order_acquire)) {
  }
  noInterrupts();
}

void unlock_pin_settings_critical() {
  pin_settings_lock.store(false, std::memory_order_release);
  interrupts();
}


std::atomic<bool> disable_high_power(false);
std::atomic<bool> disable_safety_critical(false);
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
  lock_pin_settings_critical();
  for (int i = 0; i < NUM_PINS; i++) {
    pin_settings[i].varient = None;
  }
  unlock_pin_settings_critical();
}

void hal_disable_output() {
  lock_pin_settings_critical();
  disable_safety_critical.store(true, std::memory_order_relaxed);
  for(int i = 0; i < NUM_PINS; i ++){
    switch(pin_settings[i].varient){
      case DOutput:{
        if(pin_settings[i].varients.DOutput.safety_critical){
          digitalWrite(i, pin_settings[i].varients.DOutput.safety_critical_val);
        }
      }
        break;
      case PwmOutput:{
        if(pin_settings[i].varients.PwmOutput.safety_critical){
          ledcDetachPin(i);
          digitalWrite(i, pin_settings[i].varients.PwmOutput.safety_critical_disabled_val);
        }
      }
        break;

      default:
        break;
    }
  }
  unlock_pin_settings_critical();
}

void hal_enable_output() {
  lock_pin_settings_critical();
  disable_safety_critical.store(false, std::memory_order_relaxed);
  for(int i = 0; i < NUM_PINS; i ++){
    switch(pin_settings[i].varient){
      case DOutput:{
        if(pin_settings[i].varients.DOutput.safety_critical){
          digitalWrite(i, pin_settings[i].varients.DOutput.current_val);
        }
      }
        break;
      case PwmOutput:{
        if(pin_settings[i].varients.PwmOutput.safety_critical){
          ledcAttachPin(i, pin_settings[i].varients.PwmOutput.channel);
        }
      }
        break;

      default:
        break;
    }
  }
  unlock_pin_settings_critical();
}

void hal_force_disable() {
  hal_disable_output();
}

void hal_brownout_output_start() {
  lock_pin_settings_critical();
  disable_high_power.store(true, std::memory_order_relaxed);
  unlock_pin_settings_critical();
}

void hal_brownout_output_stop() {
  lock_pin_settings_critical();
  disable_high_power.store(false, std::memory_order_relaxed);
  unlock_pin_settings_critical();
}


PinHandle create_pin_handle(uint8_t pin, PinSettingVarient setting,
                            PinSettingVarients data = {}, bool lock = true) {
  if(lock)
    lock_pin_settings_critical();
  assert(pin < NUM_PINS);
  assert(pin_settings[pin].varient == None);
  assert(pin_abilities[pin].abilities[setting]);
  pin_settings[pin].varient = setting;
  pin_settings[pin].varients = data;
  if(lock)
    unlock_pin_settings_critical();
  return PinHandle(pin, create_pin_handle_creation());
}

void delete_pin_handle(PinHandle handle, bool lock = true) {
  if(lock)
    lock_pin_settings_critical();
  assert(pin_settings[handle.physicalPinNumber()].varient != None);
  pin_settings[handle.physicalPinNumber()].varient = None;
  if(lock)
    unlock_pin_settings_critical();
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
PinHandle create_digital_output(uint8_t pin, bool disabled_value, bool brownout_val,
                                bool should_disable_output, bool should_brown_out) {
  lock_pin_settings_critical();
  auto handle = create_pin_handle(pin, DOutput,
                                  PinSettingVarients{.DOutput{
                                      .current_val = disabled_value,
                                      .safety_critical_val = disabled_value,
                                      .high_power_val = brownout_val,
                                      .safety_critical = should_disable_output,
                                      .high_power_consumption = should_brown_out,
                                  }}, false);

  pinMode(pin, OUTPUT);
  digitalWrite(handle.physicalPinNumber(), disabled_value);
  unlock_pin_settings_critical();
  return handle;
}

bool digital_output_read(PinHandle& handle) {
  assert(pin_settings[handle.physicalPinNumber()].varient == DOutput);
  return digitalRead(handle.physicalPinNumber());
}

void digital_output_write(PinHandle& handle, bool value) {
  lock_pin_settings_critical();
  assert(pin_settings[handle.physicalPinNumber()].varient == DOutput);
  auto setting = &pin_settings[handle.physicalPinNumber()].varients.DOutput;
  setting->current_val = value;
  if (!((disable_high_power.load(std::memory_order_relaxed) &&
         setting->high_power_consumption) ||
        (disable_safety_critical.load(std::memory_order_relaxed) &&
         setting->safety_critical))) {
    digitalWrite(handle.physicalPinNumber(), value);
  }
  unlock_pin_settings_critical();
}

void delete_digital_output(PinHandle handle) {
  lock_pin_settings_critical();
  assert(pin_settings[handle.physicalPinNumber()].varient == DOutput);
  digitalWrite(handle.physicalPinNumber(), false);
  pinMode(handle.physicalPinNumber(), INPUT);
  delete_pin_handle(handle, false);
  unlock_pin_settings_critical();
}


//---------------- pwm generators

uint8_t PwmHandle::channel(){
  return this->_channel;
}

PwmHandle::PwmHandle(){}

PwmHandle::PwmHandle(uint8_t channel, HandleCreation* ctx){
  this->_channel = channel;
}

PwmHandle create_pwm_generator(uint8_t resolution, uint32_t frequecy, uint32_t phase){
  for(uint8_t i = 0; i < NUM_PWM_CHANNELS; i ++){
    if(!pwm_config[i].exists){
      assert(ledcSetup(i, frequecy, resolution));
      ledcWrite(i, phase);
      pwm_config[i].exists = true;
      pwm_config[i].frequency = frequecy;
      pwm_config[i].phase = phase;
      pwm_config[i].resolution = resolution;
      return PwmHandle(i, create_pin_handle_creation());
    }
  }
  assert(false);
}
void update_pwm_duty(PwmHandle& handle, uint32_t phase){
  lock_pin_settings_critical();
  assert(pwm_config[handle.channel()].exists);
  ledcWrite(handle.channel(), phase);
  unlock_pin_settings_critical();
}
void update_pwm_frequency(PwmHandle& handle, uint32_t frequency){
  lock_pin_settings_critical();
  assert(pwm_config[handle.channel()].exists);
  ledcWriteTone(handle.channel(), frequency);
  unlock_pin_settings_critical();
}
void update_pwm_resolution(PwmHandle& handle, uint8_t resolution){
  lock_pin_settings_critical();
  assert(pwm_config[handle.channel()].exists);
  ledcChangeFrequency(handle.channel(), ledcReadFreq(handle.channel()), resolution);
  unlock_pin_settings_critical();
}

void delete_pwm_generator(PwmHandle handle){
  lock_pin_settings_critical();
  assert(pwm_config[handle.channel()].exists);
  ledcWrite(handle.channel(), 0);
  pwm_config[handle.channel()].exists = false;
  //TODO. possibly use ledc_stop()
  unlock_pin_settings_critical();
}

//---------------- pwm output

PinHandle create_pwn_output(uint8_t pin, PwmHandle& pwm_handle, bool disabled_value, bool brownout_val,
                                bool should_disable_output , bool should_brown_out){
  lock_pin_settings_critical();

  auto handle = create_pin_handle(pin, PwmOutput,
                                  PinSettingVarients{.PwmOutput{
                                    .channel = pwm_handle.channel(),
                                    .safety_critical = should_disable_output,
                                    .safety_critical_disabled_val = disabled_value,
                                    .high_power_consumption = should_brown_out,
                                    .high_power_disabled_val = brownout_val
                                  }}, false);

  // pinMode(pin, OUTPUT);
  ledcAttachPin(pin, pwm_handle.channel());
  unlock_pin_settings_critical();
  return handle;
}

void delete_pwm_output(PinHandle handle){
  lock_pin_settings_critical();
  ledcDetachPin(handle.physicalPinNumber());
  pinMode(handle.physicalPinNumber(), INPUT);
  delete_pin_handle(handle, false);
  unlock_pin_settings_critical();
}