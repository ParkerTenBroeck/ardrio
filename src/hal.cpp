#include "hal.h"
#include "esp32-hal-ledc.h"

enum PinSettingVarient{
  StrappingPin,
  DInput,
  DInputPullUp,
  DInputPullDown,
  DOutput,
  PwmOutput,
  AnalogIn,
  DacOutput,
  CapacitiveTouchIn,
  InturruptInput,
  
  SCL,
  SDA,

  U2_TXD,
  U2_RXD,

  RTC,

  // don't use this
  NUM_PIN_SETTINGS,

  None,
};

struct DOutputSetting{
  bool high;  
};

struct PwmOutputSetting{

};

struct DacOutputSetting{
  uint16_t val;
};

union PinSettingVarients{
  DOutputSetting DOutput;
  DOutputSetting DOutputPullUp;
  DOutputSetting DOutputPullDown;
  PwmOutputSetting PwmOutput;
  DacOutputSetting DacOutput;
};

struct PinSetting{
  PinSettingVarient varient;
  PinSettingVarients varients;
};

struct PinAbilities{
  bool abilities[NUM_PIN_SETTINGS];

  template<typename... Args>
  PinAbilities(Args... args){
    for(int i = 0; i < NUM_PIN_SETTINGS; i ++){
      this->abilities[i] = false;
    }
    this->add_abilities(args...);
  }

  template<typename T, typename... Args>
  void add_abilities(T t, Args... args)
  {
      this->abilities[t] = true;
      add_abilities(args...);
  }

    template<typename T>
  void add_abilities(T t)
  {
      this->abilities[t] = true;
  }

  void add_abilities(){}
};

PinSetting pin_settings[40] = {
};

PinAbilities pin_abilities[40] = {
  PinAbilities(), // 00-D0
  PinAbilities(), // 01-TX0 Used for 
  PinAbilities(StrappingPin, DInput, DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC), // 02-D2
  PinAbilities(), // 03-RX0
  PinAbilities(DInput, DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC), // 04-D4
  PinAbilities(StrappingPin, DInput, DOutput, InturruptInput, PwmOutput), // 05-D5
  PinAbilities(), // 06-D6
  PinAbilities(), // 07-D7
  PinAbilities(), // 08-D8
  PinAbilities(), // 09-D9
  PinAbilities(), // 10-D10
  PinAbilities(), // 11-D11
  PinAbilities(StrappingPin, DInput, DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC), // 12-D12
  PinAbilities(DInput, DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC), // 13-D13
  PinAbilities(DInput, DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC), // 14-D14
  PinAbilities(StrappingPin, DInput, DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC), // 15-D15
  PinAbilities(DInput, DOutput, InturruptInput, U2_RXD, PwmOutput), // 16-RX2
  PinAbilities(DInput, DOutput, InturruptInput, U2_TXD, PwmOutput), // 17-TX2
  PinAbilities(DInput, DOutput, InturruptInput, PwmOutput), // 18-D18
  PinAbilities(DInput, DOutput, InturruptInput, PwmOutput), // 19-D19
  PinAbilities(), // 20-XX
  PinAbilities(DInput, DOutput, InturruptInput, SCL, PwmOutput), // 21-D21
  PinAbilities(DInput, DOutput, InturruptInput, SDA, PwmOutput), // 22-D22
  PinAbilities(DInput, DOutput, InturruptInput, PwmOutput,), // 23-D23
  PinAbilities(), // 24-XX
  PinAbilities(DInput, DOutput, InturruptInput, AnalogIn, DacOutput, PwmOutput, RTC), // 25-D25
  PinAbilities(DInput, DOutput, InturruptInput, AnalogIn, DacOutput, PwmOutput, RTC), // 26-D26
  PinAbilities(DInput, DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC), // 27-D27
  PinAbilities(), // 28-XX
  PinAbilities(), // 29-XX
  PinAbilities(), // 30-XX
  PinAbilities(), // 31-XX
  PinAbilities(DInput, DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC), // 32-D32
  PinAbilities(DInput, DOutput, InturruptInput, AnalogIn, CapacitiveTouchIn, PwmOutput, RTC), // 33-D33
  PinAbilities(DInput, InturruptInput, AnalogIn, RTC), // 34-D34
  PinAbilities(DInput, InturruptInput, AnalogIn, RTC), // 35-D35
  PinAbilities(DInput, InturruptInput, AnalogIn, RTC), // 36-VP
  PinAbilities(), // 37-XX
  PinAbilities(), // 38-XX
  PinAbilities(DInput, InturruptInput, AnalogIn, RTC), // 39-VN
};

void run_hal_initialization() {

}

void hal_disable_output(){

}

void hal_enable_output(){

}

void hal_force_disable(){

}

void hal_brownout_output_start(){

}

void hal_brownout_output_stop(){

}