#pragma once
#include <Arduino.h>

enum class AllianceStation: uint8_t{
  Red1 = 0,
  Red2 = 1,
  Red3 = 2,
  Blue1 = 3,
  Blue2 = 4,
  Blue3 = 5,
};

struct ControlCode{
  uint8_t mode: 2;
  uint8_t enabled: 1;
  uint8_t fms_attached: 1;
  uint8_t brownout_protection: 1;
  uint8_t _reserved: 1;
  uint8_t stop_connection: 1;
  uint8_t estop: 1;
};

struct RobotRequestCode{
  uint8_t request_tcp_lib_info: 1;
  uint8_t _reserved0: 1;
  uint8_t restart_roborio_code: 1;
  uint8_t restart_roborio: 1;
  uint8_t request_normal: 1;
  uint8_t _reserved1: 3;
};

struct DSToRobotUDPInner{
  uint16_t sequence;
  uint8_t com_version;
  ControlCode control_code;
  RobotRequestCode request_code;
  AllianceStation alliance_station;
  uint8_t tags[];
};

struct DSToRobotUDP{
  size_t size;
  size_t capacity;
  DSToRobotUDPInner* data;

  static DSToRobotUDP from_big_endian_buffer(uint8_t* buf, size_t len, size_t capacity);
  size_t tags_len();
};


struct RobotVoltage{
  uint8_t itg;
  uint8_t dec;

  RobotVoltage() noexcept = default;
  RobotVoltage(uint8_t itg, uint8_t dec);
  RobotVoltage(float voltage);

  float toFloat();
};

struct RobotStatusCode{
  uint8_t dissabled: 1;
  uint8_t teleop_code: 1;
  uint8_t auton_code: 1;
  uint8_t test_code: 1;
  uint8_t is_roborio: 1;
  uint8_t has_robot_code: 1;
  uint8_t _7: 1;
  uint8_t _8: 1;
};

struct DriverstationRequestCode{
  uint8_t request_time: 1;
  uint8_t request_disable: 1;
  uint8_t reserved: 6;
};

struct RobotToDSUDPInner{
  uint16_t sequence;
  uint8_t com_version;
  ControlCode control_code;
  RobotStatusCode status;
  RobotVoltage voltage;
  DriverstationRequestCode request;
  uint8_t tags[];
};

struct RobotToDSUDP{
  size_t size;
  size_t capacity;
  RobotToDSUDPInner* data;

  static RobotToDSUDP from_big_endian_buffer(uint8_t* buf, size_t len, size_t capacity);

  static RobotToDSUDP from_little_endian_buffer(uint8_t* buf, size_t capacity);

  size_t tags_len();

  // the size of the returned buffer is the same as this.size
  uint8_t* to_big_endian_buffer();
};

const char* stationAsString(AllianceStation station);





//-----------------------------------------------------


union Buttons{
  uint32_t bits;
  struct{
    uint32_t button0: 1;
    uint32_t button1: 1;
    uint32_t button2: 1;
    uint32_t button3: 1;
    uint32_t button4: 1;
    uint32_t button5: 1;
    uint32_t button6: 1;
    uint32_t button7: 1;
    uint32_t button8: 1;
    uint32_t button9: 1;
    uint32_t button10: 1;
    uint32_t button11: 1;
    uint32_t button12: 1;
    uint32_t button13: 1;
    uint32_t button14: 1;
    uint32_t button15: 1;
    uint32_t button16: 1;
    uint32_t button17: 1;
    uint32_t button18: 1;
    uint32_t button19: 1;
    uint32_t button20: 1;
    uint32_t button21: 1;
    uint32_t button22: 1;
    uint32_t button23: 1;
    uint32_t button24: 1;
    uint32_t button25: 1;
    uint32_t button26: 1;
    uint32_t button27: 1;
    uint32_t button28: 1;
    uint32_t button29: 1;
    uint32_t button30: 1;
    uint32_t button31: 1;
  };

  bool get(size_t button);
};

struct ControllerMetadata{
  uint16_t button_len: 5;
  uint16_t axis_len: 4;
  uint16_t pov_len: 2;
  uint16_t exists: 1;
};

struct Controller{
  ControllerMetadata metadata;
  Buttons buttons;
  int8_t axis[14];
  uint16_t povs[2];
};
