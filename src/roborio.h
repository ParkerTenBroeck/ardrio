#pragma once

#include<Arduino.h>

typedef enum class AllianceStation: uint8_t{
  Red1 = 0,
  Red2 = 1,
  Red3 = 2,
  Blue1 = 3,
  Blue2 = 4,
  Blue3 = 5,
} AllianceStation;

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

public:

  uint size;
  uint capacity;
  DSToRobotUDPInner* data;

  static DSToRobotUDP from_big_endian_buffer(uint8_t* buf, uint len, uint capacity){
    if (capacity < 6){
      return {};
    }
    DSToRobotUDP self = {len, capacity, (DSToRobotUDPInner*)buf};

    // swap the bytes 
    self.data->sequence = self.data->sequence >> 8 | self.data->sequence << 8;

    //TODO converte the tags
    return self;
  }

  uint tags_len(){
    return this->size - 6;
  }
};


struct RobotVoltage{
  uint8_t itg;
  uint8_t dec;

  float toFloat(){
    return (float)this->itg + ((float) this->dec) / 256.0;
  }

  static RobotVoltage fromFloat(float voltage){
    return {(uint8_t)voltage, (uint8_t)((voltage-(uint8_t)voltage)*256.0)};
  }
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
  uint size;
  uint capacity;
  RobotToDSUDPInner* data;

  static RobotToDSUDP from_big_endian_buffer(uint8_t* buf, uint len, uint capacity){
    if (capacity < 8){
      return {};
    }
    RobotToDSUDP self = {len, capacity, (RobotToDSUDPInner*)buf};

    // big endian to little endian
    self.data->sequence = self.data->sequence >> 8 | self.data->sequence << 8;

    //TODO converte the tags
    return self;
  }

  static RobotToDSUDP from_little_endian_buffer(uint8_t* buf, uint capacity){
    if (capacity < 8){
      return {};
    }
    RobotToDSUDP self = {8, capacity, (RobotToDSUDPInner*)buf};
        //TODO make sure the tags idk like exist
    return self;
  }

  uint tags_len(){
    return this->size - 8;
  }

  // the size of the returned buffer is the same as this.size
  uint8_t* to_big_endian_buffer(){
      this->data->sequence = this->data->sequence >> 8 | this->data->sequence << 8;

      //TODO converte the tags 

      return (uint8_t*)this->data;
  }
};

const char* stationAsString(AllianceStation station){
  switch ((AllianceStation)station){
    case AllianceStation::Red1:
      return "Red1";
    case AllianceStation::Red2:
      return "Red2";  
    case AllianceStation::Red3:
      return "Red3";
    case AllianceStation::Blue1: 
      return "Blue1";
    case AllianceStation::Blue2:
      return "Blue2";
    case AllianceStation::Blue3:
      return "Blue3";
    default:
      return "Bruh?";
  }
}





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

  bool get(uint button){
    if (button >= 32){
      return false;
    }else{
      return ((*(uint32_t*)this) >> button) & 1;
    }
  }
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
