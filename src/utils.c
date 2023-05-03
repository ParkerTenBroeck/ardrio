// #include "roborio.h"


// typedef enum class AllianceStation: uint8_t{
//   Red1 = 0,
//   Red2 = 1,
//   Red3 = 2,
//   Blue1 = 3,
//   Blue2 = 4,
//   Blue3 = 5,
// } AllianceStation;

// struct ControlCode{
//   uint8_t mode: 2;
//   uint8_t enabled: 1;
//   uint8_t fms_attached: 1;
//   uint8_t brownout_protection: 1;
//   uint8_t _reserved: 1;
//   uint8_t stop_connection: 1;
//   uint8_t estop: 1;
// };

// struct RobotRequestCode{
//   uint8_t request_tcp_lib_info: 1;
//   uint8_t _reserved0: 1;
//   uint8_t restart_roborio_code: 1;
//   uint8_t restart_roborio: 1;
//   uint8_t request_normal: 1;
//   uint8_t _reserved1: 3;
// };

// struct DSToRobotUDPInner{
//   uint16_t sequence;
//   uint8_t com_version;
//   ControlCode control_code;
//   RobotRequestCode request_code;
//   AllianceStation alliance_station;
//   uint8_t tags[];
// };

// struct DSToRobotUDP{

// public:

//   uint size;
//   uint capacity;
//   DSToRobotUDPInner* data;

//   static DSToRobotUDP from_big_endian_buffer(uint8_t* buf, uint len, uint capacity){
//     if (capacity < 6){
//       return {};
//     }
//     DSToRobotUDP self = {len, capacity, (DSToRobotUDPInner*)buf};

//     // swap the bytes 
//     self.data->sequence = self.data->sequence >> 8 | self.data->sequence << 8;

//     //TODO converte the tags
//     return self;
//   }

//   uint tags_len(){
//     return this->size - 6;
//   }
// };


// struct RobotVoltage{
//   uint8_t itg;
//   uint8_t dec;

//   float toFloat(){
//     return (float)this->itg + ((float) this->dec) / 256.0;
//   }

//   static RobotVoltage fromFloat(float voltage){
//     return {(uint8_t)voltage, (uint8_t)((voltage-(uint8_t)voltage)*256.0)};
//   }
// };



// static RobotToDSUDP RobotToDSUDP::from_big_endian_buffer(uint8_t* buf, uint len, uint capacity){
//   if (capacity < 8){
//     return {};
//   }
//   RobotToDSUDP self = {len, capacity, (RobotToDSUDPInner*)buf};

//   // big endian to little endian
//   self.data->sequence = self.data->sequence >> 8 | self.data->sequence << 8;

//   //TODO converte the tags
//   return self;
// }

// static RobotToDSUDP RobotToDSUDP::from_little_endian_buffer(uint8_t* buf, uint capacity){
//   if (capacity < 8){
//     return {};
//   }
//   RobotToDSUDP self = {8, capacity, (RobotToDSUDPInner*)buf};
//       //TODO make sure the tags idk like exist
//   return self;
// }

// uint RobotToDSUDP::tags_len(){
//   return this->size - 8;
// }

// // the size of the returned buffer is the same as this.size
// uint8_t* RobotToDSUDP::to_big_endian_buffer(){
//     this->data->sequence = this->data->sequence >> 8 | this->data->sequence << 8;

//     //TODO converte the tags 

//     return (uint8_t*)this->data;
// }



// //-----------------------------------------------------


// const char* stationAsString(AllianceStation station){
//   switch ((AllianceStation)station){
//     case AllianceStation::Red1:
//       return "Red1";
//     case AllianceStation::Red2:
//       return "Red2";  
//     case AllianceStation::Red3:
//       return "Red3";
//     case AllianceStation::Blue1: 
//       return "Blue1";
//     case AllianceStation::Blue2:
//       return "Blue2";
//     case AllianceStation::Blue3:
//       return "Blue3";
//     default:
//       return "Bruh?";
//   }
// }


// //-----------------------------------------------------

// bool Buttons::get(uint button){
//   if (button >= 32){
//     return false;
//   }else{
//     return ((*(uint32_t*)this) >> button) & 1;
//   }
// }


