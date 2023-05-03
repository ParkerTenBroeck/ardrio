#include "data_types.h"

//-------------------------------------------------------

DSToRobotUDP DSToRobotUDP::from_big_endian_buffer(uint8_t* buf, size_t len, size_t capacity){
  if (capacity < 6){
    return {};
  }
  DSToRobotUDP self = {len, capacity, (DSToRobotUDPInner*)buf};

  // swap the bytes 
  self.data->sequence = self.data->sequence >> 8 | self.data->sequence << 8;

  //TODO converte the tags
  return self;
}

size_t DSToRobotUDP::tags_len(){
  return this->size - 6;
}

//-------------------------------------------------------

bool Buttons::get(size_t button){
  if (button >= 32){
    return false;
  }else{
    return ((*(uint32_t*)this) >> button) & 1;
  }
}

//-------------------------------------------------------

float RobotVoltage::toFloat(){
  return (float)this->itg + ((float) this->dec) / 256.0;
}

RobotVoltage::RobotVoltage(float voltage){
  this->itg = (uint8_t)voltage;
  this->dec =  (uint8_t)((voltage-(uint8_t)voltage)*256.0);
}

RobotVoltage::RobotVoltage(uint8_t itg, uint8_t dec){
  this->itg = itg;
  this->dec = dec;
}

//-------------------------------------------------------

RobotToDSUDP RobotToDSUDP::from_big_endian_buffer(uint8_t *buf, size_t len, size_t capacity)
{
  if (capacity < 8)
  {
    return {};
  }
  RobotToDSUDP self = {len, capacity, (RobotToDSUDPInner *)buf};

  // big endian to little endian
  self.data->sequence = self.data->sequence >> 8 | self.data->sequence << 8;

  // TODO converte the tags
  return self;
}

RobotToDSUDP RobotToDSUDP::from_little_endian_buffer(uint8_t *buf, size_t capacity)
{
  if (capacity < 8)
  {
    return {};
  }
  RobotToDSUDP self = {8, capacity, (RobotToDSUDPInner *)buf};
  // TODO make sure the tags idk like exist
  return self;
}

size_t RobotToDSUDP::tags_len()
{
  return this->size - 8;
}

// the size of the returned buffer is the same as this.size
uint8_t *RobotToDSUDP::to_big_endian_buffer()
{
  this->data->sequence = this->data->sequence >> 8 | this->data->sequence << 8;

  // TODO converte the tags

  return (uint8_t *)this->data;
}

//-------------------------------------------------------

const char *stationAsString(AllianceStation station)
{
  switch ((AllianceStation)station)
  {
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