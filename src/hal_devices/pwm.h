#include "Arduino.h"
#include "hal.h"
#include <atomic>

enum class Channel{
  Channel0,
  Channel1,
  Channel2,
  Channel3,
  Channel4,
  Channel5,
  Channel6,
  Channel7,
  Channel8,
  Channel9,
  Channel10,
  Channel11,
  Channel12,
  Channel13,
  Channel14,
  Channel15,
};

struct Resolution{
  uint8_t bits : 4;
};

class PWM{
 private:
  static std::atomic<uint8_t> auto_channel;
 public:
  uint8_t channel;
  uint8_t resolution = 16;
  uint32_t frequency;
};
