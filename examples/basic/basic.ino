#include "ardrio.h"
#include "wifi_setup.h"

class MyRobot : public TimedRobot {
 public:
  void robot_periodic() override {
    auto val = analogRead(A0);
    DRIVERSTATION.observe_voltage(RobotVoltage{
        .itg = (uint8_t)(val >> 8),
        .dec = (uint8_t)(val),
    });
    if (DRIVERSTATION.get_controller(0).buttons.get(0)) {
      DRIVERSTATION.start_brownout();
    } else {
      DRIVERSTATION.end_brownout();
    }
  }
  void disabled_init() override { DRIVERSTATION.print("Disabled Start"); }
  void teleop_init() { DRIVERSTATION.print("Teleop Start"); }
  void test_init() override { DRIVERSTATION.print("Test Start"); }
  void auton_init() override { DRIVERSTATION.print("Auton Start"); }

  void disabled_exit() override { DRIVERSTATION.print("Disabled Exit"); }
  void teleop_exit() override { DRIVERSTATION.print("Teleop Exit"); }
  void test_exit() override { DRIVERSTATION.print("Test Exit"); }
  void auton_exit() override { DRIVERSTATION.print("Auton Exit"); }
};

void setup() {
  Serial.begin(9600);

  if (true) {
    configure_static_bridge(1114, IPAddress(255, 0, 0, 0));
    begin_bridge(1114, "TheFMS");
  } else {
    config_ap(1114);
    begin_ap(1114, "ESP32");
  }

  run_robot<MyRobot>();
}

void loop() { /*doesn't run*/ }