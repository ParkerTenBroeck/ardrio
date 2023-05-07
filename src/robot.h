#pragma once

#include "data_types.h"
#include "driverstation.h"
#include "hal.h"
#include "esp32-hal.h"

enum class RunMode {
  Blocking,
  NewTask,
  NewTaskEndCurrent,
};

class RobotBase {
 public:
  virtual void start_competition();
  virtual void end_competition();
};

class TimedRobot : public RobotBase {
 private:
  ControlCode current_mode;
  bool first = true;

 public:
  uint32_t delta_time_ms = 20;

  void start_competition() override;
  void end_competition() override;

  virtual void disabled_init() {}
  virtual void teleop_init() {}
  virtual void test_init() {}
  virtual void auton_init() {}

  virtual void robot_periodic() {}
  virtual void disabled_periodic() {}
  virtual void teleop_periodic() {}
  virtual void test_periodic() {}
  virtual void auton_periodic() {}

  virtual void disabled_exit() {}
  virtual void teleop_exit() {}
  virtual void test_exit() {}
  virtual void auton_exit() {}

 private:
  void run_tick();
};

template <typename T>
void start_robot(bool wait_for_ds = true, RunMode mode = RunMode::Blocking) {

  run_hal_initialization();

  DRIVERSTATION.disable_hook = [](Driverstation* ds){
    ds->print("Disabled");
  };
  DRIVERSTATION.teleop_hook = [](Driverstation* ds){
    ds->print("Teleop");
  };
  DRIVERSTATION.test_hook = [](Driverstation* ds){
    ds->print("Test");
  };
  DRIVERSTATION.auton_hook = [](Driverstation* ds){
    ds->print("Auton");
  };
  DRIVERSTATION.brownout_start_hook = [](Driverstation* ds){
    ds->print("Brownout Start");
  };
  DRIVERSTATION.brownout_end_hook = [](Driverstation* ds){
    ds->print("Brownout End");
  };
  DRIVERSTATION.estop_hook = [](Driverstation* ds){
    ds->print("Estop");
  };
  DRIVERSTATION.restart_code_hook = [](Driverstation* ds){
    ds->print("Restart Code");
    delay(100);
    ESP.restart();
  };
  DRIVERSTATION.restart_hook = [](Driverstation* ds){
    ds->print("Restart");
    delay(100);
    ESP.restart();
  };


  auto run_robot = [] (void* param) {
    DRIVERSTATION.begin();
    while(!DRIVERSTATION.connected() && wait_for_ds)
      delay(1);
    T robot = T();
    robot.start_competition();
    robot.end_competition();
    vTaskDelete(NULL);
    delay(2147483647);
  };
  if (mode == RunMode::Blocking) {
    run_robot(NULL);
  } else if (mode == RunMode::NewTask || mode == RunMode::NewTaskEndCurrent) {
    xTaskCreateUniversal(run_robot, "Robot Code",
                         uxTaskGetStackHighWaterMark(NULL), NULL, 1, NULL,
                         ARDUINO_RUNNING_CORE);
    if (mode == RunMode::NewTaskEndCurrent) {
      vTaskDelete(NULL);
    }
  } else {
    // idk how to panic in c++
  }
}
