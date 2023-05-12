#pragma once

#include "data_types.h"
#include "driverstation.h"
#include "esp32-hal.h"
#include "hal.h"

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
void start_robot(bool wait_for_ds = true, RunMode mode = RunMode::Blocking,
                 Driverstation* driverstation = &DRIVERSTATION) {
  run_hal_initialization();
  hal_disable_output();

  driverstation->disable_hook = [](Driverstation* ds) {
    hal_disable_output();
    ds->print("Disabled");
  };
  driverstation->teleop_hook = [](Driverstation* ds) {
    hal_enable_output();
    ds->print("Teleop");
  };
  driverstation->test_hook = [](Driverstation* ds) {
    hal_enable_output();
    ds->print("Test");
  };
  driverstation->auton_hook = [](Driverstation* ds) {
    hal_enable_output();
    ds->print("Auton");
  };
  driverstation->brownout_start_hook = [](Driverstation* ds) {
    hal_brownout_output_start();
    ds->print("Brownout Start");
  };
  driverstation->brownout_end_hook = [](Driverstation* ds) {
    hal_brownout_output_stop();
    ds->print("Brownout End");
  };
  driverstation->estop_hook = [](Driverstation* ds) {
    hal_force_disable();
    ds->print("Estop");
  };
  driverstation->restart_code_hook = [](Driverstation* ds) {
    ds->print("Restart Code");
    delay(100);
    ESP.restart();
  };
  driverstation->restart_hook = [](Driverstation* ds) {
    ds->print("Restart");
    delay(100);
    ESP.restart();
  };

  void (*run_robot)(void*);

  if (wait_for_ds) {
    run_robot = [](void* ds_v) {
      auto ds = (Driverstation*)ds_v;
      ds->begin();
      while (!ds->connected()) delay(1);
      T robot = T();
      robot.start_competition();
      robot.end_competition();
      vTaskDelete(NULL);
      delay(2147483647);
    };
  } else {
    run_robot = [](void* ds_v) {
      auto ds = (Driverstation*)ds_v;
      ds->begin();
      T robot = T();
      robot.start_competition();
      robot.end_competition();
      vTaskDelete(NULL);
      delay(2147483647);
    };
  }
  if (mode == RunMode::Blocking) {
    run_robot((void*)driverstation);
  } else if (mode == RunMode::NewTask || mode == RunMode::NewTaskEndCurrent) {
    xTaskCreateUniversal(run_robot, "Robot Code",
                         uxTaskGetStackHighWaterMark(NULL),
                         (void*)driverstation, 1, NULL, ARDUINO_RUNNING_CORE);
    if (mode == RunMode::NewTaskEndCurrent) {
      vTaskDelete(NULL);
    }
  } else {
    // idk how to panic in c++
  }
}
