#include "robot.h"
#include "esp_task.h"
#include "esp_task_wdt.h"


void TimedRobot::start_competition() {
  esp_task_wdt_init(1, true);
  esp_task_wdt_add(NULL);
  TickType_t last_wake = xTaskGetTickCount();
  BaseType_t was_delayed;
  while (1) {
    esp_task_wdt_reset();
    was_delayed =
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(this->delta_time_ms));
    this->run_tick();
    if (!was_delayed) {
      // print warning for task taking too long
    }
  }
}

void TimedRobot::run_tick() {
  // auto start = micros();
  auto mode = DRIVERSTATION.get_control_code();

  if (this->first || mode.enabled != this->current_mode.enabled ||
      mode.mode != this->current_mode.mode) {
    if (!this->first) {
      if (!this->current_mode.enabled) {
        this->disabled_exit();
      } else if (this->current_mode.mode == 0) {
        this->teleop_exit();
      } else if (this->current_mode.mode == 1) {
        this->test_exit();
      } else if (this->current_mode.mode == 2) {
        this->auton_exit();
      }
    }

    this->current_mode = mode;

    if (!mode.enabled) {
      this->disabled_init();
    } else if (mode.mode == 0) {
      this->teleop_init();
    } else if (mode.mode == 1) {
      this->test_init();
    } else if (mode.mode == 2) {
      this->auton_init();
    }
    this->first = false;
  }
  this->current_mode = mode;

  DRIVERSTATION.observe_robot_code();
  this->robot_periodic();
  if (!mode.enabled) {
    DRIVERSTATION.observe_disabled();
    this->disabled_periodic();
  } else if (mode.mode == 0) {
    DRIVERSTATION.observe_teleop();
    this->teleop_periodic();
  } else if (mode.mode == 1) {
    DRIVERSTATION.observe_test();
    this->test_periodic();
  } else if (mode.mode == 2) {
    DRIVERSTATION.observe_auton();
    this->auton_periodic();
  }
  // DRIVERSTATION.printf("Time: %u", micros() - start);
}

void TimedRobot::end_competition() {}
