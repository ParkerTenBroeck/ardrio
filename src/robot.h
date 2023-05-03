#pragma once

#include "esp32-hal.h"
#include "data_types.h"
#include "driverstation.h"

enum class RunMode{
  Blocking,
  NewTask,
  NewTaskEndCurrent,
};

class RobotBase{
public:
  virtual void start_competition();
  virtual void end_competition();
};

class TimedRobot: public RobotBase{
  private:
  ControlCode current_mode;
  bool first = true;
public:
  uint32_t delta_time_ms = 20;

  void start_competition() override;
  void end_competition() override;

  virtual void disabled_init(){}
  virtual void teleop_init(){}
  virtual void test_init(){}
  virtual void auton_init(){}

  virtual void robot_periodic(){}
  virtual void disabled_periodic(){}
  virtual void teleop_periodic(){}
  virtual void test_periodic(){}
  virtual void auton_periodic(){}

  virtual void disabled_exit(){}
  virtual void teleop_exit(){}
  virtual void test_exit(){}
  virtual void auton_exit(){}
private:
  void run_tick();
};

template <typename T>
void run_robot_inner(void* param) {
  DRIVERSTATION.begin();
  T robot = T();
  robot.start_competition();
  robot.end_competition();
  vTaskDelete(NULL);
  delay(2147483647);
}

template <typename T>
void run_robot(RunMode mode = RunMode::Blocking){
  if (mode == RunMode::Blocking){
    run_robot_inner<T>(NULL);
  }else if (mode == RunMode::NewTask || mode == RunMode::NewTaskEndCurrent){
      xTaskCreateUniversal(run_robot_inner<T>,"Robot Code", uxTaskGetStackHighWaterMark(NULL) , NULL, 1, NULL, ARDUINO_RUNNING_CORE);
      if (mode == RunMode::NewTaskEndCurrent){
        vTaskDelete(NULL);
      }
  }else{
    //idk how to panic in c++
  }
}

