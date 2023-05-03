#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "data_types.h"
#include <WiFi.h>
#include <atomic>

#define BUF_CAPACITY 256 * 8

struct Driverstation
{
private:
  std::atomic<AllianceStation> alliance_station;
  std::atomic<ControlCode> control;
  std::atomic<RobotRequestCode> r_request;

  SemaphoreHandle_t udp_controller_mutex;
  Controller controllers[6];

  std::atomic<uint32_t> robot_status_watchdog;

  std::atomic<DriverstationRequestCode> ds_request;
  std::atomic<RobotStatusCode> status;
  std::atomic<RobotVoltage> voltage;

  // tcp coms
  WiFiClient tcp_client;
  SemaphoreHandle_t tcp_client_mutex;
  // only a single thread(task?) accesses this so no lock
  WiFiServer tcp;

  // udp coms
  // the same applies here where only a single thread(task?) accesses
  // these
  uint16_t current_sequence;
  uint32_t last_received_udp;
  WiFiUDP udp;
  WiFiUDP udp_fms;
  uint16_t udp_send_port;
  // this needs protection
  IPAddress ds_ip;
  SemaphoreHandle_t udp_estop_brownout_mutex;

  // private tcp things
  //  this is only accessed within the context of tcp_mutex being locked
  //  so its safe to have it as non atomic
  uint16_t msg_sequence;

  // private udp things
  std::atomic<float> countdown;

  TaskHandle_t network_task_handle;
  SemaphoreHandle_t network_task_running;
  std::atomic<bool> exit_all_tasks;

  std::atomic<bool> connected;

  uint8_t buf[BUF_CAPACITY];

  static void driverstation_update_loop(Driverstation *instance);

public:
  void begin(uint16_t udp_port = 1110, uint16_t udp_fms_port = 1115, uint16_t udp_send_port = 1150, uint16_t tcp_port = 1740);

  ~Driverstation();

public:
  void observe_disabled();
  void observe_teleop();
  void observe_test();
  void observe_auton();
  void observe_robot_code();
  void observe_roborio();

  void observe_voltage(RobotVoltage voltage);

  void start_brownout();
  void end_brownout();

  void estop();

  void request_disable();
  void request_time();

  AllianceStation get_alliance_station();
  ControlCode get_control_code();
  float get_countdown();
  Controller get_controller(uint8_t controller);


  void print(const char *msg, size_t str_size);
  void print(std::string msg);
  void print(const char *msg);
  void printf(const char *format, ...);

private:
  // this is for the observe_[mode] functions.
  // 255 dissable
  // 0 teleop
  // 1 test
  // 2 auto
  void observe_mode(uint8_t mode);

  void update_control_mode(bool enabled, uint8_t mode, ControlCode initial);
  void disable();
  void teleop();
  void auton();
  void test();

  void update();
  bool handle_incomming_udp(uint8_t buf[], uint len);
  bool read_controller_tag(uint8_t data[], uint len, int controller);
};

static Driverstation DRIVERSTATION;

#ifndef EXPOSE_DRIVERSTATION_BUF_CAPACITY
#undef BUF_CAPACITY
#endif
