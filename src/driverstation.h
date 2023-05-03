#pragma once

#include "esp32-hal-timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include <sys/_pthreadtypes.h>
#include "sys_arch.h"
#include "freertos/portmacro.h"
#include "esp32-hal.h"
#include <cassert>
#include <cstring>
#include "WiFiClient.h"
#include <cmath>
#include "roborio.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <atomic>

#define BUF_CAPACITY 256 * 8

struct Driverstation{
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

  //private tcp things
  // this is only accessed within the context of tcp_mutex being locked
  // so its safe to have it as non atomic
  uint16_t msg_sequence;

  //private udp things
  std::atomic<float> countdown;
  


  TaskHandle_t network_task_handle;
  SemaphoreHandle_t network_task_running;
  std::atomic<bool> exit_all_tasks;

  std::atomic<bool> connected;


  uint8_t buf[BUF_CAPACITY];

  static void driverstation_update_loop(Driverstation* instance){
    while(!instance->exit_all_tasks.load(std::memory_order_relaxed)){
      instance->update();
      delay(1);
    }
    xSemaphoreGive(instance->network_task_running);
  }

public:

  void observe_voltage(RobotVoltage voltage){
    this->voltage.store(voltage, std::memory_order_relaxed);
  }

private:
  void observe_mode(uint8_t mode){
    auto initial = this->status.load(std::memory_order_relaxed);
    auto updated = initial;
    do{
      updated = initial;
      updated.dissabled = mode == 255;
      updated.test_code = mode == 0;
      updated.teleop_code = mode == 1;
      updated.auton_code = mode == 2;
    }while(!this->status.compare_exchange_weak(
      initial,
      updated,
      std::memory_order_relaxed,
      std::memory_order_relaxed
    ));
    this->robot_status_watchdog.store(micros(), std::memory_order_relaxed);
  }
public:
  void observe_disabled(){
    observe_mode(255);
  }

  void observe_teleop(){
    observe_mode(0);
  }

  void observe_test(){
    observe_mode(1);
  }

  void observe_auton(){
    observe_mode(2);
  }

  void observe_robot_code(){
    auto initial = this->status.load(std::memory_order_relaxed);
    auto updated = initial;
    do{
      updated = initial;
      updated.has_robot_code = true;
    }while(!this->status.compare_exchange_weak(
      initial,
      updated,
      std::memory_order_relaxed,
      std::memory_order_relaxed
    ));
    this->robot_status_watchdog.store(micros(), std::memory_order_relaxed);
  }

  void observe_roborio(){
    RobotStatusCode initial = this->status.load(std::memory_order_relaxed);
    RobotStatusCode updated = initial;
    do{
      updated = initial;
      updated.is_roborio = true;
    }while(!this->status.compare_exchange_weak(
      initial,
      updated,
      std::memory_order_relaxed,
      std::memory_order_relaxed
    ));
    this->robot_status_watchdog.store(micros(), std::memory_order_relaxed);
  }

  void start_brownout();

  void end_brownout();

  void estop();

  void request_disable(){
    auto initial = this->ds_request.load(std::memory_order_relaxed);
    auto updated = initial;
    do{
      updated = initial;
      updated.request_disable = true;
    }while(!this->ds_request.compare_exchange_weak(
      initial,
      updated,
      std::memory_order_relaxed,
      std::memory_order_relaxed
    ));
  }

  void request_time(){
    auto initial = this->ds_request.load(std::memory_order_relaxed);
    auto updated = initial;
    do{
      updated = initial;
      updated.request_time = true;
    }while(!this->ds_request.compare_exchange_weak(
      initial,
      updated,
      std::memory_order_relaxed,
      std::memory_order_relaxed
    ));
  }

  AllianceStation get_alliance_station(){
    return this->alliance_station.load(std::memory_order_relaxed);
  }

  ControlCode get_control_code(){
    return this->control.load(std::memory_order_relaxed);
  }

  float get_countdown(){
    return this->countdown.load(std::memory_order_relaxed);
  }

  Controller get_controller(uint8_t controller){
    if(controller > 5){
      return {};
    }
    xSemaphoreTake(this->udp_controller_mutex, portMAX_DELAY);
    auto controller_data = this->controllers[controller];
    xSemaphoreGive(this->udp_controller_mutex);
    return controller_data;
  }

  void begin(uint16_t udp_port = 1110, uint16_t udp_fms_port = 1115, uint16_t udp_send_port = 1150, uint16_t tcp_port = 1740){
    this->udp.begin(udp_port);
    this->udp_fms.begin(udp_fms_port);
    this->udp_send_port = udp_send_port;

    this->tcp = WiFiServer(tcp_port);
    this->tcp.begin();
    
    this->tcp_client_mutex = xSemaphoreCreateMutex();
    assert(this->tcp_client_mutex);

    this->network_task_running = xSemaphoreCreateMutex();
    assert(this->network_task_running);

    this->udp_estop_brownout_mutex = xSemaphoreCreateMutex();
    assert(this->udp_estop_brownout_mutex);

    this->udp_controller_mutex = xSemaphoreCreateMutex();
    assert(this->udp_controller_mutex);

    xSemaphoreTake(this->network_task_running, portMAX_DELAY);
    xTaskCreatePinnedToCore(
      (void (*)(void*))Driverstation::driverstation_update_loop, 
      "Driverstatoin Network Communication",
       10000, 
       this, 
       100, 
       &network_task_handle,
       xPortGetCoreID() == 0 ? 1 : 0
    );
    
    esp_task_wdt_add(network_task_handle);
  }

  ~Driverstation(){
    xSemaphoreTake(this->tcp_client_mutex, portMAX_DELAY);
    xSemaphoreTake(this->network_task_running, portMAX_DELAY);
    xSemaphoreTake(this->udp_estop_brownout_mutex, portMAX_DELAY);
    xSemaphoreTake(this->udp_controller_mutex, portMAX_DELAY);
    // vTaskDelete
    if(this->tcp_client && this->tcp_client.connected()){
      this->tcp_client.stop();
    }
    this->udp.stop();
    this->udp_fms.stop();
    this->tcp.stop();
    vSemaphoreDelete(this->tcp_client_mutex);
    vSemaphoreDelete(this->udp_controller_mutex);
    vSemaphoreDelete(this->network_task_handle);
    vSemaphoreDelete(this->udp_estop_brownout_mutex);
  }

  void print(const char* msg, size_t str_size){
    xSemaphoreTake(this->tcp_client_mutex, portMAX_DELAY);
    if(this->tcp_client){
      if(this->tcp_client.connected()){
        int time = millis();
        int total_size = str_size + 7;
        assert(total_size <= 0xFFFF && str_size + 7 > str_size);
        char val[] = {
          (char)(total_size >> 8), (char)(total_size), //size
          0x0c, //tag
          (char)(time >> 24),  //time
          (char)(time >> 16), 
          (char)(time >> 8), 
          (char)(time),

          (char)(this->msg_sequence >> 8), //sequence
          (char)(this->msg_sequence),
        };
        this->tcp_client.write(val, 9);
        this->tcp_client.write(msg, str_size);
        this->tcp_client.flush();
        this->msg_sequence += 1;
      }
    }
    xSemaphoreGive(this->tcp_client_mutex);
  }

  void print(std::string msg){
    print(msg.c_str(), msg.size());
  }

  void print(const char* msg){
    print(msg, strlen(msg));
  }

  void printf(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    char temp[64];
    char* buffer = temp;
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);
    if (len > sizeof(temp) - 1) {
        buffer = new (std::nothrow) char[len + 1];
        if (!buffer) {
            return;
        }
        va_start(arg, format);
        vsnprintf(buffer, len + 1, format, arg);
        va_end(arg);
    }
    print((const char*) buffer, len);
    if (buffer != temp) {
        delete[] buffer;
    }
  }

private:

  void update_control_mode(bool enabled, uint8_t mode, ControlCode initial){
      auto updated = initial;
      do{
        updated = initial;
        updated.enabled = enabled;
        updated.mode = mode;
      }while(!this->control.compare_exchange_weak(
        initial,
        updated,
        std::memory_order_relaxed,
        std::memory_order_relaxed
      ));
  }

  void disable();

  void teleop(){
    auto control = this->control.load(std::memory_order_relaxed);
    if(!(control.enabled && control.mode == 0)){
    this->update_control_mode(true, 0, control);
      Serial.println("Teleop");
    }
  }

  void auton(){
    auto control = this->control.load(std::memory_order_relaxed);
    if(!(control.enabled && control.mode == 2)){
    this->update_control_mode(true, 2, control);
      Serial.println("Auton");
    }
  }

  void test(){
    auto control = this->control.load(std::memory_order_relaxed);
    if(!(control.enabled && control.mode == 1)){
      this->update_control_mode(true, 1, control);
      Serial.println("Test");
    }
  }

  void update(){
    esp_task_wdt_reset();
    int len = 0;
    int packetSize = this->udp.parsePacket();
    if (packetSize <= 0){
      packetSize = this->udp_fms.parsePacket();
      if (packetSize > 0){
        len = this->udp_fms.read(this->buf, BUF_CAPACITY);
        if (len > 0)
          this->ds_ip = this->udp_fms.remoteIP();
      }
    }else{
      len = this->udp.read(this->buf, BUF_CAPACITY);  
      if (len > 0)
        this->ds_ip = udp.remoteIP(); 
    }


    if (len > 0){
      if(this->handle_incomming_udp(this->buf, len)){
        last_received_udp = micros();
      }

      if(this->tcp_client && this->tcp_client.connected()){
        this->tcp_client.flush();
      }
    }

    if (micros() - last_received_udp > 300 * 1000){
      disable();
    }
    if(micros() - this->robot_status_watchdog > 150 * 1000){
      this->status.store({}, std::memory_order_relaxed);
    }
    if(micros() - this->robot_status_watchdog > 200 * 1000){
      disable();
    }

    if(!this->tcp_client || !this->tcp_client.connected()){
      WiFiClient client = this->tcp.accept();
      if(client){
        if(client.connected()){
          if(client.remoteIP() == this->ds_ip){
            Serial.println("Client Connection With correct IP");
            this->tcp_client = client;
            return;
          }
        }
        client.stop();
      }
    }else{
      this->tcp_client.read(buf, BUF_CAPACITY);
    }
  }

  bool handle_incomming_udp(uint8_t buf[], uint len){
      uint16_t received_sequence;
      {
        DSToRobotUDP packet = DSToRobotUDP::from_big_endian_buffer(buf, (uint)len, BUF_CAPACITY);
        if (packet.data->com_version != 1){
          return false;
        }
        if(packet.data->request_code.restart_roborio_code){
          ESP.restart();
        }
        if(packet.data->request_code.restart_roborio){
          ESP.restart();
        }

        xSemaphoreTake(this->udp_estop_brownout_mutex, portMAX_DELAY);
        {
          auto current_control = this->control.load(std::memory_order_relaxed);
          if(packet.data->control_code.enabled != 
              current_control.enabled){
            auto mode = packet.data->control_code.mode;
            if(packet.data->control_code.estop | 
                current_control.estop){
              this->estop();
            }else if(current_control.brownout_protection){
              // do nothing
            }else if(!packet.data->control_code.enabled){
              this->disable();
            }else if (mode == 0){
              this->teleop();
            }else if (mode == 2){
              this->auton();
            }else if (mode == 1){
              this->test();
            }else{
              this->estop();
            }
          }
        }

        {
          auto initial = this->control.load(std::memory_order_relaxed);
          auto updated = initial;

          do{
            updated = initial;
            updated.estop |= packet.data->control_code.estop;
            if(updated.estop){
              updated.enabled = false;
            }
            updated.fms_attached = packet.data->control_code.fms_attached;
            updated.stop_connection = packet.data->control_code.stop_connection;
          }while(!this->control.compare_exchange_weak(
            initial,
            updated,
            std::memory_order_relaxed,
            std::memory_order_relaxed
          ));
        }
        xSemaphoreGive(this->udp_estop_brownout_mutex);

        received_sequence = packet.data->sequence;
        this->alliance_station.store(packet.data->alliance_station, std::memory_order_relaxed);
        

        int controller_count = 0;
        bool seen_countdown = false;

        uint tags_remainder_len = packet.tags_len();
        auto tags_remainder = packet.data->tags;
        while (tags_remainder_len > 0){
          auto tag_len = tags_remainder[0];
          tags_remainder = &tags_remainder[1];
          tags_remainder_len -= 1;
          if (tag_len > 0 && tag_len <= tags_remainder_len){
            auto tag = tags_remainder[0];
            tags_remainder_len -= tag_len;
            auto tag_data = &tags_remainder[1];
            tags_remainder = &tags_remainder[tag_len];
            // makes more sense to excluse the tag from the "tag data"
            tag_len -= 1;

            switch (tag){
              case 7:
              if (tag_len == 4){
                union Stupid{
                  uint32_t bits;
                  float val;
                };
                Stupid countdown = {
                  .bits = ((((uint32_t)tag_data[0]) << 24) | (((uint32_t)tag_data[1]) << 16) | (((uint32_t)tag_data[2]) << 8) | ((uint32_t)tag_data[3]))
                };
                this->countdown.store(countdown.val, std::memory_order_relaxed);
                seen_countdown = true;
              }
              break;
              case 12:
                if(this->read_controller_tag(tag_data, tag_len, controller_count)){
                  // this is horrible and gross but
                  // somehow there aren't nammed scopes
                  goto tag_loop_exit;
                }
                controller_count += 1;
                break;
              default:
                continue;
            }
          }
        }
        tag_loop_exit:

        xSemaphoreTake(this->udp_controller_mutex, portMAX_DELAY);
        for(; controller_count < 6; controller_count++){
          this->controllers[controller_count].metadata.exists = false;
        }
        xSemaphoreGive(this->udp_controller_mutex);

        if(!seen_countdown){
          this->countdown.store((float)NAN, std::memory_order_relaxed);
        }
      }

      if (this->current_sequence + 1 >= received_sequence || received_sequence - this->current_sequence > 10 ){
        this->current_sequence = received_sequence;
        RobotToDSUDP packet = RobotToDSUDP::from_little_endian_buffer(buf, BUF_CAPACITY);
        packet.data->com_version = 1;
        packet.data->voltage = this->voltage.load(std::memory_order_relaxed);
        packet.data->control_code = this->control.load(std::memory_order_relaxed);
        packet.data->sequence = received_sequence;
        packet.data->status = this->status.load(std::memory_order_relaxed);
        packet.data->request = this->ds_request.load(std::memory_order_relaxed);


        this->udp.beginPacket(this->udp.remoteIP(), this->udp_send_port);
        this->udp.write(packet.to_big_endian_buffer(), packet.size);
        this->udp.endPacket();
      }

      return true;
  }

  bool read_controller_tag(uint8_t data[], uint len, int controller){
    xSemaphoreTake(this->udp_controller_mutex, portMAX_DELAY);
    if (len == 0){
      this->controllers[controller].metadata.exists = false;
      return false;
    }

    int axis_len = data[0];
    if ((axis_len + 1 >= len) || (axis_len > 12)){
      this->controllers[controller].metadata.exists = false;
      return true;
    }
    this->controllers[controller].metadata.axis_len = axis_len;
    for(int i = 0; i < axis_len; i ++){
      this->controllers[controller].axis[i] = data[i+1];
    }
    for(int i = axis_len; i < 12; i ++){
      this->controllers[controller].axis[i] = 0;
    }

    // we can do this becuase we return if the length of the previous data
    // was greater than OR equal
    int buttons = data[axis_len+1];
    int buttons_len = (buttons+7)/8;
    if (buttons_len +2+axis_len >= len || buttons > 32){
      this->controllers[controller].metadata.exists = false;
      return true;
    }
    
    this->controllers[controller].metadata.button_len = buttons;
    uint32_t button_data = 0;
    for(int i = 0; i < buttons_len; i ++){
      button_data = button_data << 8 | (uint32_t)data[axis_len+2+i];
    }
    this->controllers[controller].buttons.bits = button_data;



    int povs = data[buttons_len +2+axis_len];
    int pov_len = povs*2;
    
    if (pov_len + 3 + buttons_len + axis_len > len || povs >2){
      this->controllers[controller].metadata.exists = false;
      return true;
    }


    this->controllers[controller].metadata.pov_len = povs;
    for(int i = 0; i < povs; i ++){
      auto d1 = data[i*2+3+buttons_len + axis_len];
      auto d2 = data[i*2+1+3+buttons_len + axis_len];
      this->controllers[controller].povs[i] = (uint16_t)(d1 << 8) | (uint16_t)d2;
    }

    for(int i = povs; i < 2; i ++){
      this->controllers[controller].povs[i] = 0;
    }

    this->controllers[controller].metadata.exists = true;

    xSemaphoreGive(this->udp_controller_mutex);

    return false;
  }
};

void IRAM_ATTR Driverstation::disable() {
  auto control = this->control.load(std::memory_order_relaxed);
  if(control.enabled && !control.estop){
    this->update_control_mode(false, 0, control);
    Serial.println("Disabled");
  }
}

void IRAM_ATTR Driverstation::estop() {
  xSemaphoreTake(this->udp_estop_brownout_mutex, portMAX_DELAY);
  auto initial = this->control.load(std::memory_order_relaxed);
  if(initial.estop){
    auto updated = initial;
    do{
      updated = initial;
      updated.enabled = false;
      updated.estop = true;
    }while(!this->control.compare_exchange_weak(
      initial,
      updated,
      std::memory_order_relaxed,
      std::memory_order_relaxed
    ));
    Serial.println("ESTOP");
  }
  xSemaphoreGive(this->udp_estop_brownout_mutex);
}

void IRAM_ATTR Driverstation::start_brownout(){
  xSemaphoreTake(this->udp_estop_brownout_mutex, portMAX_DELAY);
  auto initial = this->control.load(std::memory_order_relaxed);
  auto updated = initial;
  if (initial.brownout_protection){
    this->disable(); 
    xSemaphoreGive(this->udp_estop_brownout_mutex);
    return;
  }
  do{
    updated = initial;
    updated.brownout_protection = true;
  }while(!this->control.compare_exchange_weak(
    initial,
    updated,
    std::memory_order_relaxed,
    std::memory_order_relaxed
  ));
  this->disable();
  xSemaphoreGive(this->udp_estop_brownout_mutex);
}

void IRAM_ATTR Driverstation::end_brownout(){
  xSemaphoreTake(this->udp_estop_brownout_mutex, portMAX_DELAY);
  auto initial = this->control.load(std::memory_order_relaxed);
  auto updated = initial;
  do{
    updated = initial;
    updated.brownout_protection = false;
  }while(!this->control.compare_exchange_weak(
    initial,
    updated,
    std::memory_order_relaxed,
    std::memory_order_relaxed
  ));
  xSemaphoreGive(this->udp_estop_brownout_mutex);
}

static Driverstation DRIVERSTATION;

#undef BUF_CAPACITY