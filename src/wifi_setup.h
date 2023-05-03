#include "Arduino.h"

void begin_mdns(uint16_t team_number);

void config_ap(uint16_t team_number, IPAddress dhcp_lease_start = (uint32_t)0);
void config_ap(IPAddress local_ip, IPAddress gateway, IPAddress subnet,
               IPAddress dhcp_lease_start = (uint32_t)0);

void begin_ap(uint16_t team_number, const char* name = NULL,
              const char* pass = NULL, int32_t channel = 1,
              bool start_mdns = true);

void configure_static_bridge(uint16_t team_number,
                             IPAddress subnet = IPAddress(255, 255, 255, 0),
                             IPAddress gateway = IPAddress(0, 0, 0, 0));
void configure_static_bridge(IPAddress static_ip, IPAddress gateway,
                             IPAddress subnet);

void begin_bridge(uint16_t team_number, const char* name,
                  const char* pass = NULL, int32_t channel = 0,
                  bool start_mdns = true);