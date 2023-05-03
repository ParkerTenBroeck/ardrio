#include "wifi_setup.h"

#include "ESPmDNS.h"
#include "WiFi.h"

void begin_mdns(uint16_t team_number) {
  auto str =
      std::string("roboRIO-") + std::to_string(1114) + std::string("-FRC");

  if (!MDNS.begin(str.c_str())) {
    Serial.println("Bruh DNS dont work");
    return;
  }
  MDNS.addService("ni-rt", "tcp", 1110);
  Serial.println("Started mDNS Service");
}

void config_ap(uint16_t team_number, IPAddress dhcp_lease_start) {
  IPAddress local_ip(10, team_number / 100, team_number % 100, 2);
  IPAddress gateway(10, team_number / 100, team_number % 100, 1);
  IPAddress subnet(255, 255, 255, 0);

  config_ap(local_ip, gateway, subnet, dhcp_lease_start);
}

void config_ap(IPAddress local_ip, IPAddress gateway, IPAddress subnet,
               IPAddress dhcp_lease_start) {
  if (!WiFi.softAPConfig(local_ip, gateway, subnet)) {
    Serial.println("Failed to configure Wifi");
    return;
  }
}

void begin_ap(uint16_t team_number, const char* name, const char* pass,
              int32_t channel, bool start_mdns) {
  Serial.print("Starting AP ");

  std::string str_name;
  if (name) {
    str_name = std::string(name) + std::string("_") + std::to_string(1114);
  } else {
    str_name = std::to_string(1114);
  }

  if (pass) {
    WiFi.softAP(str_name.c_str(), pass, channel);
  } else {
    WiFi.softAP(str_name.c_str(), "", channel);
  }

  Serial.println("");
  Serial.println("WiFi Open");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  if (start_mdns) {
    begin_mdns(team_number);
  }
}

void configure_static_bridge(uint16_t team_number, IPAddress subnet,
                             IPAddress gateway) {
  IPAddress static_ip(10, team_number / 100, team_number % 100, 2);
  if (!gateway) {
    gateway = IPAddress(10, team_number / 100, team_number % 100, 1);
  }
  if (!WiFi.config(static_ip, gateway, subnet)) {
    Serial.println("Failed to configure Wifi");
    return;
  }
}

void configure_static_bridge(IPAddress static_ip, IPAddress subnet,
                             IPAddress gateway) {
  if (!WiFi.config(static_ip, gateway, subnet)) {
    Serial.println("Failed to configure Wifi");
    return;
  }
}

void begin_bridge(uint16_t team_number, const char* name, const char* pass,
                  int32_t channel, bool start_mdns) {
  assert(name);
  Serial.print("\nConnecting to: ");
  Serial.println(name);

  if (pass) {
    WiFi.begin(name, pass, channel);
  } else {
    WiFi.begin(name, "", channel);
  }
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  if (start_mdns) {
    begin_mdns(team_number);
  }
}