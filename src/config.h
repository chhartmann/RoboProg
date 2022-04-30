#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <ArduinoJson.h>

typedef StaticJsonDocument<512> ConfigJsonDoc;
static const char* wifi_ssid_key = "Wifi-SSID";
static const char* wifi_pwd_key = "Wifi-Password";
static const char* wifi_hostname_key = "Hostname";
static const char* ros_agent_ip_key = "ROS-Agent-IP";

#endif // __CONFIG_H__