#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <config.h>
#include <script_interface.h>
#include <servo_handler.h>
#include <web_interface.h>

LuaWrapper lua;
TaskHandle_t luaTaskHandle = NULL;
String luaScript;

void luaTaskFunc(void * parameter){
  Serial.println("Lua task started");
  web_send_event("lua_output", "Lua task started");
  String result = lua.Lua_dostring(&luaScript);  
  Serial.println("Lua task finished");
  web_send_event("lua_output", "Lua task finished");
  Serial.println(result);
  web_send_event("lua_output", result);
  // TaskStatus_t pxTaskStatus;
  // vTaskGetInfo(NULL, &pxTaskStatus, pdTRUE, eInvalid);
  // Serial.println(String("Task Name: ") + pxTaskStatus.pcTaskName);
  // Serial.println(String("Task Stack Size: ") + pxTaskStatus.usStackHighWaterMark);
  luaTaskHandle = NULL;
  vTaskDelete(NULL);
}

void script_run(const char* data) {
  script_stop();
  luaScript = data;
//  Serial.println(data);
  xTaskCreate(luaTaskFunc, "Lua Task", 8000, NULL, 1, &luaTaskHandle);
}

void script_stop() {
  if (luaTaskHandle != NULL) {
    // TaskStatus_t pxTaskStatus;
    // vTaskGetInfo(luaTaskHandle, &pxTaskStatus, pdTRUE, eInvalid);
    // Serial.println(String("Task Name: ") + pxTaskStatus.pcTaskName);
    // Serial.println(String("Task Stack Size: ") + pxTaskStatus.usStackHighWaterMark);

    vTaskDelete(luaTaskHandle);
    luaTaskHandle = NULL;
    Serial.println("Lua task deleted");
    web_send_event("lua_output", "Lua task deleted");
  }  
}


static int lua_set_joint_angles(lua_State *lua_state) {
  for (int i = 0; i < num_servos; ++i) {
    set_servo_angle(i, luaL_checkinteger(lua_state, i + 1));
  }
  return 0;
}

static int lua_wrapper_pinMode(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  int b = luaL_checkinteger(lua_state, 2);
  pinMode(a, b);
  return 0;
}

static int lua_wrapper_digitalWrite(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  int b = luaL_checkinteger(lua_state, 2);
  digitalWrite(a, b);
  return 0;
}

static int lua_wrapper_digitalRead(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  int val = digitalRead(a);
  lua_pushnumber(lua_state, val);
  return 1;
}

static int lua_wrapper_delay(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  delay(a);
  return 0;
}

static int lua_log_serial(lua_State *lua_state) {
  const char* a = luaL_checkstring(lua_state, 1);
  Serial.print(a);
  return 0;
}

static int lua_log_web(lua_State *lua_state) {
  const char* a = luaL_checkstring(lua_state, 1);
  String msg = a;
  web_send_event("lua_output", msg);
  return 0;
}

static void add_json_to_table(lua_State *lua_state, const char* key, JsonVariant& val) {
  lua_pushstring(lua_state, key);
  if (val.is<JsonArray>()) {
    lua_newtable(lua_state);
    int index = 1;
    for (JsonVariant elem : val.as<JsonArray>()) {
      add_json_to_table(lua_state, String(index++).c_str(), elem);
    }
  } else if (val.is<JsonObject>()) {
    lua_newtable(lua_state);
    for (JsonPair kv : val.as<JsonObject>()) {
      const char* key = kv.key().c_str();
      JsonVariant val = kv.value();
      add_json_to_table(lua_state, key, val);
    }
  } else if (val.is<const char*>()) {
    lua_pushstring(lua_state, val.as<const char*>());
  } else if (val.is<int>()) {
    lua_pushnumber(lua_state, val.as<int>());
  } else if (val.is<bool>()) {
    lua_pushboolean(lua_state, val.as<bool>());
  } else {
    Serial.println("Unknown type in config.json");
  }
  lua_settable(lua_state, -3);
}

static int lua_get_config(lua_State *lua_state) {
  ConfigJsonDoc configDoc;
  File configFile = SPIFFS.open("/config.json", "r");
  deserializeJson(configDoc, configFile);
  configFile.close();
  JsonObject root = configDoc.as<JsonObject>();

  lua_newtable(lua_state);
  for (JsonPair kv : root) {
    const char* key = kv.key().c_str();
    JsonVariant val = kv.value();
    add_json_to_table(lua_state, key, val);
  }  
  return 1;
}

void script_setup() {
  lua.Lua_register("setJointAngles", (const lua_CFunction) &lua_set_joint_angles);
  lua.Lua_register("pinMode", (const lua_CFunction) &lua_wrapper_pinMode);
  lua.Lua_register("digitalWrite", (const lua_CFunction) &lua_wrapper_digitalWrite);
  lua.Lua_register("digitalRead", (const lua_CFunction) &lua_wrapper_digitalRead);
  lua.Lua_register("delay", (const lua_CFunction) &lua_wrapper_delay);
  lua.Lua_register("logSerial", (const lua_CFunction) &lua_log_serial);
  lua.Lua_register("logWeb", (const lua_CFunction) &lua_log_web);
  lua.Lua_register("getConfig", (const lua_CFunction) &lua_get_config);
}

