#include <string>
#include <fstream>
#include <streambuf>

#include <esp_log.h>
#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <ArduinoJson.h>
#include <config.h>
#include <script_interface.h>
#include <servo_handler.h>
#include <web_interface.h>

extern "C" {
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

static const char *TAG = "lua";

lua_State* luaState;
TaskHandle_t luaTaskHandle = NULL;
std::string luaScript;

static void add_json_to_table(lua_State *lua_state, JsonVariant& val);

void luaTaskFunc(void * parameter){
  web_send("log", "Lua task started");
  std::string result;
  if (luaL_dostring(luaState, luaScript.c_str())) {
    result += "# lua error:\n" + std::string(lua_tostring(luaState, -1));
    lua_pop(luaState, 1);
  }
  web_send("log", "Lua task finished");
  ESP_LOGI(TAG, "%s", result.c_str());
  web_send("log", result);
  luaTaskHandle = NULL;
  vTaskDelete(NULL);
}

void script_run(const char* data) {
  script_stop();
  luaScript = data;
  ESP_LOGI(TAG, "%s", data);
  xTaskCreate(luaTaskFunc, "Lua Task", 8000, NULL, 1, &luaTaskHandle);
}

void script_stop() {
  if (luaTaskHandle != NULL) {
    vTaskDelete(luaTaskHandle);
    luaTaskHandle = NULL;
    web_send("log", "Lua task deleted");
  }
}


static int lua_set_joint_angles(lua_State *lua_state) {
  for (int i = 0; i < num_servos; ++i) {
    set_joint_angle(i, luaL_checkinteger(lua_state, i + 1));
  }
  return 0;
}

static int lua_wrapper_pinMode(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  int b = luaL_checkinteger(lua_state, 2);

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pin_bit_mask = 1ULL << a;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

  if (b == 0) { // input
    io_conf.mode = GPIO_MODE_INPUT;
  } else { // output
    io_conf.mode = GPIO_MODE_OUTPUT;
  }

  gpio_config(&io_conf);

  return 0;
}

static int lua_wrapper_digitalWrite(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  int b = luaL_checkinteger(lua_state, 2);
  gpio_set_level((gpio_num_t)a, b);
  return 0;
}

static int lua_wrapper_digitalRead(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  int val = gpio_get_level((gpio_num_t)a);
  lua_pushnumber(lua_state, val);
  return 1;
}

static int lua_wrapper_delay(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  vTaskDelay( a / portTICK_PERIOD_MS);
  return 0;
}

static int lua_log_serial(lua_State *lua_state) {
  const char* a = luaL_checkstring(lua_state, 1);
  ESP_LOGI(TAG, "%s", a);
  return 0;
}

static int lua_log_web(lua_State *lua_state) {
  const char* a = luaL_checkstring(lua_state, 1);
  std::string msg = a;
  web_send("log", msg);
  return 0;
}

static void add_json_to_table(lua_State *lua_state, const char* key, JsonVariant& val) {
  lua_pushstring(lua_state, key);
  add_json_to_table(lua_state, val);
  lua_settable(lua_state, -3);
}

static void add_json_to_table(lua_State *lua_state, int key, JsonVariant& val) {
  lua_pushnumber(lua_state, key);
  add_json_to_table(lua_state, val);
  lua_settable(lua_state, -3);
}

static void add_json_to_table(lua_State *lua_state, JsonVariant& val) {
  if (val.is<JsonArray>()) {
    lua_newtable(lua_state);
    int index = 1;
    for (JsonVariant elem : val.as<JsonArray>()) {
      add_json_to_table(lua_state, index++, elem);
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
    ESP_LOGE(TAG, "Unknown type in config.json");
  }
}

static int lua_get_config(lua_State *lua_state) {
  ConfigJsonDoc configDoc;
  std::ifstream t("/spiffs/config.json");
  std::string configFile((std::istreambuf_iterator<char>(t)),
                  std::istreambuf_iterator<char>());
  (void)deserializeJson(configDoc, configFile);
  lua_newtable(lua_state);
  JsonVariant config = configDoc.as<JsonVariant>();
  add_json_to_table(lua_state, config);
  return 1;
}

void script_setup() {
  luaState = luaL_newstate();
  luaopen_base(luaState);
  luaopen_table(luaState);
  luaopen_string(luaState);
  luaopen_math(luaState);
  lua_register(luaState, "setJointAngles", lua_set_joint_angles);
  lua_register(luaState, "pinMode", lua_wrapper_pinMode);
  lua_register(luaState, "digitalWrite", lua_wrapper_digitalWrite);
  lua_register(luaState, "digitalRead", lua_wrapper_digitalRead);
  lua_register(luaState, "delay", lua_wrapper_delay);
  lua_register(luaState, "logSerial", lua_log_serial);
  lua_register(luaState, "logWeb", lua_log_web);
  lua_register(luaState, "getConfig", lua_get_config);
}

