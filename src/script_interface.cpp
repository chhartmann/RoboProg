#include <script_interface.h>
#include <servo_handler.h>

LuaWrapper lua;

static int lua_set_joint_angles(lua_State *lua_state) {
  for (int i = 0; i < num_servos; ++i) {
    servos[i]->write(lua_tointeger(lua_state, i + 1));
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
  return 0;
}

static int lua_wrapper_delay(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  delay(a);
  return 0;
}

void script_setup() {
  lua.Lua_register("setJointAngles", (const lua_CFunction) &lua_set_joint_angles);
  lua.Lua_register("pinMode", (const lua_CFunction) &lua_wrapper_pinMode);
  lua.Lua_register("digitalWrite", (const lua_CFunction) &lua_wrapper_digitalWrite);
  lua.Lua_register("digitalRead", (const lua_CFunction) &lua_wrapper_digitalRead);
  lua.Lua_register("delay", (const lua_CFunction) &lua_wrapper_delay);
}