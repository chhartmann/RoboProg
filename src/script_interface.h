#ifndef __SCRIPT_INTERFACE_H__
#define __SCRIPT_INTERFACE_H__

#include <LuaWrapper.h>

void script_setup();
void script_run(const char* data);
void script_stop();

#endif // __SCRIPT_INTERFACE_H__