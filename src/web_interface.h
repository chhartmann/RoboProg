#ifndef __WEB_INTERFACE_H__
#define __WEB_INTERFACE_H__

#include <string>

void web_setup();
void web_send(const char* type, std::string data);

#endif // __WEB_INTERFACE_H__