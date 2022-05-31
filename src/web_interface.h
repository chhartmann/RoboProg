#ifndef __WEB_INTERFACE_H__
#define __WEB_INTERFACE_H__

class String;

void web_setup();
void web_send(const char* type, String data);

#endif // __WEB_INTERFACE_H__