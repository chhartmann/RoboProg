#ifndef __WEB_INTERFACE_H__
#define __WEB_INTERFACE_H__

void web_setup();
void web_send_event(const char* event_name, String& event_data);

#endif // __WEB_INTERFACE_H__