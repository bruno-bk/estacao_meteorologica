#ifndef WIFI_ESP
#define WIFI_ESP

#include <WiFi.h>

void wifi_loop( void * pvParameters );
void set_parameters_wifi(const char* wifi_ssid, const char* wifi_password);

#endif
