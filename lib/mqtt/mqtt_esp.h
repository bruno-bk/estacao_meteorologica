#ifndef MQTT_ESP
#define MQTT_ESP

void set_parameters_mqtt(const char* server, int port, const char* user, const char* password);
void mqtt_loop(void *pvParameters);

#endif