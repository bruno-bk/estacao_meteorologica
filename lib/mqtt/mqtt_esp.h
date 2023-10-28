#ifndef MQTT_ESP
#define MQTT_ESP

void send_mqtt_message(const char* topic, const char* msg);
void set_parameters_mqtt(const char* server, int port, const char* user, const char* password);
void mqtt_loop();

#endif