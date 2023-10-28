#include <Arduino.h>
#include "wifi_esp.h"
#include "mqtt_esp.h"
#include "config.h"

void setup() {
    Serial.begin(115200);
    
    set_parameters_wifi(WIFI_SSID, WIFI_PASSWORD);
    set_parameters_mqtt(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    wifi_loop();
    mqtt_loop();
    delay(1000);
}
