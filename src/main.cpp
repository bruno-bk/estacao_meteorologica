#include <Arduino.h>
#include "wifi_esp.h"
#include "config.h"

void setup() {
    Serial.begin(115200);
    
    set_parameters_wifi(WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
    wifi_loop();
    delay(1000);
}
