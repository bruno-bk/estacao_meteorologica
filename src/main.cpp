#include <Arduino.h>
#include <DHT.h>

#include "wifi_esp.h"
#include "mqtt_esp.h"
#include "config.h"

#define DHTPIN 4

DHT dht(DHTPIN, DHT11);

void read_DHT11() {
    float h = dht.readHumidity();
    char humidity[5];

    if (isnan(h)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }
    sprintf(humidity, "%d", (uint32_t)h);
    Serial.print("humidity: ");
    Serial.print(humidity);
    Serial.println("%");
    send_mqtt_message("station/humidity", humidity);
}

void setup() {
    Serial.begin(115200);

    dht.begin();
    
    set_parameters_wifi(WIFI_SSID, WIFI_PASSWORD);
    set_parameters_mqtt(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    wifi_loop();
    mqtt_loop();

    read_DHT11();

    delay(5000);
}
