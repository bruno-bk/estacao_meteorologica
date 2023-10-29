#include <Arduino.h>
#include <DHT.h>
#include <AS5600.h>

#include "wifi_esp.h"
#include "mqtt_esp.h"
#include "config.h"

#define DHTPIN 4
#define ENCODERPIN 5

DHT dht(DHTPIN, DHT11);
AS5600 as5600;

volatile long pulses;
unsigned long timeold;

void IRAM_ATTR counter() {
    pulses++;
}

void read_encoder() {
    detachInterrupt(ENCODERPIN);
    char wind_speed[5];

    double rps = ((1000 / 20.0) / (millis() - timeold)) * pulses;
    double speed = rps * 0.5340707511102649 * 3.6; // 0.5340707511102649 = PI*0.085m
    pulses = 0;

    Serial.print(rps);
    Serial.print(" rps | ");
    Serial.print(speed);
    Serial.println(" km/h");

    dtostrf(speed, 1, 1, wind_speed);
    send_mqtt_message("station/wind_speed", wind_speed);

    timeold = millis();
    attachInterrupt(ENCODERPIN, counter, RISING);    
}

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

void read_as5600() {
    float a = abs((as5600.rawAngle()*360.0/4095.0));
    char angle[5];

    sprintf(angle, "%d", (uint32_t)a);
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.println("°");
    send_mqtt_message("station/angle", angle);
}

void setup() {
    Serial.begin(115200);

    pinMode(ENCODERPIN, INPUT);
    timeold = millis();
    attachInterrupt(ENCODERPIN, counter, FALLING);

    dht.begin();

    as5600.begin(4);
    as5600.setDirection(AS5600_CLOCK_WISE);
    
    set_parameters_wifi(WIFI_SSID, WIFI_PASSWORD);
    set_parameters_mqtt(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    wifi_loop();
    mqtt_loop();

    read_DHT11();
    read_as5600();
    read_encoder();

    delay(5000);
}
