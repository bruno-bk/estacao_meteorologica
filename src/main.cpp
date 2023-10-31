#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <DHT.h>
#include <AS5600.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>

#include "wifi_esp.h"
#include "mqtt_esp.h"
#include "config.h"

#define DHTPIN 4
#define ENCODERPIN 5

DHT dht(DHTPIN, DHT11);
AS5600 as5600;
Adafruit_BMP280 bmp280;
BH1750 bh1750;

volatile long pulses;
unsigned long timeold;

void read_bh1750(void * pvParameters) {
    float l;
    char lux[8];
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        l = bh1750.readLightLevel();
        if (l < 0) {
            Serial.println(F("Failed to read lux from bh1750 sensor!"));
        } else {
            sprintf(lux, "%d", (uint32_t)l);
            Serial.print("[");
            Serial.print(millis());
            Serial.print("] Lux: ");
            Serial.println(lux);
            send_mqtt_message("station/lux", lux);
        }
        vTaskDelayUntil(&xLastWakeTime, 60000/portTICK_PERIOD_MS );
    }
}

void read_bmp280(void * pvParameters) {
    float t;
    char temperature[8];
    float p;
    char pressure[10];

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        t = bmp280.readTemperature();
        if (isnan(t)) {
            Serial.println(F("Failed to read temperature from bmp280 sensor!"));
        } else {
            dtostrf(t, 1, 1, temperature);
            Serial.print("Temperature: ");
            Serial.println(temperature);
            send_mqtt_message("station/temperature", temperature);
        }

        p = bmp280.readPressure();
        if (isnan(p)) {
            Serial.println(F("Failed to read pressure from bmp280 sensor!"));
        } else {
            dtostrf((p /= 100), 1, 1, pressure);
            Serial.print("Pressure: ");
            Serial.println(pressure);
            send_mqtt_message("station/pressure", pressure);
        }
        vTaskDelayUntil(&xLastWakeTime, 10000/portTICK_PERIOD_MS );
    }
}

void IRAM_ATTR counter() {
    pulses++;
}

void read_encoder(void * pvParameters) {
    char wind_speed[5];

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        detachInterrupt(ENCODERPIN);
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

        vTaskDelayUntil(&xLastWakeTime, 5000/portTICK_PERIOD_MS );
    }
}

void read_DHT11(void * pvParameters) {
    float h;
    char humidity[5];

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        h = dht.readHumidity();
        if (isnan(h)) {
            Serial.println(F("Failed to read from DHT sensor!"));
            return;
        }
        sprintf(humidity, "%d", (uint32_t)h);
        Serial.print("humidity: ");
        Serial.print(humidity);
        Serial.println("%");
        send_mqtt_message("station/humidity", humidity);

        vTaskDelayUntil(&xLastWakeTime, 10000/portTICK_PERIOD_MS );
    }
}

void read_as5600(void * pvParameters) {
    float a;
    char angle[5];

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        a = abs(as5600.rawAngle()*360.0/4095.0);

        sprintf(angle, "%d", (uint32_t)a);
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.println("°");
        send_mqtt_message("station/angle", angle);

        vTaskDelayUntil(&xLastWakeTime, 10000/portTICK_PERIOD_MS );
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(ENCODERPIN, INPUT);
    timeold = millis();
    attachInterrupt(ENCODERPIN, counter, FALLING);

    dht.begin();

    as5600.begin(4);
    as5600.setDirection(AS5600_CLOCK_WISE);

    if (!bmp280.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor.");
    }

    if (!bh1750.begin(bh1750.CONTINUOUS_HIGH_RES_MODE, 0x23)){
        Serial.println("Could not find a valid BH1750 sensor.");
    }

    set_parameters_wifi(WIFI_SSID, WIFI_PASSWORD);
    set_parameters_mqtt(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);

    xTaskCreate(wifi_loop, "wifi_manager", 4096, NULL, 1, NULL);
    xTaskCreate(mqtt_loop, "mqtt_manager", 4096, NULL, 1, NULL);

    delay(1000);

    xTaskCreate(read_bh1750, "bh1750", 2048, NULL, 1, NULL);
    xTaskCreate(read_bmp280, "bmp280", 2048, NULL, 1, NULL);
    xTaskCreate(read_encoder, "encoder", 2048, NULL, 1, NULL);
    xTaskCreate(read_DHT11, "DHT11", 2048, NULL, 1, NULL);
    xTaskCreate(read_as5600, "as5600", 2048, NULL, 1, NULL);
}

void loop() {}
