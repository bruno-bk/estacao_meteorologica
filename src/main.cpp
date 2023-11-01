#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <DHT.h>
#include <AS5600.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>

#include "wifi_esp.h"
#include "mqtt_esp.h"
#include "config.h"

QueueHandle_t lux;
QueueHandle_t temperature;
QueueHandle_t pressure;
QueueHandle_t wind_speed;
QueueHandle_t humidity;
QueueHandle_t angle;

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
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        l = bh1750.readLightLevel();
        if (l < 0) {
            Serial.println(F("Failed to read lux from bh1750 sensor!"));
        } else {
            xQueueSend(lux, &l, pdMS_TO_TICKS(0));
        }
        vTaskDelayUntil(&xLastWakeTime, 60000/portTICK_PERIOD_MS );
    }
}

void read_bmp280(void * pvParameters) {
    float t;
    float p;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        t = bmp280.readTemperature();
        if (isnan(t)) {
            Serial.println(F("Failed to read temperature from bmp280 sensor!"));
        } else {
            xQueueSend(temperature, &t, pdMS_TO_TICKS(0));
        }

        p = bmp280.readPressure();
        if (isnan(p)) {
            Serial.println(F("Failed to read pressure from bmp280 sensor!"));
        } else {
            xQueueSend(pressure, &p, pdMS_TO_TICKS(0));
        }
        vTaskDelayUntil(&xLastWakeTime, 10000/portTICK_PERIOD_MS );
    }
}

void IRAM_ATTR counter() {
    pulses++;
}

void read_encoder(void * pvParameters) {
    float rps;
    float speed;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        detachInterrupt(ENCODERPIN);
        rps = ((1000 / 20.0) / (millis() - timeold)) * pulses;
        speed = rps * 0.5340707511102649 * 3.6; // 0.5340707511102649 = PI*0.085m
        pulses = 0;

        Serial.print(rps);
        Serial.print(" rps | ");
        Serial.print(speed);
        Serial.println(" km/h");

        xQueueSend(wind_speed, &speed, pdMS_TO_TICKS(0));

        timeold = millis();
        attachInterrupt(ENCODERPIN, counter, RISING);

        vTaskDelayUntil(&xLastWakeTime, 5000/portTICK_PERIOD_MS );
    }
}

void read_DHT11(void * pvParameters) {
    float h;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        h = dht.readHumidity();
        if (isnan(h)) {
            Serial.println(F("Failed to read from DHT sensor!"));
        } else {
            xQueueSend(humidity, &h, pdMS_TO_TICKS(0));
        }

        vTaskDelayUntil(&xLastWakeTime, 10000/portTICK_PERIOD_MS );
    }
}

void read_as5600(void * pvParameters) {
    float a;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        a = abs(as5600.rawAngle()*360.0/4095.0);

        xQueueSend(angle, &a, pdMS_TO_TICKS(0));

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

    lux = xQueueCreate(10, sizeof(float));
    temperature = xQueueCreate(10, sizeof(float));
    pressure = xQueueCreate(10, sizeof(float));
    wind_speed = xQueueCreate(10, sizeof(float));
    humidity = xQueueCreate(10, sizeof(float));
    angle = xQueueCreate(10, sizeof(float));

    xTaskCreate(wifi_loop, "wifi_manager", 4096, NULL, 1, NULL);
    delay(1000);
    xTaskCreate(mqtt_loop, "mqtt_manager", 4096, NULL, 1, NULL);
    delay(1000);
    xTaskCreate(read_bh1750, "bh1750", 2048, NULL, 1, NULL);
    delay(500);
    xTaskCreate(read_bmp280, "bmp280", 2048, NULL, 1, NULL);
    delay(500);
    xTaskCreate(read_encoder, "encoder", 2048, NULL, 1, NULL);
    delay(500);
    xTaskCreate(read_DHT11, "DHT11", 2048, NULL, 1, NULL);
    delay(500);
    xTaskCreate(read_as5600, "as5600", 2048, NULL, 1, NULL);
}

void loop() {}
