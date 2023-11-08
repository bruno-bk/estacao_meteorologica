#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

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

SemaphoreHandle_t xMutex_I2C = NULL;

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
    float last_l = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        xSemaphoreTake(xMutex_I2C,portMAX_DELAY );
        l = bh1750.readLightLevel();
        xSemaphoreGive(xMutex_I2C);
        if (l < 0) {
            Serial.println(F("Failed to read lux from bh1750 sensor!"));
        } else if (abs(l - last_l) >= 3000) {
            last_l = l;
            xQueueSend(lux, &l, pdMS_TO_TICKS(0));
        }
        vTaskDelayUntil(&xLastWakeTime, 60000/portTICK_PERIOD_MS );
    }
}

float filter(float input, float* x, float* y, const float* a, const float* b) {
  x[2] = x[1];
  x[1] = x[0];
  y[2] = y[1];
  y[1] = y[0];
  
  x[0] = input;

  y[0] = (b[0] * x[0] + b[1] * x[1] + b[2] * x[2] - a[1] * y[1] -  a[2] * y[2]);

  return y[0];
}

void read_bmp280(void * pvParameters) {
    float t;
    float last_t = 0;
    float p;
    float last_p = 0;
    
    const float a[] = {1, -0.9390625058174923, 0};
    const float b[] = {0.030468747091253818, 0.030468747091253818, 0};

    float x[3] = {0, 0, 0};
    float y[3] = {0, 0, 0};

    const uint8_t SAMPLING_AMOUNT = 25;
    float moving_average[SAMPLING_AMOUNT];
    uint8_t index = 0;
    float sum = 0;
    for (index = 0; index < SAMPLING_AMOUNT; index++){
        moving_average[index] = bmp280.readPressure()/100;
        sum += moving_average[index];
    }
    index = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        for(int i = 0; i < 200; i++){
            xSemaphoreTake(xMutex_I2C,portMAX_DELAY );
            t = bmp280.readTemperature();
            xSemaphoreGive(xMutex_I2C);

            if (isnan(t)) {
                Serial.println(F("Failed to read temperature from bmp280 sensor!"));
            } else {
                t = filter(t, x, y, a, b);
            }
            vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS );
        }

        if (abs(t - last_t) >= 0.5) {
            last_t = t;
            xQueueSend(temperature, &t, pdMS_TO_TICKS(0));
        }

        p = bmp280.readPressure();
        p /= 100;
        if (isnan(p)) {
            Serial.println(F("Failed to read pressure from bmp280 sensor!"));
        } else {
            sum -= moving_average[index];
            moving_average[index] = p;
            sum += moving_average[index];

            index++;
            if(index == SAMPLING_AMOUNT){
                index = 0;
            }

            p = sum/SAMPLING_AMOUNT;

            if (abs(p - last_p) >= 0.5) {
                last_p = p;
                xQueueSend(pressure, &p, pdMS_TO_TICKS(0));
            }
        }
    }
}

void IRAM_ATTR counter() {
    pulses++;
}

void read_encoder(void * pvParameters) {
    float rps;
    float speed;
    float last_speed = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        detachInterrupt(ENCODERPIN);
        rps = ((1000 / 20.0) / (millis() - timeold)) * pulses;
        speed = rps * 0.5340707511102649 * 3.6; // 0.5340707511102649 = PI*0.085m
        pulses = 0;

        if (abs(speed - last_speed) >= 0.5) {
            last_speed = speed;
            xQueueSend(wind_speed, &speed, pdMS_TO_TICKS(0));
        }

        timeold = millis();
        attachInterrupt(ENCODERPIN, counter, RISING);

        vTaskDelayUntil(&xLastWakeTime, 30000/portTICK_PERIOD_MS );
    }
}

void read_DHT11(void * pvParameters) {
    float h;
    float last_h = 0;
    const uint8_t SAMPLING_AMOUNT = 25;
    float moving_average[SAMPLING_AMOUNT];
    uint8_t index = 0;
    float sum = 0;
    while (isnan(dht.readHumidity())){
        delay(1000);
    }
    for (index = 0; index < SAMPLING_AMOUNT; index++){
        moving_average[index] = dht.readHumidity();
        sum += moving_average[index];
    }
    index = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        h = dht.readHumidity();
        if (isnan(h)) {
            Serial.println(F("Failed to read from DHT sensor!"));
        } else {
            sum -= moving_average[index];
            moving_average[index] = h;
            sum += moving_average[index];

            index++;
            if(index == SAMPLING_AMOUNT){
                index = 0;
            }

            h = sum/SAMPLING_AMOUNT;

            if (abs(h - last_h) >= 1) {
                last_h = h;
                xQueueSend(humidity, &h, pdMS_TO_TICKS(0));
            }
        }
        vTaskDelayUntil(&xLastWakeTime, 20000/portTICK_PERIOD_MS );
    }
}

void read_as5600(void * pvParameters) {
    float a;
    float last_a = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        xSemaphoreTake(xMutex_I2C,portMAX_DELAY );
        a = abs(as5600.rawAngle()*360.0/4095.0);
        xSemaphoreGive(xMutex_I2C);

        if (abs(a - last_a) >= 3.6) {
            last_a = a;
            xQueueSend(angle, &a, pdMS_TO_TICKS(0));
        }

        vTaskDelayUntil(&xLastWakeTime, 30000/portTICK_PERIOD_MS );
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

    xMutex_I2C = xSemaphoreCreateMutex();

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
