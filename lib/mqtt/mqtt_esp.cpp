#include "mqtt_esp.h"
#include "wifi_esp.h"
#include "config.h"

#include <PubSubClient.h>

extern WiFiClient wifi_esp;
PubSubClient client_mqtt(wifi_esp);

char mqtt_server[100] = "";
int mqtt_port = 1883;
char mqtt_user[50];
char mqtt_password[50];

void connect_broker() {
    client_mqtt.setServer(mqtt_server, mqtt_port);
    while (!client_mqtt.connected()) {
        Serial.print("Attempting MQTT connection to ");
        Serial.println(mqtt_server);
        if (client_mqtt.connect("Estacao_meteorologica")) {
            Serial.println("connected");

            client_mqtt.subscribe("esp32/output");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client_mqtt.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

boolean client_mqtt_is_connected(){
    return client_mqtt.connected();
}

void send_mqtt_message(const char* topic, const char* msg) {
    Serial.print("send: ");
    Serial.print(msg);
    Serial.print(" to ");
    Serial.println(topic);
    client_mqtt.publish(topic, msg);
}

void set_parameters_mqtt(const char* server, int port, const char* user, const char* password) {
    strcpy(mqtt_server, server);
    mqtt_port = port;
    strcpy(mqtt_user, user);
    strcpy(mqtt_password, password);
}

void mqtt_loop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        if (!client_mqtt.connected()) {
            connect_broker();
        }
        client_mqtt.loop();
        vTaskDelayUntil(&xLastWakeTime, 5000/portTICK_PERIOD_MS );
    }
}

