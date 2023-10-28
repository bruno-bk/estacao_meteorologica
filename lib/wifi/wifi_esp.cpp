#include "wifi_esp.h"

WiFiClient wifi_esp;

char ssid[50];
char password[50];

void connect_wifi() {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.print("  ");

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void set_parameters_wifi(const char* wifi_ssid, const char* wifi_password) {
    strcpy(ssid, wifi_ssid);
    strcpy(password, wifi_password);
}

void wifi_loop() {
    if (WiFi.status() != WL_CONNECTED){
        connect_wifi();
    }
}
