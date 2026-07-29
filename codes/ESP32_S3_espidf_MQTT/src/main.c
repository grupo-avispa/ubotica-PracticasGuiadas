/* 
 * ESP32_ESPIDF_MQTT
 *
 * PlatformIO project using ESPIDF framework to demonstrate MQTT communication on ESP32.
 *
 *  For this example you will just need a ESP32-S3
 * 
 *  Created on: 2024-07-10
 *  Author: Juan Pedro Bandera Rubio, Dpto. de Tecnología Electrónica, Universidad de Málaga
 *  License: TODO!!!
*/
#include <stdio.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/task.h"

#include "ub_esp32_s3_mqtt.h"
#include "ub_esp32_s3_wifi.h"
#include "sdkconfig.h"

static const char *TAG = "mqtt_example";

// Get the Wi-Fi credentials from the platformio.ini file
static char* WIFI_SSID = CONFIG_SSID;
static char* WIFI_PASSWORD = CONFIG_PSSWD;
// Get the MQTT configuration from the platformio.ini file
static char* MQTT_BROKER_URL = CONFIG_MQTT_BROKER;
/*static int MQTT_BROKER_PORT = CONFIG_MQTT_PORT;
static char* MQTT_USERNAME = CONFIG_MQTT_USERNAME;
static char* MQTT_PASSWORD = CONFIG_MQTT_PASSWORD;*/
static char* MQTT_TOPIC = CONFIG_MQTT_TOPIC;

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Wi-Fi...");
    ESP_ERROR_CHECK(ub_esp32_s3_wifi_init());

    esp_err_t ret = ub_esp32_s3_wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi network");
    }

    wifi_ap_record_t ap_info;
    ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "Wi-Fi station interface not initialized");
    }
    else if (ret == ESP_ERR_WIFI_NOT_CONNECT) {
        ESP_LOGE(TAG, "Wi-Fi station is not connected");
    } else {
        ESP_LOGI(TAG, "--- Access Point Information ---");
        ESP_LOG_BUFFER_HEX("MAC Address", ap_info.bssid, sizeof(ap_info.bssid));
        ESP_LOG_BUFFER_CHAR("SSID", ap_info.ssid, sizeof(ap_info.ssid));
        ESP_LOGI(TAG, "Primary Channel: %d", ap_info.primary);
        ESP_LOGI(TAG, "RSSI: %d", ap_info.rssi);
    }

    // Initialize and start the MQTT client
    mqtt_app_start_url(MQTT_BROKER_URL);

    // Test: Subscribe to a topic to listen for messages
    mqtt_subscribe(MQTT_TOPIC, 0);

    // Test: Publish a message to the topic
    const char* message = "Hello from ESP32-S3!";
    mqtt_publish(MQTT_TOPIC, message, strlen(message), 0, 0);

    // Test: Wait for a while to receive messages
    vTaskDelay(pdMS_TO_TICKS(10000));

    // Test: Unsubscribe from the topic
    mqtt_unsubscribe(MQTT_TOPIC);

    // Test: Disconnect from Wi-Fi
    ret = ub_esp32_s3_wifi_disconnect();

}