// Copyright (c) 2026 Juan Pedro Bandera Rubio
// Copyright (c) 2026 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ub_esp32_mqtt.h"

static const char *TAG = "ub_MQTT";

esp_mqtt_client_handle_t client = NULL;

uint32_t MQTT_CONNECTED = 0;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, (int)(event_id));
 
    ESP_LOGI(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, (int)(event_id));

    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        MQTT_CONNECTED=1;

        msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC, 0); 
        ESP_LOGI(TAG, "sent subscribe to topic %s successful, msg_id=%d", MQTT_TOPIC, msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        MQTT_CONNECTED=0;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);       
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// init MQTT client and connect to the broker. 
//      This function should be called before any publish or subscribe operations.
void ub_esp32_mqtt_init(void)
{
    esp_log_level_set("ub_MQTT", ESP_LOG_INFO);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URL,
        .broker.address.port = BROKER_PORT,
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// publish a message to the MQTT broker. 
//      Returns the message ID if successful, or -1 if the client is not initialized.
int ub_esp32_mqtt_publish(const char *message, int qos, int retain)
{
    if (client == NULL) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return -1;
    }

    int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, message, 0, qos, retain);
    return msg_id;
}

// publish robot data to the MQTT broker as a serialized JSON string. 
//      Returns the message ID if successful, or -1 if the client is not initialized.
int ub_esp32_mqtt_publish_robot_data(struct robot_data *data, int qos, int retain)
{
    if (client == NULL) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return -1;
    }
    // Serialize robot data to JSON
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return -1;
    }

    cJSON_AddNumberToObject(json, "robot_id", data->robot_id);
    cJSON_AddNumberToObject(json, "pos_x", data->pos_x);
    cJSON_AddNumberToObject(json, "pos_y", data->pos_y);
    cJSON_AddNumberToObject(json, "speed", data->speed);
    cJSON_AddNumberToObject(json, "heading", data->heading);

    char *json_string = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);

    if (json_string == NULL) {
        ESP_LOGE(TAG, "Failed to serialize JSON");
        return -1;
    }

    int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, json_string, 0, qos, retain);
    free(json_string);
    return msg_id;
}