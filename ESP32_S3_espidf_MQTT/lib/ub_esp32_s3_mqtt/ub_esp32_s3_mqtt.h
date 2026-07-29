// Library to manage MQTT connection for ESP32-S3
#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "protocol_examples_common.h"
#include "mqtt_client.h"

// MQTT client handler
static esp_mqtt_client_handle_t client = NULL;

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
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

/*
* @brief Functions to initialize and start the MQTT client
 *
 *  These functions initialize the MQTT client with the specified configuration and start it.
 *  They also register the MQTT event handler to handle events such as connection, disconnection,
 *  subscription, and message reception.
 */
void mqtt_app_start_url(const char* broker_url);
void mqtt_app_start(const char* broker_url, uint32_t broker_port, const char* username, const char* password);

/*
 * @brief Function to subscribe to an MQTT topic
 *
 *  This function subscribes the MQTT client to the specified topic.
 *
 * @param topic The topic to subscribe to.
 * @param qos The quality of service for the subscription.
 */
void mqtt_subscribe(const char* topic, int qos);

/*
 * @brief Function to publish a message on an MQTT topic
 *
 *  This function publishes a message on the specified topic.
 *
 * @param topic The topic to publish the message to.
 * @param data The message data to publish.
 * @param len The length of the message data.
 * @param qos The quality of service for the publication.
 * @param retain Whether to retain the message.
 */
void mqtt_publish(const char* topic, const char* data, int len, int qos, int retain);

/*
* @brief Function to unsubscribe from an MQTT topic
    *
    *  This function unsubscribes the MQTT client from the specified topic.
    *
    * @param topic The topic to unsubscribe from.
    */
void mqtt_unsubscribe(const char* topic);