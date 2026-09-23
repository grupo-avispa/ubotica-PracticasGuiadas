| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |

# _MQTT communication using JSON over WiFi_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example code for the **Microbótica** subject at GIET provides the following:

- A library to easily connect your ESP32 S3 to a WiFi. You will need only to set the WiFi SSID and PASSWORD in the SDK Configuration and use a single function in your code.

- A library to start an MQTT client, subscribe to a topic, and publish messages to that topic. The library incorporates an example struct to store robot data, that you can expand according to your needs. This structure is published in the topic as a serialized JSON, so you can have a first experience in using JSON format.

- An example code that allows testing both libraries.
