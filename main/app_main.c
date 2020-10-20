/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"



#include "SGP30.h"
#include "AS7262.h"

static const char *TAG = "MQTT_EXAMPLE";

float sensorValues[AS7262_NUM_CHANNELS];
sgp30_t main_sensor;


SemaphoreHandle_t xSemaphore = NULL;



static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            break;

        case MQTT_EVENT_PUBLISHED:
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;

        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

/**< MQTT main task */
static void mqtt_app_start(void *arg) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .username = CONFIG_BROKER_USERNAME,
        .password = CONFIG_BROKER_PASSWORD
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

    int msg_id;
    char payload[100];

    
    vTaskDelay(1000 / portTICK_RATE_MS);

    while (1) {
        // Color sensor payload
        sprintf(payload, "esp1,sensor=\"Color\" v=%2f,b=%2f,g=%2f,y=%2f,o=%2f,r=%2f", 
            sensorValues[AS726x_VIOLET], sensorValues[AS726x_BLUE],
            sensorValues[AS726x_GREEN], sensorValues[AS726x_YELLOW],
            sensorValues[AS726x_ORANGE], sensorValues[AS726x_RED]);

        msg_id = esp_mqtt_client_publish(client, "devices/esp_1", payload, 0, 0, 0);
        ESP_LOGI(TAG, "Color sensor MQTT Publish, msg_id=%d", msg_id);

        // Air sensor payload
        sprintf(payload, "esp1,sensor=\"Air\" tvoc=%d,eco2=%d", main_sensor.TVOC, main_sensor.eCO2);

        msg_id = esp_mqtt_client_publish(client, "devices/esp_1", payload, 0, 0, 0);
        ESP_LOGI(TAG, "Air sensor MQTT Publish, msg_id=%d", msg_id);
        
        vTaskDelay(7000 / portTICK_RATE_MS);
    }
}

/**< AS7262 main task */
static void color_sensor_task(void *arg) {
    ESP_LOGI(TAG, "Color sensor task init");
    // i2c_master_init();
    
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE ) {
        as7262_init();
        xSemaphoreGive(xSemaphore);
    }
    
    vTaskDelay(1000 / portTICK_RATE_MS);

    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    set_led_drv_on(true);
    xSemaphoreGive(xSemaphore);
    vTaskDelay(1000 / portTICK_RATE_MS);
    
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    set_led_drv_on(false);
    xSemaphoreGive(xSemaphore);

    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    uint8_t temp = read_temperature();
    xSemaphoreGive(xSemaphore);


    // start_measurement();
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    set_conversion_type(MODE_2);
    xSemaphoreGive(xSemaphore);

    // Changing GAIN 
    control_setup.GAIN = GAIN_64X;
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        virtualWrite(AS726X_CONTROL_SETUP, get_control_setup_hex(control_setup));
        xSemaphoreGive(xSemaphore);
    }

    while(1) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            if (data_ready()) {
                read_calibrated_values(sensorValues, AS7262_NUM_CHANNELS);
                temp = read_temperature();
                xSemaphoreGive(xSemaphore);


                ESP_LOGI(TAG, "Device temperature: %d", temp);
                ESP_LOGI(TAG, " Violet:  %f", sensorValues[AS726x_VIOLET]);
                ESP_LOGI(TAG, " Blue:  %f", sensorValues[AS726x_BLUE]);
                ESP_LOGI(TAG, " Green:  %f", sensorValues[AS726x_GREEN]);
                ESP_LOGI(TAG, " Yellow:  %f", sensorValues[AS726x_YELLOW]);
                ESP_LOGI(TAG, " Orange:  %f", sensorValues[AS726x_ORANGE]);
                ESP_LOGI(TAG, " Red:  %f", sensorValues[AS726x_RED]);
                ESP_LOGI(TAG, " ------------------ ");
            } else {
                xSemaphoreGive(xSemaphore);
            }
        }

        vTaskDelay(5000 / portTICK_RATE_MS);
    }
}

static void air_sensor_task(void *arg) {
    ESP_LOGI(TAG, "SGP30 main task initializing...");

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        sgp30_init(&main_sensor);
        xSemaphoreGive(xSemaphore);
    }

    // SGP30 needs to be read every 1s and sends TVOC = 400 14 times when initializing
    for (int i = 0; i < 14; i++) {
        vTaskDelay(1000 / portTICK_RATE_MS);

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            sgp30_IAQ_measure(&main_sensor);
            xSemaphoreGive(xSemaphore);
        }

        ESP_LOGI(TAG, "SGP30 Calibrating... TVOC: %d,  eCO2: %d",  main_sensor.TVOC, main_sensor.eCO2);
    }

    // Read initial baselines 
    uint16_t eco2_baseline, tvoc_baseline;


    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        sgp30_get_IAQ_baseline(&main_sensor, &eco2_baseline, &tvoc_baseline);
        xSemaphoreGive(xSemaphore);
    }
    
    ESP_LOGI(TAG, "BASELINES - TVOC: %d,  eCO2: %d",  tvoc_baseline, eco2_baseline);


    ESP_LOGI(TAG, "SGP30 main task is running...");
    while(1) {
        vTaskDelay(1000 / portTICK_RATE_MS);

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            sgp30_IAQ_measure(&main_sensor);
            xSemaphoreGive(xSemaphore);
        }

        ESP_LOGI(TAG, "TVOC: %d,  eCO2: %d",  main_sensor.TVOC, main_sensor.eCO2);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xSemaphore = xSemaphoreCreateMutex();

    xSemaphoreGive(xSemaphore);


    xTaskCreate(mqtt_app_start, "mqtt_main_task", 1024 * 3, (void *)0, 10, NULL);
    xTaskCreate(color_sensor_task, "color_sensor_main_task", 1024 * 2, (void *)0, 20, NULL);
    xTaskCreate(air_sensor_task, "air_sensor_main_task", 1024 * 2, (void *)0, 15, NULL);
}
