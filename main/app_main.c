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
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"

#include "sensors.h"
#include "network.h"
#include "feedback.h"
#include "rtos_sync.h"

#include "sdkconfig.h"

#include "mesh_device_app.h"

static const char *TAG = "MAIN";

EventGroupHandle_t sensorsEventGroup = NULL;


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

    sensor_timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    feedback_timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); 

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(i2c_master_driver_initialize());
    
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    #if CONFIG_COENV_NODE_TYPE_GATEWAY
    ESP_ERROR_CHECK(example_connect());
    #endif

    err = ble_mesh_device_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err 0x%06x)", err);
    }

    xSemaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(xSemaphore);

    sensorsEventGroup = xEventGroupCreate();
    xEventGroupClearBits(sensorsEventGroup, 0xff);

    tg0_timer_init(SENSOR_ID, SENSOR_INTERVAL_SEC); 
    

    #if CONFIG_COENV_NODE_TYPE_GATEWAY
    xTaskCreate(gateway_device_task, "gateway_main_task", 1024 * 4, (void *)0, 30, NULL);
    #elif CONFIG_COENV_NODE_TYPE_SENSOR
    xTaskCreate(node_device_task, "node_main_task", 1024 * 3, (void *)0, 30, NULL);
    #endif

    SSD1306_i2c_bus_init();

    xTaskCreate(main_sensor_task, "main_sensor_task", 2048, NULL, 10, NULL); //start gpio task
    xTaskCreate(feedback_task, "feedback_task", 2048, NULL, 10, NULL); //start gpio task

}
