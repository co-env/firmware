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


#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */


static const char *TAG = "MQTT_EXAMPLE";

i2c_port_t i2c_num = I2C_MASTER_NUM;

float sensorValues[AS7262_NUM_CHANNELS];
sgp30_t main_sensor;

SemaphoreHandle_t xSemaphore = NULL;

/******** I2C ********/

// ESP I2C Driver setup 
esp_err_t i2c_master_driver_initialize(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);   
}

/**
 * @brief generic function for reading I2C data
 * 
 * @param reg_addr register adress to read from 
 * @param reg_data pointer to save the data read 
 * @param len length of data to be read
 * @param intf_ptr 
 * 
 * >init: dev->intf_ptr = &dev_addr;
 * 
 * @return ESP_OK/BME280_OK if reading was successful
 */
int8_t main_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) { // *intf_ptr = dev->intf_ptr
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    if (len == 0) {
        return ESP_OK;
    }

    uint8_t addr = *(uint8_t*)intf_ptr;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    
    if(reg_addr != 0xff){
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    }

    if (len > 1) {
        i2c_master_read(cmd, reg_data, len, ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    rslt = ret;
    return rslt;

}

/**
 * @brief generic function for writing data via I2C 
 *  
 * @param reg_addr register adress to write to 
 * @param reg_data register data to be written 
 * @param len length of data to be written
 * @param intf_ptr 
 * 
 * @return ESP_OK/BME280_OK if writing was successful
 */
int8_t main_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    
    if(reg_addr != 0xff){
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    }

    i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    rslt = ret;
    return rslt;

}

/******** MQTT ********/
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
    // esp_mqtt_client_config_t mqtt_cfg = {
    //     .uri = CONFIG_BROKER_URL,
        // .username = CONFIG_BROKER_USERNAME,
        // .password = CONFIG_BROKER_PASSWORD
    // };
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://ec2-54-94-236-59.sa-east-1.compute.amazonaws.com",
        .username = "esp32",
        .password = "Senhamqtt1"
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
        as7262_init(main_i2c_read, main_i2c_write);
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
        sgp30_init(&main_sensor, main_i2c_read, main_i2c_write);
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
    ESP_ERROR_CHECK(i2c_master_driver_initialize());

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
