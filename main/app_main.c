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
#include "driver/i2c.h"

#include "SGP30.h"
#include "AS7262.h"
#include "esp32_bme280.h"
#include "microfone.h"
#include "display_task.h"


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

as7262_dev_t as7262_main_sensor;
sgp30_dev_t sgp30_main_sensor;
const adc_channel_t mic_channel = ADC_CHANNEL_6;

struct bme280_data comp_data; //TODO: mudar dado para global, usado por outras tasks


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
    int8_t ret = 0; /* Return 0 for Success, non-zero for failure */

    if (len == 0) {
        return ESP_OK;
    }

    uint8_t chip_addr = *(uint8_t*)intf_ptr;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    
    if (reg_addr != 0xff) {
        i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
        i2c_master_start(cmd);
    }
    
    i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    if (len > 1) {
        i2c_master_read(cmd, reg_data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    return ret;
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
    int8_t ret = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    
    if(reg_addr != 0xff){
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    }

    i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/******** MQTT ********/
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
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

/**
 * @brief generic delay function for BME280 library
 */
void main_delay_us(uint32_t period, void *intf_ptr) {
    /**
     * Return control or wait,
     * for a period amount of milliseconds
     */
    TickType_t delay = period / (1000 * portTICK_PERIOD_MS);
    vTaskDelay(delay);
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

    float _t;

    
    vTaskDelay(1000 / portTICK_RATE_MS);

    while (1) {
        // Color sensor payload
        sprintf(payload, "esp1,sensor=\"Color\" v=%2f,b=%2f,g=%2f,y=%2f,o=%2f,r=%2f", 
            as7262_main_sensor.calibrated_values[AS726x_VIOLET], as7262_main_sensor.calibrated_values[AS726x_BLUE],
            as7262_main_sensor.calibrated_values[AS726x_GREEN], as7262_main_sensor.calibrated_values[AS726x_YELLOW],
            as7262_main_sensor.calibrated_values[AS726x_ORANGE], as7262_main_sensor.calibrated_values[AS726x_RED]);

        msg_id = esp_mqtt_client_publish(client, "devices/esp_1", payload, 0, 0, 0);
        ESP_LOGI(TAG, "Color sensor MQTT Publish, msg_id=%d", msg_id);

        // Air sensor payload
        sprintf(payload, "esp1,sensor=\"Air\" tvoc=%d,eco2=%d", sgp30_main_sensor.TVOC, sgp30_main_sensor.eCO2);

        msg_id = esp_mqtt_client_publish(client, "devices/esp_1", payload, 0, 0, 0);
        ESP_LOGI(TAG, "Air sensor MQTT Publish, msg_id=%d", msg_id);

        memset(payload, 0, strlen(payload));
        
        _t = ((int)(comp_data.temperature / 100)) + (comp_data.temperature % 100) * 0.01;
        sprintf(payload, "esp1,sensor=\"Temperature\" temperature=%.2f", _t);
        msg_id = esp_mqtt_client_publish(client, "devices/esp_1", payload, 0, 0, 0);
        
        vTaskDelay(7000 / portTICK_RATE_MS);
    }
}

/****** Sensores *******/

/**< AS7262 main task */
static void color_sensor_task(void *arg) {
    ESP_LOGI(TAG, "Color sensor task init");
    // i2c_master_init();
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE ) {
        as7262_init(&as7262_main_sensor, (as7262_read_fptr_t)main_i2c_read, (as7262_write_fptr_t)main_i2c_write);
        xSemaphoreGive(xSemaphore);
    }
    
    vTaskDelay(1000 / portTICK_RATE_MS);

    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    as7262_set_led_drv_on(&as7262_main_sensor, true);
    xSemaphoreGive(xSemaphore);
    vTaskDelay(1000 / portTICK_RATE_MS);
    
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    as7262_set_led_drv_on(&as7262_main_sensor, false);
    xSemaphoreGive(xSemaphore);

    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    uint8_t temp = as7262_read_temperature(&as7262_main_sensor);
    xSemaphoreGive(xSemaphore);


    // start_measurement();
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    as7262_set_conversion_type(&as7262_main_sensor, MODE_2);
    xSemaphoreGive(xSemaphore);

    // Changing GAIN 
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        as7262_set_gain(&as7262_main_sensor, GAIN_64X);
        xSemaphoreGive(xSemaphore);
    }

    while(1) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            if (as7262_data_ready(&as7262_main_sensor)) {
                as7262_read_calibrated_values(&as7262_main_sensor, AS7262_NUM_CHANNELS);
                temp = as7262_read_temperature(&as7262_main_sensor);
                xSemaphoreGive(xSemaphore);


                ESP_LOGI(TAG, "Device temperature: %d", temp);
                ESP_LOGI(TAG, " Violet:  %f", as7262_main_sensor.calibrated_values[AS726x_VIOLET]);
                ESP_LOGI(TAG, " Blue:    %f", as7262_main_sensor.calibrated_values[AS726x_BLUE]);
                ESP_LOGI(TAG, " Green:   %f", as7262_main_sensor.calibrated_values[AS726x_GREEN]);
                ESP_LOGI(TAG, " Yellow:  %f", as7262_main_sensor.calibrated_values[AS726x_YELLOW]);
                ESP_LOGI(TAG, " Orange:  %f", as7262_main_sensor.calibrated_values[AS726x_ORANGE]);
                ESP_LOGI(TAG, " Red:     %f", as7262_main_sensor.calibrated_values[AS726x_RED]);
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
        sgp30_init(&sgp30_main_sensor, (sgp30_read_fptr_t)main_i2c_read, (sgp30_write_fptr_t)main_i2c_write);
        xSemaphoreGive(xSemaphore);
    }

    // SGP30 needs to be read every 1s and sends TVOC = 400 14 times when initializing
    for (int i = 0; i < 14; i++) {
        vTaskDelay(1000 / portTICK_RATE_MS);

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            sgp30_IAQ_measure(&sgp30_main_sensor);
            xSemaphoreGive(xSemaphore);
        }

        ESP_LOGI(TAG, "SGP30 Calibrating... TVOC: %d,  eCO2: %d",  sgp30_main_sensor.TVOC, sgp30_main_sensor.eCO2);
    }

    // Read initial baselines 
    uint16_t eco2_baseline, tvoc_baseline;


    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        sgp30_get_IAQ_baseline(&sgp30_main_sensor, &eco2_baseline, &tvoc_baseline);
        xSemaphoreGive(xSemaphore);
    }
    
    ESP_LOGI(TAG, "BASELINES - TVOC: %d,  eCO2: %d",  tvoc_baseline, eco2_baseline);


    ESP_LOGI(TAG, "SGP30 main task is running...");
    while(1) {
        vTaskDelay(1000 / portTICK_RATE_MS);

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            sgp30_IAQ_measure(&sgp30_main_sensor);
            xSemaphoreGive(xSemaphore);
        }

        ESP_LOGI(TAG, "TVOC: %d,  eCO2: %d",  sgp30_main_sensor.TVOC, sgp30_main_sensor.eCO2);
    }
}


static void bme280_sensor_task(void *arg) {
    ESP_LOGI(TAG, "SGP30 main task initializing...");
    esp_err_t erro = ESP_OK;

    // struct bme280_data comp_data; //TODO: mudar dado para global, usado por outras tasks

    //* init bme280
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        erro = bme280_sensor_init((bme280_read_fptr_t)main_i2c_read, (bme280_write_fptr_t)main_i2c_write, (bme280_delay_us_fptr_t)main_delay_us);
        xSemaphoreGive(xSemaphore);
    }

    if (erro != BME280_OK) {
        ESP_LOGE(TAG, "Failed to init BME280, error: %d", erro);
    } else {
        ESP_LOGI(TAG, "BME280 Initialized, error: %d", erro);
    }

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        erro = bme280_config();
        xSemaphoreGive(xSemaphore);
    }

    if(erro == BME280_OK) printf("Config check\n");
    else printf("Could not config BME280\n");

    while (1) {
        vTaskDelay(1000 / portTICK_RATE_MS);

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            erro = bme280_meas_forcedmode(&comp_data);
            xSemaphoreGive(xSemaphore);

            ESP_LOGI(TAG, "Temperature: %d",comp_data.temperature);
            ESP_LOGI(TAG, "Pressure: %d",comp_data.pressure);
            ESP_LOGI(TAG, "Humidity: %d",comp_data.humidity);

        }
        if(erro != BME280_OK) printf("Could not measure :(");
        
        //! Test only
        update_display_data(comp_data.temperature, sgp30_main_sensor.TVOC, sgp30_main_sensor.eCO2);
    }

}

static void sound_sensor_task(void *arg) {
    ESP_LOGI(TAG, "Mic main task initializing...");

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        adc1_config(mic_channel);
        xSemaphoreGive(xSemaphore);
    }

    while(1) {
        (void)get_voltage_variation(mic_channel);
        vTaskDelay(pdMS_TO_TICKS(1000));
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

    xTaskCreate( FontDisplayTask, "FontDisplayTask", 4096, NULL, 1, NULL );


    xTaskCreate(mqtt_app_start, "mqtt_main_task", 1024 * 3, (void *)0, 10, NULL);
    xTaskCreate(color_sensor_task, "color_sensor_main_task", 1024 * 2, (void *)0, 20, NULL);
    xTaskCreate(air_sensor_task, "air_sensor_main_task", 1024 * 2, (void *)0, 15, NULL);
    xTaskCreate(bme280_sensor_task, "bme280_sensor_main_task", 1024 * 2, (void *)0, 15, NULL);
    xTaskCreate(sound_sensor_task, "sound_sensor_main_task", 1024 * 2, (void *)0, 15, NULL);

}
