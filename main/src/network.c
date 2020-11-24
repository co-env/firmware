/**
 * @file network.c
 * 
 * @brief
 * 
 * @author
 * 
 * @date  11/2020
 */

#include "network.h"
#include "mesh_device_app.h"

#include "display_task.h"

static const char *TAG = "NETWORK";


/**! BLE Mesh Server Model Syncronized Queue 
 *  This is where received data from all other sensors is added 
 */ 
QueueHandle_t ble_mesh_received_data_queue = NULL;


/**
 * @brief  MQTT Event Handler Callback function
 * 
 * @param  handler_args
 * @param  base
 * @param  event_id
 * @param  event_data
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

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
}

/**
 * @brief  Sends `model_sensor_data_t` data to a cloud hosted MQTT broker.
 * 
 * @param  mqtt_client  MQTT client handler with connection arguments
 * @param  sensor_data  Struct containing all data that will be sent
 * 
 * @note  Sent data is formatted as InfluxDB queries
 */
static esp_err_t send_data_to_cloud(esp_mqtt_client_handle_t mqtt_client, model_sensor_data_t sensor_data) {
    char payload[200];
    esp_err_t err;
    char mqtt_topic[14];

    sprintf(mqtt_topic, "devices/%s", sensor_data.device_name);


    /** BME280 Payload */
    sprintf(payload, "%s,sensor=\"Temperature\" temperature=%.2f,humidity=%.2f", sensor_data.device_name, sensor_data.temperature, sensor_data.humidity);
    err = esp_mqtt_client_publish(mqtt_client, mqtt_topic, payload, 0, 0, 0);

    if (err) {
        return err;
    }

    /** SGP30 Payload */
    memset(payload, 0, strlen(payload));
    sprintf(payload, "%s,sensor=\"Air\" tvoc=%d,eco2=%d", sensor_data.device_name, sensor_data.tVOC, sensor_data.eCO2);
    err = esp_mqtt_client_publish(mqtt_client, mqtt_topic, payload, 0, 0, 0);
    if (err) {
        return err;
    }

    /** AS7262 Payload */
    memset(payload, 0, strlen(payload));
    sprintf(payload, "%s,sensor=\"Color\" v=%.2f,b=%.2f,g=%.2f,y=%.2f,o=%.2f,r=%.2f", sensor_data.device_name,
                sensor_data.violet, sensor_data.blue, sensor_data.green, sensor_data.yellow, sensor_data.orange, sensor_data.red);
    err = esp_mqtt_client_publish(mqtt_client, mqtt_topic, payload, 0, 0, 0);

    return err;
} 


void gateway_device_task(void *arg) { 

     /** MQTT Related Initializations */
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .username = CONFIG_BROKER_USERNAME,
        .password = CONFIG_BROKER_PASSWORD
    };
    
    //* Syncronized queue where received data is put by the Server Model 
    ble_mesh_received_data_queue = xQueueCreate(5, sizeof(model_sensor_data_t));
    ESP_LOGW(TAG, "Queue initialization done");

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    ESP_LOGW(TAG, "MQTT initialization done.");

    vTaskDelay(1000 / portTICK_RATE_MS);

    /** BLE Mesh Server Model initializations */
    model_sensor_data_t _received_data;  /*!< Received data from another sensor */

    model_sensor_data_t _own_sensor_data = {  /*!< Data from this sensor */ 
        .device_name = CONFIG_DEVICE_ID,
    };

    while (1) {
        /**< Checks queue to see if there's new incoming BLE Mesh data */
        if (xQueueReceive(ble_mesh_received_data_queue, &_received_data, 1000 / portTICK_RATE_MS) == pdPASS) {
            ESP_LOGI(TAG, "Recebido dados de %s", _received_data.device_name);
            ESP_LOGI(TAG, "    Temperatura: %f", _received_data.temperature);
            ESP_LOGI(TAG, "    Pressao:     %f", _received_data.pressure);
            ESP_LOGI(TAG, "    Umidade:     %f", _received_data.humidity);
            ESP_LOGI(TAG, "    TVOC:        %d", _received_data.tVOC);
            ESP_LOGI(TAG, "    eCO2:        %d", _received_data.eCO2);
            ESP_LOGI(TAG, "    Ruido:       %d", _received_data.noise_level);
            ESP_LOGI(TAG, "    Violet:      %f", _received_data.violet);
            ESP_LOGI(TAG, "    Blue:        %f", _received_data.blue);
            ESP_LOGI(TAG, "    Green:       %f", _received_data.green);
            ESP_LOGI(TAG, "    Yellow:      %f", _received_data.yellow);
            ESP_LOGI(TAG, "    Orange:      %f", _received_data.orange);
            ESP_LOGI(TAG, "    Red:         %f", _received_data.red);

            //* Sends received data to cloud 
            send_data_to_cloud(client, _received_data);
            // update_display_data(_own_sensor_data.temperature, _own_sensor_data.tVOC, _own_sensor_data.eCO2);
            vTaskDelay(2000 / portTICK_RATE_MS);
        }

        /**< Updates data from sensor readings */
        _own_sensor_data.eCO2 = sgp30_main_sensor.eCO2;
        _own_sensor_data.tVOC = sgp30_main_sensor.TVOC;
        _own_sensor_data.temperature = comp_data.temperature;
        _own_sensor_data.humidity = comp_data.humidity;

        _own_sensor_data.red = as7262_main_sensor.calibrated_values[AS726x_RED];
        _own_sensor_data.orange = as7262_main_sensor.calibrated_values[AS726x_ORANGE];
        _own_sensor_data.yellow = as7262_main_sensor.calibrated_values[AS726x_YELLOW];
        _own_sensor_data.green = as7262_main_sensor.calibrated_values[AS726x_GREEN];
        _own_sensor_data.blue = as7262_main_sensor.calibrated_values[AS726x_BLUE];
        _own_sensor_data.violet = as7262_main_sensor.calibrated_values[AS726x_VIOLET];


        //* Sends sensor data to cloud 
        send_data_to_cloud(client, _own_sensor_data);
        
        update_display_data(_own_sensor_data.temperature, _own_sensor_data.tVOC, _own_sensor_data.eCO2);
        
        vTaskDelay(7000 / portTICK_RATE_MS);
    }
}



void node_device_task(void *arg) {

    model_sensor_data_t device_data = {
        .device_name = CONFIG_DEVICE_ID,
    };

    while (1) {
        device_data.eCO2 = sgp30_main_sensor.eCO2;
        device_data.tVOC = sgp30_main_sensor.TVOC;

        device_data.temperature = comp_data.temperature;
        device_data.humidity = comp_data.humidity;
        device_data.pressure = comp_data.pressure;

        device_data.noise_level = 40;

        device_data.red = as7262_main_sensor.calibrated_values[AS726x_RED];
        device_data.orange = as7262_main_sensor.calibrated_values[AS726x_ORANGE];
        device_data.yellow = as7262_main_sensor.calibrated_values[AS726x_YELLOW];
        device_data.green = as7262_main_sensor.calibrated_values[AS726x_GREEN];
        device_data.blue = as7262_main_sensor.calibrated_values[AS726x_BLUE];
        device_data.violet = as7262_main_sensor.calibrated_values[AS726x_VIOLET];

        ble_mesh_custom_sensor_client_model_message_set(device_data);
        
        //TODO: Change task loop timing 
        vTaskDelay(10000 / portTICK_RATE_MS);
    }

}
