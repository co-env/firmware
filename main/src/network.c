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

#include "feedback.h"
#include "rtos_sync.h"

// #include "display_task.h"

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

    ESP_LOGI(TAG, "%s - sending data...", __FUNCTION__);
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
    double lux = sensor_data.red + sensor_data.orange + sensor_data.yellow + sensor_data.green + sensor_data.blue +  sensor_data.violet;
    lux /= 45 * 0.1464128843338;  //! counts -> uW/cm2 -> lux

    memset(payload, 0, strlen(payload));
    sprintf(payload, "%s,sensor=\"Color\" v=%.2f,b=%.2f,g=%.2f,y=%.2f,o=%.2f,r=%.2f,lux=%.2f", sensor_data.device_name,
                sensor_data.violet, sensor_data.blue, sensor_data.green, sensor_data.yellow, sensor_data.orange, sensor_data.red, lux);
    err = esp_mqtt_client_publish(mqtt_client, mqtt_topic, payload, 0, 0, 0);
    if (err) {
        return err;
    }

    /** Microphone Payload */
    memset(payload, 0, strlen(payload));
    sprintf(payload, "%s,sensor=\"Noise\" noise=%u", sensor_data.device_name, sensor_data.noise_level);
    err = esp_mqtt_client_publish(mqtt_client, mqtt_topic, payload, 0, 0, 0);

    /** Check to see if there's new feedback content */
    if ((sensor_data.feedback >> 7) & 0x01) {
        //!! Parse feedback content and send it here !!!!
        ESP_LOGW(TAG, "%s - Sending feedback data to cloud", __FUNCTION__);
    }

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
    ble_mesh_received_data_queue = xQueueCreate(15, sizeof(model_sensor_data_t));
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
            ESP_LOGI(TAG, "    Feedback:    %d %d %d %d %d %d %d %d",
                    (_received_data.feedback >> 7) & 0x01, (_received_data.feedback >> 6) & 0x01, 
                    (_received_data.feedback >> 5) & 0x01, (_received_data.feedback >> 4) & 0x01, 
                    (_received_data.feedback >> 3) & 0x01, (_received_data.feedback >> 2) & 0x01, 
                    (_received_data.feedback >> 1) & 0x01, (_received_data.feedback >> 0) & 0x01 );

            //* Sends received data to cloud 
            send_data_to_cloud(client, _received_data);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }

        /**< Updates data from sensor readings */
        if ((xEventGroupWaitBits(sensorsEventGroup,EVT_GRP_BITS, 1, 1, 1000 / portTICK_RATE_MS) & EVT_GRP_BITS) == EVT_GRP_BITS) {
            /**< Checks if there's a feedback question routine going on */
            if (xEventGroupGetBits(sensorsEventGroup) & EVT_GRP_FEEDBACK_TIME) {
                xEventGroupWaitBits(sensorsEventGroup,EVT_GRP_FEEDBACK_COMPLETE, 1, 0, portMAX_DELAY);
                xEventGroupClearBits(sensorsEventGroup, EVT_GRP_FEEDBACK_TIME);
            }
            _own_sensor_data.eCO2 = sgp30_main_sensor.eCO2;
            _own_sensor_data.tVOC = sgp30_main_sensor.TVOC;
            _own_sensor_data.temperature = comp_data.temperature;
            _own_sensor_data.humidity = comp_data.humidity;

            _own_sensor_data.noise_level = (uint16_t)mic_noise_level;

            _own_sensor_data.red = as7262_main_sensor.calibrated_values[AS726x_RED];
            _own_sensor_data.orange = as7262_main_sensor.calibrated_values[AS726x_ORANGE];
            _own_sensor_data.yellow = as7262_main_sensor.calibrated_values[AS726x_YELLOW];
            _own_sensor_data.green = as7262_main_sensor.calibrated_values[AS726x_GREEN];
            _own_sensor_data.blue = as7262_main_sensor.calibrated_values[AS726x_BLUE];
            _own_sensor_data.violet = as7262_main_sensor.calibrated_values[AS726x_VIOLET];

            _own_sensor_data.feedback = 0x00;            // First, reset feedback data
            
            _own_sensor_data.feedback |= (answer_data.new_answer << 7); // Set NEW_DATA flag (MSB)

            _own_sensor_data.feedback |= (answer_data.temp_comf  << 4);
            _own_sensor_data.feedback |= (answer_data.high_temp  << 3);
            _own_sensor_data.feedback |= (answer_data.sound_comf << 2);
            _own_sensor_data.feedback |= (answer_data.light_comf << 1);
            _own_sensor_data.feedback |= (answer_data.lightness  << 0);  // ( << 0) kkk aaaaaaaa

            // //! Debug only
            // ESP_LOGI(TAG, "Feedback data binary: %u %u %u %u %u %u %u %u",
            // (_own_sensor_data.feedback >> 7) & 0x01, (_own_sensor_data.feedback >> 6) & 0x01, 
            // (_own_sensor_data.feedback >> 5) & 0x01, (_own_sensor_data.feedback >> 4) & 0x01, 
            // (_own_sensor_data.feedback >> 3) & 0x01, (_own_sensor_data.feedback >> 2) & 0x01, 
            // (_own_sensor_data.feedback >> 1) & 0x01, (_own_sensor_data.feedback >> 0) & 0x01 );

            //* Sends sensor data to cloud 
            send_data_to_cloud(client, _own_sensor_data);
            
            answer_data.new_answer = false;  // It will be set to true again by the feedback state machine
        }
    }
}



void node_device_task(void *arg) {

    model_sensor_data_t device_data = {
        .device_name = CONFIG_DEVICE_ID,
    };

    while (1) {
        /**< Wait for all sensor tasks complete */
        if ((xEventGroupWaitBits(sensorsEventGroup,EVT_GRP_BITS, 1, 1, portMAX_DELAY) & EVT_GRP_BITS) == EVT_GRP_BITS) {
            /**< Checks if there's a feedback question routine going on */
            if (xEventGroupGetBits(sensorsEventGroup) & EVT_GRP_FEEDBACK_TIME) {
                xEventGroupWaitBits(sensorsEventGroup,EVT_GRP_FEEDBACK_COMPLETE, 1, 0, portMAX_DELAY);
                xEventGroupClearBits(sensorsEventGroup, EVT_GRP_FEEDBACK_TIME);
            }
            device_data.eCO2 = sgp30_main_sensor.eCO2;
            device_data.tVOC = sgp30_main_sensor.TVOC;

            device_data.temperature = comp_data.temperature;
            device_data.humidity = comp_data.humidity;
            device_data.pressure = comp_data.pressure;

            device_data.noise_level = (uint16_t)mic_noise_level;

            device_data.red = as7262_main_sensor.calibrated_values[AS726x_RED];
            device_data.orange = as7262_main_sensor.calibrated_values[AS726x_ORANGE];
            device_data.yellow = as7262_main_sensor.calibrated_values[AS726x_YELLOW];
            device_data.green = as7262_main_sensor.calibrated_values[AS726x_GREEN];
            device_data.blue = as7262_main_sensor.calibrated_values[AS726x_BLUE];
            device_data.violet = as7262_main_sensor.calibrated_values[AS726x_VIOLET];

            device_data.feedback = 0x00;            // First, reset feedback data
            
            device_data.feedback |= (answer_data.new_answer << 7); // Set NEW_DATA flag (MSB)

            device_data.feedback |= (answer_data.temp_comf  << 4);
            device_data.feedback |= (answer_data.high_temp  << 3);
            device_data.feedback |= (answer_data.sound_comf << 2);
            device_data.feedback |= (answer_data.light_comf << 1);
            device_data.feedback |= (answer_data.lightness  << 0);  // ( << 0) kkk aaaaaaaa

            //! Debug only
            // ESP_LOGI(TAG, "Feedback data binary: %u %u %u %u %u %u %u %u",
            // (device_data.feedback >> 7) & 0x01, (device_data.feedback >> 6) & 0x01, 
            // (device_data.feedback >> 5) & 0x01, (device_data.feedback >> 4) & 0x01, 
            // (device_data.feedback >> 3) & 0x01, (device_data.feedback >> 2) & 0x01, 
            // (device_data.feedback >> 1) & 0x01, (device_data.feedback >> 0) & 0x01 );

            ESP_LOGW(TAG, "Dados dos sensores %s", device_data.device_name);
            ESP_LOGW(TAG, "    Temperatura: %f", device_data.temperature);
            ESP_LOGW(TAG, "    Pressao:     %f", device_data.pressure);
            ESP_LOGW(TAG, "    Umidade:     %f", device_data.humidity);
            ESP_LOGW(TAG, "    TVOC:        %d", device_data.tVOC);
            ESP_LOGW(TAG, "    eCO2:        %d", device_data.eCO2);
            ESP_LOGW(TAG, "    Ruido:       %d", device_data.noise_level);
            ESP_LOGW(TAG, "    Violet:      %f", device_data.violet);
            ESP_LOGW(TAG, "    Blue:        %f", device_data.blue);
            ESP_LOGW(TAG, "    Green:       %f", device_data.green);
            ESP_LOGW(TAG, "    Yellow:      %f", device_data.yellow);
            ESP_LOGW(TAG, "    Orange:      %f", device_data.orange);
            ESP_LOGW(TAG, "    Red:         %f", device_data.red);
            ESP_LOGW(TAG, "    Feedback:    %d %d %d %d %d %d %d %d",
                    (device_data.feedback >> 7) & 0x01, (device_data.feedback >> 6) & 0x01, 
                    (device_data.feedback >> 5) & 0x01, (device_data.feedback >> 4) & 0x01, 
                    (device_data.feedback >> 3) & 0x01, (device_data.feedback >> 2) & 0x01, 
                    (device_data.feedback >> 1) & 0x01, (device_data.feedback >> 0) & 0x01 );

            ble_mesh_custom_sensor_client_model_message_set(device_data);
            
            answer_data.new_answer = false;  // It will be set to true again by the feedback state machine
        }
        
    }

}
