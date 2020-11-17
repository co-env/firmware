#include "network.h"

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

/**< MQTT main task */
void mqtt_app_start(void *arg) { 
//TODO include payloads as arguments?
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