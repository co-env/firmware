#include "sensors.h"
#include "rtos_sync.h"

static const char *TAG = "SENSORS";

static adc_channel_t mic_channel = ADC_CHANNEL_6;

SemaphoreHandle_t xSemaphore = NULL;
// EventGroupHandle_t sensorsEventGroup = NULL;

/******** I2C ********/
i2c_port_t i2c_num = I2C_MASTER_NUM;

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

/****** Sensores *******/

/**< AS7262 main task */
void color_sensor_task(void *arg) {
    ESP_LOGI(TAG, "Color sensor task init");
    // i2c_master_init();
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE ) {
        as7262_init(&as7262_main_sensor, (as7262_read_fptr_t)main_i2c_read, (as7262_write_fptr_t)main_i2c_write);
        xSemaphoreGive(xSemaphore);
    }
    
    vTaskDelay(1000 / portTICK_RATE_MS);

    //! Integration time:  0x3C * 2.8ms = 168ms
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        as7262_set_integration_time(&as7262_main_sensor, 0x3C); 
        xSemaphoreGive(xSemaphore);
    }

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
        as7262_set_gain(&as7262_main_sensor, GAIN_16X);
        xSemaphoreGive(xSemaphore);
    }

    double total_light = 0;
    double lux_factor = 0.1464128843338;

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
                // ESP_LOGI(TAG, " ------------------ ");

                total_light = 0;
                for (int i = AS726x_VIOLET; i <= AS726x_RED; i++) {
                    total_light += as7262_main_sensor.calibrated_values[i];
                }
                total_light /= 45 * lux_factor;
                ESP_LOGI(TAG, " LUX:     %f", total_light);
                ESP_LOGI(TAG, " ------------------ ");
                
                xEventGroupSetBits(sensorsEventGroup, EVT_GRP_COLOR_SENSOR_COMPLETE);
                vTaskSuspend(NULL);
            } else {
                xSemaphoreGive(xSemaphore);
            }
        }

        // vTaskDelay(5000 / portTICK_RATE_MS);
    }
}

void air_sensor_task(void *arg) {
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
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            sgp30_IAQ_measure(&sgp30_main_sensor);
            xSemaphoreGive(xSemaphore);
        }

        ESP_LOGI(TAG, "TVOC: %d,  eCO2: %d",  sgp30_main_sensor.TVOC, sgp30_main_sensor.eCO2);

        // vTaskDelay(1000 / portTICK_RATE_MS);
        xEventGroupSetBits(sensorsEventGroup, EVT_GRP_AIR_SENSOR_COMPLETE);
        vTaskSuspend(NULL);
    }
}

void bme280_sensor_task(void *arg) {
    ESP_LOGI(TAG, "SGP30 main task initializing...");
    esp_err_t erro = ESP_OK;


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

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            erro = bme280_meas_forcedmode(&comp_data);
            xSemaphoreGive(xSemaphore);

            ESP_LOGI(TAG, "Temperature: %f",comp_data.temperature);
            ESP_LOGI(TAG, "Pressure: %f",comp_data.pressure);
            ESP_LOGI(TAG, "Humidity: %f",comp_data.humidity);

        }
        if(erro != BME280_OK){    
            printf("Could not measure :(");
        }
        
        // update_display_data(comp_data.temperature, sgp30_main_sensor.TVOC, sgp30_main_sensor.eCO2);
        // vTaskDelay(1000 / portTICK_RATE_MS);
        xEventGroupSetBits(sensorsEventGroup, EVT_GRP_TEMP_SENSOR_COMPLETE);
        vTaskSuspend(NULL);
    }
}


void sound_sensor_task(void *arg) {
    ESP_LOGI(TAG, "Mic main task initializing...");

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        adc1_config(mic_channel);
        xSemaphoreGive(xSemaphore);
    }

    while(1) {
        // (void)get_voltage_variation(mic_channel);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        mic_noise_level = get_noise_level_db(mic_channel);
        ESP_LOGI(TAG, "Noise:  %f", mic_noise_level);
        xEventGroupSetBits(sensorsEventGroup, EVT_GRP_SOUND_SENSOR_COMPLETE);
        vTaskSuspend(NULL);
    }
}

void main_sensor_task(void *arg){
    timer_event_t evt;
    TaskHandle_t color_sensor_th, air_sensor_th, bme_sensor_th, sound_sensor_th;

    //* Init all sensors functions
    xTaskCreate(color_sensor_task, "color_sensor_main_task", 1024 * 2, (void *)0, 20, &color_sensor_th);
    xTaskCreate(air_sensor_task, "air_sensor_main_task", 1024 * 2, (void *)0, 15, &air_sensor_th);
    xTaskCreate(bme280_sensor_task, "bme280_sensor_main_task", 1024 * 2, (void *)0, 15, &bme_sensor_th);
    xTaskCreate(sound_sensor_task, "sound_sensor_main_task", 1024 * 2, (void *)0, 15, &sound_sensor_th);

    while(1) {
        if(xQueueReceive(sensor_timer_queue,&evt,100)){
            printf("Resume tasks:\n");
            //Resume tasks in sensor timeout
            vTaskResume(color_sensor_th);
            vTaskResume(air_sensor_th);
            vTaskResume(bme_sensor_th);
            vTaskResume(sound_sensor_th);
        }
    }
}
 
/*@*/