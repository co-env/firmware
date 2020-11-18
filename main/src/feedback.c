/**
 * Copyright (c) 2017-2018 Tara Keeling
 * 
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include "feedback.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

/*** Push buttons ***/ 
xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) { //??
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void bt_config(void){
    gpio_config_t io_conf; //> generic gpio config struct
    
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; //bit mask of the pins, use GPIO4/5 here
    io_conf.intr_type = GPIO_INTR_POSEDGE; //interrupt of rising edge //? acho que tanto faz
    io_conf.mode = GPIO_MODE_INPUT; //set as input mode    
    io_conf.pull_up_en = 0; //pins 34-39 do not have internal pull up/down  ////enable pull-up mode

    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //install gpio isr service
    gpio_isr_handler_add(BUTTON_0, gpio_isr_handler, (void*) BUTTON_0); //hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_1, gpio_isr_handler, (void*) BUTTON_1); //hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_2, gpio_isr_handler, (void*) BUTTON_2); //hook isr handler for specific gpio pin
}

void gpio_task_example(void* arg) {
    uint32_t io_num;

    bt_config();

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

/****
 * MAIN
 * gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); 
 * xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL); //start gpio task
 */

/*** Display OLED ***/
static const char* TAG = "DISPLAY-EXAMPLE";

static const int I2CDisplayAddress = 0x3C;
static const int I2CDisplayWidth = 128;
static const int I2CDisplayHeight = 64;
static const int I2CResetPin = -1;

struct SSD1306_Device Display;

const struct SSD1306_FontDef* FontList[ ] = {
    &Font_droid_sans_fallback_11x13,
    &Font_droid_sans_fallback_15x17,
    &Font_droid_sans_fallback_24x28,
    &Font_droid_sans_mono_7x13,
    &Font_droid_sans_mono_13x24,
    &Font_droid_sans_mono_16x31,
    &Font_liberation_mono_9x15,
    &Font_liberation_mono_13x21,
    &Font_liberation_mono_17x30,
    NULL
};


static bool SSD1306_i2c_bus_init(struct SSD1306_Device *Device) {
    assert(SSD1306_I2CMasterInitDefault() == true);
    assert(SSD1306_I2CMasterAttachDisplayDefault(Device, I2CDisplayWidth, I2CDisplayHeight, I2CDisplayAddress, I2CResetPin) == true);

    return true;
}

void FontDisplayTask(void* arg) {
    // struct SSD1306_Device* Display = ( struct SSD1306_Device* ) arg;

    char temperature[20];
    char voc[20];
    char eCO2[20];

    if (SSD1306_i2c_bus_init(&Display)) {
            SSD1306_SetFont(&Display, &Font_liberation_mono_9x15);

            SSD1306_Clear(&Display, SSD_COLOR_BLACK);

            SSD1306_FontDrawAnchoredString(&Display, TextAnchor_North , "COEnv", SSD_COLOR_WHITE);


            SSD1306_SetFont(&Display, &Font_droid_sans_mono_7x13);
            
            sprintf(temperature, "Temp: 0   %c", 0xb0);
            sprintf(voc,         "TVOC: 0   ppb");
            sprintf(eCO2,        "eCO2: 400 ppm");

            SSD1306_FontDrawString(&Display, 0, 14, temperature, SSD_COLOR_WHITE);
            SSD1306_FontDrawString(&Display, strlen(temperature) * 7, 14, "C", SSD_COLOR_WHITE);

            SSD1306_FontDrawString(&Display, 0, 25, voc, SSD_COLOR_WHITE);
            
            SSD1306_FontDrawString(&Display, 0, 36, eCO2, SSD_COLOR_WHITE);
            
            SSD1306_Update(&Display);

            ESP_LOGI(TAG, "Display updated!");
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

void update_display_data(uint32_t temperature, uint16_t tvoc, uint16_t eco2) {
    char temperature_string[20];
    char voc_string[20];
    char eCO2_string[20];

    float t = temperature / 100;

    t += ((temperature % 100) * 0.01);

    SSD1306_Clear(&Display, SSD_COLOR_BLACK);

    SSD1306_SetFont(&Display, &Font_liberation_mono_9x15);
    SSD1306_FontDrawAnchoredString(&Display, TextAnchor_North , "COEnv", SSD_COLOR_WHITE);


    SSD1306_SetFont(&Display, &Font_droid_sans_mono_7x13);

    SSD1306_DrawHLine(&Display, 5, 14, 120, SSD_COLOR_WHITE);

    sprintf(temperature_string, "Temp: %2.2f %c", t, 0xb0);
    sprintf(voc_string,         "TVOC: %5d ppb", tvoc);
    sprintf(eCO2_string,        "eCO2: %5d ppm", eco2);

    SSD1306_FontDrawString(&Display, 0, 18, temperature_string, SSD_COLOR_WHITE);
    SSD1306_FontDrawString(&Display, strlen(temperature_string) * 7, 16, "C", SSD_COLOR_WHITE);

    SSD1306_FontDrawString(&Display, 0, 30, voc_string, SSD_COLOR_WHITE);
    
    SSD1306_FontDrawString(&Display, 0, 42, eCO2_string, SSD_COLOR_WHITE);
    
    SSD1306_Update(&Display);

    ESP_LOGI(TAG, "Display updated!");
}
