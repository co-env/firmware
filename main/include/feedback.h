#ifndef __DISPLAY_TASK_H__
#define __DISPLAY_TASK_H__

#include <stdio.h>

#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#define BUTTON_0     35
#define BUTTON_1     32
#define BUTTON_2     33
#define GPIO_INPUT_PIN_SEL  ((1ULL<<BUTTON_0) | (1ULL<<BUTTON_1) | (1ULL<<BUTTON_2))
#define ESP_INTR_FLAG_DEFAULT 0 //??

extern xQueueHandle gpio_evt_queue;

void gpio_task_example(void* arg);

void FontDisplayTask(void* arg);
void update_display_data(uint32_t temperature, uint16_t tvoc, uint16_t eco2);

#endif  // __DISPLAY_TASK_H__