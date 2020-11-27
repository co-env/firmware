#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

bool SSD1306_i2c_bus_init(void); //initialized in feedback task

void temp_question_screen(void);
void temp_descr_question_screen(void);
void sound_question_screen(void);

void light_question_screen(void);
void light_descr_question_screen(void);
void off_screen(void);
void on_screen(void);
void thankyou_screen(void);

void splash_screen(uint8_t i);

void display_start(void);
void update_display_data(uint32_t temperature, uint16_t tvoc, uint16_t eco2);

#endif  // __DISPLAY_H__