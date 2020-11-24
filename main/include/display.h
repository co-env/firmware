#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <stdio.h>

#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"

void temp_question_screen(void);
void temp_descr_question_screen(void);
void sound_question_screen(void);
void light_question_screen(void);
void light_descr_question_screen(void);
void off_screen(void);

void display_start(void);
void update_display_data(uint32_t temperature, uint16_t tvoc, uint16_t eco2);

#endif  // __DISPLAY_H__