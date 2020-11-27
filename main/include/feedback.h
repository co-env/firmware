/**
 * @file feedback.h
 * 
 * @brief
 * 
 * @author
 * 
 * @date  11/2020
 */


#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "esp_types.h"

#include "display.h"
#include "timer.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"


/*** Push buttons ***/ 
#define BUTTON_0     35 //Não
#define BUTTON_1     32 
#define BUTTON_2     33 //Sim
#define GPIO_INPUT_PIN_SEL  ((1ULL<<BUTTON_0) | (1ULL<<BUTTON_1) | (1ULL<<BUTTON_2))
#define ESP_INTR_FLAG_DEFAULT 0 //??

extern xQueueHandle gpio_evt_queue;
void gpio_task_example(void* arg);

/*** Máquina de Estados ***/
typedef enum { 
    STATE_INITIAL, 
    STATE_TEMP, 
    STATE_TEMP_DESCR, 
    STATE_SOUND, 
    STATE_LIGHT, 
    STATE_LIGHT_DESCR, 
    STATE_FINAL,
    NUM_STATES 
} state_t;

typedef struct {
    bool temp_comf;
    bool high_temp; 
    bool sound_comf;
    bool light_comf;
    bool lightness;
    bool new_answer;
} feedback_answers_t;

extern feedback_answers_t answer_data;

typedef state_t state_func_t(uint32_t io_num, feedback_answers_t *answer_data);

state_t do_state_initial(uint32_t io_num, feedback_answers_t *answer_data); 
state_t do_state_temp(uint32_t io_num, feedback_answers_t *answer_data); 
state_t do_state_temp_descr(uint32_t io_num, feedback_answers_t *answer_data); 
state_t do_state_sound(uint32_t io_num, feedback_answers_t *answer_data); 
state_t do_state_light(uint32_t io_num, feedback_answers_t *answer_data); 
state_t do_state_light_descr(uint32_t io_num, feedback_answers_t *answer_data); 
state_t do_state_final(uint32_t io_num, feedback_answers_t *answer_data);

void feedback_task(void* arg);

#endif // FEEDBACK_H