#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "esp_types.h"

#include "display.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

/*** Timer ***/
#define TIMER_DIVIDER         16  //  Hardware timer clock divider 
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds -> 5MHZ or 200ns
#define FEEDBACK_ID           0
#define TIMEOUT_ID            1
#define FEEDBACK_INTERVAL_SEC   (30.0)   // sample test interval for the second timer
#define TIMEOUT_INTERVAL_SEC   (60.0) // sample test interval for the first timer
// #define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

/**
 * @brief structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
} timer_event_t;

extern xQueueHandle timer_queue;


/*** Push buttons ***/ 
#define BUTTON_0     35
#define BUTTON_1     32
#define BUTTON_2     33
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
} feedback_answers_t; 

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