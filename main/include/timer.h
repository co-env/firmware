#ifndef TIMER_H
#define TIMER_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "esp_types.h"

#include "driver/timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define TIMER_DIVIDER         16  //  Hardware timer clock divider 
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds -> 5MHZ or 200ns
#define FEEDBACK_ID           0
#define TIMEOUT_ID            1
#define FEEDBACK_INTERVAL_SEC   (20.0)   // sample test interval for the second timer
#define TIMEOUT_INTERVAL_SEC   (62.0) // sample test interval for the first timer
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

void tg0_timer_init(int timer_idx, double timer_interval_sec);

#endif 