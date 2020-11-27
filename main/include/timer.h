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

#define TIMER_DIVIDER           16  //  Hardware timer clock divider 
#define TIMER_SCALE             (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds -> 5MHZ or 200ns
#define SENSOR_ID               0
#define TIMEOUT_ID              1
#define SENSOR_INTERVAL_SEC     (20.0)   // sample test interval for the second timer
#define FEEDBACK_INTERVAL_SEC   (200.0)
#define TIMEOUT_INTERVAL_SEC    (62.0) // sample test interval for the first timer
#define MAX_FEEDBACK_COUNT      (int)(FEEDBACK_INTERVAL_SEC/SENSOR_INTERVAL_SEC)

/**
 * @brief structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int timer_group;
    int timer_idx;
    bool feedback_flag;
} timer_event_t;

extern xQueueHandle sensor_timer_queue;
extern xQueueHandle feedback_timer_queue;

void tg0_timer_init(int timer_idx, double timer_interval_sec);

#endif 