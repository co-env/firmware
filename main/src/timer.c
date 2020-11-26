#include "timer.h"

xQueueHandle sensor_timer_queue = NULL;
xQueueHandle feedback_timer_queue = NULL;


/**
 ** Timer group0 ISR handler
 *
 * @param para - pointer to timer_idx 
 * 
 *? Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group0_isr(void *para) {
    timer_spinlock_take(TIMER_GROUP_0);
    
    int timer_idx = (int) para;
    static uint8_t feedback_counter = 0;

    /* Retrieve the interrupt status(?) and the counter value
       from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = TIMER_GROUP_0;
    evt.timer_idx = timer_idx;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0) { //SENSOR
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
        
        if(++feedback_counter == MAX_FEEDBACK_COUNT){
            evt.feedback_flag = true;
            feedback_counter = 0;
        } else {
            evt.feedback_flag = false;
        }
    } else if (timer_intr & TIMER_INTR_T1) { //TIMEOUT
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    if(timer_idx == SENSOR_ID){
        xQueueSendFromISR(sensor_timer_queue, &evt, NULL);
        if(evt.feedback_flag){
            xQueueSendFromISR(feedback_timer_queue, &evt, NULL);
        }
    } else {
        xQueueSendFromISR(feedback_timer_queue, &evt, NULL);
    }

    timer_spinlock_give(TIMER_GROUP_0);
}

/**
 * @brief Initialize selected timer of the timer group 0
 *
 * @param timer_idx - the timer number to initialize
 * @param timer_interval_sec - the interval of alarm to set
 */
void tg0_timer_init(int timer_idx, double timer_interval_sec) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE, // only starts when timer_start() is called
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
                       (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}