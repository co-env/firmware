/**
 * Copyright (c) 2017-2018 Tara Keeling
 * 
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include "feedback.h"

/*** Timer ***/
xQueueHandle timer_queue = NULL;

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
    if (timer_intr & TIMER_INTR_T0) {
        evt.type = TEST_WITH_RELOAD; 
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    } else if (timer_intr & TIMER_INTR_T1) {
        evt.type = TEST_WITH_RELOAD; 
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    } else {
        evt.type = -1; // not supported event type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);

    timer_spinlock_give(TIMER_GROUP_0);
}

/**
 * @brief Initialize selected timer of the timer group 0
 *
 * @param timer_idx - the timer number to initialize
 * @param timer_interval_sec - the interval of alarm to set
 */
static void tg0_timer_init(int timer_idx, double timer_interval_sec) {
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
    io_conf.pull_down_en = 0;

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
 * GPIO IN MAIN
 * gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); 
 * xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL); //start gpio task
 */

/*** Máquina de Estados ***/

// tabela relacionando os estados às suas execuções
state_func_t* const state_table[ NUM_STATES ] = {
    do_state_initial, do_state_temp, do_state_temp_descr,
    do_state_sound, do_state_light, do_state_light_descr, do_state_final
};


state_t do_state_initial(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->temp_comf=0; answer_data->high_temp=0; answer_data->sound_comf=0; answer_data->light_comf=0;  answer_data->lightness=0;
    printf("Inital Answers:%d%d%d%d%d\n",answer_data->temp_comf,answer_data->high_temp,answer_data->sound_comf,answer_data->light_comf, answer_data->lightness);
    
    // temp_question_screen();
    return STATE_TEMP;
}

state_t do_state_temp(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->temp_comf = (io_num == BUTTON_1)? true: false;
    if(io_num == BUTTON_0){
        temp_descr_question_screen();
        return STATE_TEMP_DESCR;
    }
    else {
        sound_question_screen();
        return STATE_SOUND;
    }
}

state_t do_state_temp_descr(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->high_temp = (io_num == BUTTON_1)? true: false;
    
    sound_question_screen();
    return STATE_SOUND;
}

state_t do_state_sound(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->sound_comf = (io_num == BUTTON_1)? true: false;
    light_question_screen();
    return STATE_LIGHT;
}

state_t do_state_light(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->light_comf = (io_num == BUTTON_1)? true: false;
    if(io_num == BUTTON_0){
        light_descr_question_screen();
        return STATE_LIGHT_DESCR;
    }
    else {
        off_screen();
        return STATE_FINAL;
    }
}

state_t do_state_light_descr(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->lightness = (io_num == BUTTON_1)? true: false;
    off_screen();
    return STATE_FINAL;
}

state_t do_state_final(uint32_t io_num, feedback_answers_t *answer_data){
    printf("Final Answers:%d%d%d%d%d\n",answer_data->temp_comf,answer_data->high_temp,answer_data->sound_comf,answer_data->light_comf, answer_data->lightness);
    off_screen();
    return STATE_FINAL;
}


//função genérica para chamar a tabela de estados
state_t run_state(state_t cur_state, uint32_t io_num, feedback_answers_t *answer_data) {
    printf("State: %d\n",cur_state);
    return state_table[cur_state](io_num, answer_data);
};

void feedback_task(void* arg) {
    state_t cur_state = STATE_INITIAL;
    feedback_answers_t answer_data; 
    uint32_t io_num = 0; //?
    timer_event_t evt;
    
    //* Init Timers
    tg0_timer_init(FEEDBACK_ID, FEEDBACK_INTERVAL_SEC); 
    
    bt_config();
    // off_screen();

    while(1) {
        //receive a timer interrupt
        if(xQueueReceive(timer_queue, &evt, 100)){
            printf("Group[%d], timer[%d] alarm event\n", evt.timer_group, evt.timer_idx);
            if(evt.timer_idx == FEEDBACK_ID){
                cur_state = run_state(STATE_INITIAL, io_num, &answer_data);
                tg0_timer_init(TIMEOUT_ID, TIMEOUT_INTERVAL_SEC); 
            }
            else if(evt.timer_idx == TIMEOUT_ID){
                cur_state = run_state(STATE_FINAL, io_num, &answer_data);
                (void)timer_pause(0,TIMEOUT_ID);
                //? timer deinit
            }
        }

        //receive a button interrupt
        if(xQueueReceive(gpio_evt_queue, &io_num, 100)){
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            (void)timer_pause(0,TIMEOUT_ID);
            cur_state = run_state(cur_state, io_num, &answer_data);
            tg0_timer_init(TIMEOUT_ID, TIMEOUT_INTERVAL_SEC); 
        }


        // do other program logic, run other state machines, etc
    }
}
