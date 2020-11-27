/**
 * @file feedback.c
 * 
 * @brief
 * 
 * @author
 * 
 * @date  11/2020
 */

#include "feedback.h"
#include "rtos_sync.h"

static const char *TAG = "FEEDBACK";

feedback_answers_t answer_data = { 0 };

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

/** Execução dos Estados **/

/**
 * @brief Estado Inicial
 * Zera as respostas. Liga o display, faz a primeira pergunta (Temperatura)
 * @return novo estado = STATE_TEMP (salva resposta de conforto de temperatura)
 */
state_t do_state_initial(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->temp_comf=0; answer_data->high_temp=0; answer_data->sound_comf=0; answer_data->light_comf=0;  answer_data->lightness=0; answer_data->new_answer=false;
    ESP_LOGI(TAG, "Inital Answers:%d%d%d%d%d",answer_data->temp_comf,answer_data->high_temp,answer_data->sound_comf,answer_data->light_comf, answer_data->lightness);
    on_screen();
    tg0_timer_init(TIMEOUT_ID, TIMEOUT_INTERVAL_SEC); 
    temp_question_screen();
    return STATE_TEMP;
}

/**
 * @brief Estado Temperatura. 
 * Salva a resposta sobre o conforto térmico
 * Em caso de desconforto (Button 0), pergunta detalhes. 
 * Se estiver confortável (Button 2), pergunta sobre conforto acústico.
 */
state_t do_state_temp(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->temp_comf = (io_num == BUTTON_2)? true: false;
    tg0_timer_init(TIMEOUT_ID, TIMEOUT_INTERVAL_SEC); 
    if(io_num == BUTTON_0){ 
        temp_descr_question_screen();
        return STATE_TEMP_DESCR;
    }
    else {
        sound_question_screen();
        return STATE_SOUND;
    }
}

/**
 * @brief Estado Descrição da Temperatura
 * Salva a informação sobre a temperatura. Mostra a pergunta sobre conforto acústico
 * @return STATE_SOUND 
 */
state_t do_state_temp_descr(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->high_temp = (io_num == BUTTON_2)? true: false;
    tg0_timer_init(TIMEOUT_ID, TIMEOUT_INTERVAL_SEC); 
    sound_question_screen();
    
    return STATE_SOUND;
}

/**
 * @brief Estado Som
 * Salva as informações sobre conforto acústico.
 * Mostra a tela com pergunta sobre conforto luminoso
 */
state_t do_state_sound(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->sound_comf = (io_num == BUTTON_2)? true: false;
    tg0_timer_init(TIMEOUT_ID, TIMEOUT_INTERVAL_SEC); 
    light_question_screen();
    return STATE_LIGHT;
}

/**
 * @brief Estado Luz
 * Salva as informações sobre conforto luminoso.
 * Em caso de desconforto (Button 0), pergunta detalhes. 
 * Se estiver confortável (Button 2), mostra a tela de agradecimento e encerra o feedback.
 */
state_t do_state_light(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->light_comf = (io_num == BUTTON_2)? true: false;
    tg0_timer_init(TIMEOUT_ID, TIMEOUT_INTERVAL_SEC); 
    if(io_num == BUTTON_0){
        light_descr_question_screen();
        return STATE_LIGHT_DESCR;
    }
    else {
        answer_data->new_answer = true;  // Reached the final state, set NEW_ANSWER flag to TRUE
        xEventGroupSetBits(sensorsEventGroup, EVT_GRP_FEEDBACK_COMPLETE);
        (void)timer_pause(0,TIMEOUT_ID);
        thankyou_screen();
        vTaskDelay(1000 / portTICK_RATE_MS);
        off_screen();
        return STATE_FINAL;
    }
}

/**
 * @brief Estado Descrição Luz
 * Salva as informações sobre conforto luminoso.
 * Mostra a tela de agradecimento e encerra o feedback.
 */
state_t do_state_light_descr(uint32_t io_num, feedback_answers_t *answer_data){
    answer_data->lightness = (io_num == BUTTON_2)? true : false;
    answer_data->new_answer = true;  // Reached the final state, set NEW_ANSWER flag to TRUE
    
    (void)timer_pause(0,TIMEOUT_ID);
    xEventGroupSetBits(sensorsEventGroup, EVT_GRP_FEEDBACK_COMPLETE);
    
    thankyou_screen();
    vTaskDelay(1000 / portTICK_RATE_MS);
    off_screen();

    return STATE_FINAL;
}

/**
 * @brief Estado Final
 * Caso o botão seja apertado após o fim da coleta de feedback, ou o tempo limite para responder seja atingido. 
 */
state_t do_state_final(uint32_t io_num, feedback_answers_t *answer_data){
    ESP_LOGI(TAG, "Final Answers: %d%d%d%d%d", answer_data->temp_comf,answer_data->high_temp,answer_data->sound_comf,answer_data->light_comf, answer_data->lightness);
    (void)timer_pause(0,TIMEOUT_ID);
    off_screen();
    return STATE_FINAL;
}


//função genérica para chamar a tabela de estados
state_t run_state(state_t cur_state, uint32_t io_num, feedback_answers_t *answer_data) {
    ESP_LOGI(TAG, "Current state: %d", cur_state);
    return state_table[cur_state](io_num, answer_data);
};

void feedback_task(void* arg) {
    state_t cur_state = STATE_INITIAL;
    // feedback_answers_t answer_data; 
    uint32_t io_num = 0; //?
    timer_event_t evt;
    
    //* Init Timers
    // SSD1306_i2c_bus_init(); //-> na main pra usar em outra task
    bt_config();

    while(1) {
        //receive a timer interrupt
        if(xQueueReceive(feedback_timer_queue, &evt, 100)){
            ESP_LOGI(TAG, "Group[%d], timer[%d] alarm event", evt.timer_group, evt.timer_idx);
            if(evt.timer_idx == SENSOR_ID){
                xEventGroupSetBits(sensorsEventGroup, EVT_GRP_FEEDBACK_TIME);
                cur_state = run_state(STATE_INITIAL, io_num, &answer_data);
            }
            else if(evt.timer_idx == TIMEOUT_ID){
                cur_state = run_state(STATE_FINAL, io_num, &answer_data);
                xEventGroupSetBits(sensorsEventGroup, EVT_GRP_FEEDBACK_COMPLETE);
            }
        }

        //receive a button interrupt
        if(xQueueReceive(gpio_evt_queue, &io_num, 100)){
            ESP_LOGI(TAG, "GPIO[%d] event received, value: %d", io_num, gpio_get_level(io_num));
            cur_state = run_state(cur_state, io_num, &answer_data);
        }

    }
}
