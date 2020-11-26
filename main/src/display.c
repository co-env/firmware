#include "display.h"

/*** Display OLED ***/
static const char* TAG = "DISPLAY-EXAMPLE";

static const int I2CDisplayAddress = 0x3C;
static const int I2CDisplayWidth = 128;
static const int I2CDisplayHeight = 64;
static const int I2CResetPin = -1;

struct SSD1306_Device Display;

const struct SSD1306_FontDef* FontList[ ] = {
    &Font_droid_sans_fallback_11x13,
    &Font_droid_sans_fallback_15x17,
    &Font_droid_sans_fallback_24x28,
    &Font_droid_sans_mono_7x13,
    &Font_droid_sans_mono_13x24,
    &Font_droid_sans_mono_16x31,
    &Font_liberation_mono_9x15,
    &Font_liberation_mono_13x21,    &Font_liberation_mono_17x30,
    NULL
};

/** Inicializa I2C para o display (chamar apenas umas vez -> SETUP TASK FEEDBACK) */
bool SSD1306_i2c_bus_init() {
    assert(SSD1306_I2CMasterInitDefault() == true);
    assert(SSD1306_I2CMasterAttachDisplayDefault(&Display, I2CDisplayWidth, I2CDisplayHeight, I2CDisplayAddress, I2CResetPin) == true);

    return true;
}

/******** TELAS ********/ 
void display_start(void) { //!

    char temperature[20];
    char voc[20];
    char eCO2[20];

    SSD1306_Clear(&Display, SSD_COLOR_BLACK);
    SSD1306_SetFont(&Display, &Font_liberation_mono_9x15);
    SSD1306_FontDrawAnchoredString(&Display, TextAnchor_North , "COEnv", SSD_COLOR_WHITE);

    SSD1306_SetFont(&Display, &Font_droid_sans_mono_7x13);
    
    sprintf(temperature, "Temp: 0   %c", 0xb0);
    sprintf(voc,         "TVOC: 0   ppb");
    sprintf(eCO2,        "eCO2: 400 ppm");
    SSD1306_FontDrawString(&Display, 0, 14, temperature, SSD_COLOR_WHITE);
    SSD1306_FontDrawString(&Display, strlen(temperature) * 7, 14, "C", SSD_COLOR_WHITE);

    SSD1306_FontDrawString(&Display, 0, 25, voc, SSD_COLOR_WHITE);
    SSD1306_FontDrawString(&Display, 0, 36, eCO2, SSD_COLOR_WHITE);
    
    SSD1306_Update(&Display);
}

//todo: Telas de perguntas - feedback
void temp_question_screen(void) {

    SSD1306_Clear(&Display, SSD_COLOR_BLACK);
    SSD1306_SetFont(&Display, &Font_liberation_mono_9x15);
    SSD1306_FontDrawAnchoredString(&Display, TextAnchor_North , "COEnv", SSD_COLOR_WHITE);
    SSD1306_DrawHLine(&Display,5,14,120,SSD_COLOR_WHITE);

    SSD1306_SetFont(&Display, &Font_droid_sans_mono_7x13);
    
    SSD1306_FontDrawString(&Display, 0, 25, "Est\xE1 confort\xE1vel", SSD_COLOR_WHITE);
    SSD1306_FontDrawString(&Display, 0, 25, "com a temperatura?", SSD_COLOR_WHITE);

    SSD1306_DrawBox(&Display,TextAnchor_SouthWest, "Sim", SSD_COLOR_BLACK);
    SSD1306_DrawBox(&Display,TextAnchor_SouthEast, "N\xE3o", SSD_COLOR_WHITE);
}

void temp_descr_question_screen(void){}
void sound_question_screen(void){}
void light_question_screen(void){}
void light_descr_question_screen(void){}

void off_screen(void){
    SSD1306_Clear(&Display, SSD_COLOR_BLACK);
}

void update_display_data(uint32_t temperature, uint16_t tvoc, uint16_t eco2) {
    char temperature_string[20];
    char voc_string[20];
    char eCO2_string[20];

    float t = temperature / 100;

    t += ((temperature % 100) * 0.01);

    SSD1306_Clear(&Display, SSD_COLOR_BLACK);

    SSD1306_SetFont(&Display, &Font_liberation_mono_9x15);
    SSD1306_FontDrawAnchoredString(&Display, TextAnchor_North , "COEnv", SSD_COLOR_WHITE);


    SSD1306_SetFont(&Display, &Font_droid_sans_mono_7x13);

    SSD1306_DrawHLine(&Display, 5, 14, 120, SSD_COLOR_WHITE);

    sprintf(temperature_string, "Temp: %2.2f %c", t, 0xb0);
    sprintf(voc_string,         "TVOC: %5d ppb", tvoc);
    sprintf(eCO2_string,        "eCO2: %5d ppm", eco2);

    SSD1306_FontDrawString(&Display, 0, 18, temperature_string, SSD_COLOR_WHITE);
    SSD1306_FontDrawString(&Display, strlen(temperature_string) * 7, 16, "C", SSD_COLOR_WHITE);

    SSD1306_FontDrawString(&Display, 0, 30, voc_string, SSD_COLOR_WHITE);
    
    SSD1306_FontDrawString(&Display, 0, 42, eCO2_string, SSD_COLOR_WHITE);
    
    SSD1306_Update(&Display);

    ESP_LOGI(TAG, "Display updated!");
}