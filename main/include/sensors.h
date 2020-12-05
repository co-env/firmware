#ifndef SENSORS_H
#define SENSORS_H

#include <stdio.h>
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "timer.h"
#include "SGP30.h"
#include "AS7262.h"
#include "esp32_bme280.h"
#include "microfone.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */


as7262_dev_t as7262_main_sensor;
sgp30_dev_t sgp30_main_sensor;
struct bme280_data comp_data; 
float mic_noise_level;

extern SemaphoreHandle_t xSemaphore;


/** I2C **/ 
esp_err_t i2c_master_driver_initialize(void);

/** TASKS **/
// void color_sensor_task(void *arg);
// void air_sensor_task(void *arg);
// void bme280_sensor_task(void *arg);
// void sound_sensor_task(void *arg);
void main_sensor_task(void *arg);

#endif // SENSORS_H
/*@*/