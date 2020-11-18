/**
 * @file network.h
 * 
 * @brief
 * 
 * @author
 * 
 * @date  11/2020
 */

#ifndef NETWORK_H
#define NETWORK_H

#include <stdio.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "mqtt_client.h"

#include "sensors.h"

/**
 * @brief  Task that is used on the gateway device.
 *         Checks if there is received data from BLE Mesh network and 
 *         sends it to the cloud via MQTT
 * 
 * @param  arg  Pointer of arguments used on the task (should be NULL)
 */
void gateway_device_task(void *arg);

/**
 * @brief  Task that is used on a common node device.
 *         Collects data from all sensors and publishes it on group 
 *         address 0xC100 of the BLE Mesh network
 * 
 * @param  arg  Pointer of arguments used on the task (should be NULL)
 */
void node_device_task(void *arg);

 
#endif //NETWORK_H
