
#ifndef RTOS_SYNC_H
#define RTOS_SYNC_H

#include "freertos/event_groups.h"

#define EVT_GRP_COLOR_SENSOR_COMPLETE   (1<<0)
#define EVT_GRP_AIR_SENSOR_COMPLETE     (1<<1)
#define EVT_GRP_TEMP_SENSOR_COMPLETE    (1<<2)
#define EVT_GRP_SOUND_SENSOR_COMPLETE   (1<<3)
#define EVT_GRP_FEEDBACK_TIME           (1<<4)
#define EVT_GRP_FEEDBACK_COMPLETE       (1<<5)
#define EVT_GRP_BITS         (EVT_GRP_COLOR_SENSOR_COMPLETE | EVT_GRP_AIR_SENSOR_COMPLETE | EVT_GRP_TEMP_SENSOR_COMPLETE | EVT_GRP_SOUND_SENSOR_COMPLETE)

EventGroupHandle_t sensorsEventGroup;


#endif
