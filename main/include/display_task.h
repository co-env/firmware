#ifndef __DISPLAY_TASK_H__
#define __DISPLAY_TASK_H__


void FontDisplayTask(void* arg);
void update_display_data(uint32_t temperature, uint16_t tvoc, uint16_t eco2);

#endif  // __DISPLAY_TASK_H__