//
// Created by Timmy Li on 7/22/25.
//

#ifndef MAIN_H
#define MAIN_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


extern SemaphoreHandle_t camera_data_mutex;
extern QueueHandle_t crsf_commands_queue;

#endif //MAIN_H
