//
// Created by Timmy Li on 7/22/25.
//

#ifndef MAIN_H
#define MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t ir_camera_queue;
extern QueueHandle_t gimbal_controller_queue;

#endif //MAIN_H
