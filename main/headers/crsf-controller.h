//
// Created by Timmy Li on 7/22/25.
//

#ifndef CRSF_TASK_H
#define CRSF_TASK_H
#include <cstdint>

void crsf_controller_init();

void crsf_controller_task(void *pvParameters);

struct int_null_t {
    int32_t value;
    bool is_null;
};

struct crsf_command_t {
    int_null_t throttle;
    int_null_t yaw;
    int_null_t pitch;
    int_null_t roll;
};

#endif //CRSF_TASK_H
