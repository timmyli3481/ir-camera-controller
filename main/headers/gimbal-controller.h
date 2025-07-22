//
// Created by Timmy Li on 7/22/25.
//

#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H
#include <cstdint>

void gimbal_controller_init();
void gimbal_controller_task(void *parameters);

struct gimbal_controller_input {
    bool tracking;
    float target_x;
    float target_y;
};

struct gimbal_state {
    uint16_t servo_0_angle;
    uint16_t servo_1_angle;
    bool servo_1_adding;
    bool servo_0_adding;
};

#endif //GIMBAL_CONTROLLER_H
