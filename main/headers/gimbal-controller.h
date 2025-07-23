//
// Created by Timmy Li on 7/22/25.
//

#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H
#include <cstdint>

void gimbal_controller_init();
void gimbal_controller_task(void *parameters);


struct gimbal_state {
    float servo_0_angle;
    float servo_1_angle;
    bool servo_0_adding;
    bool servo_1_adding;
    bool tracking;
};

#endif //GIMBAL_CONTROLLER_H
