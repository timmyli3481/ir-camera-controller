//
// Created by Timmy Li on 7/20/25.
//

#ifndef IR_CAMERA_H
#define IR_CAMERA_H

#include <stdint.h>
#include <stdbool.h>

// Define your UART port. Change this if needed.
#define UART_PORT UART_NUM_2

// Maximum length of a received line (e.g., "FIRE,X.XX,Y.YY,PIXELS,CX,CY \r\n")
// "FIRE,-XXX.XX,-XXX.XX,NNNNNN,NNN,NNN \r\n" -> 6 (FIRE,) + 8 (angle) + 8 (angle) + 7 (pixels) + 4 (cx) + 4 (cy) + 5 (spaces/CRLF) ~ 40 chars.
// Let's make it a bit larger for safety.
#define CAMERA_RX_BUFFER_SIZE 256 // Changed from 64 to 256

// Struct to hold decoded camera data
typedef struct {
    bool found_fire;
    float angle_x;
    float angle_y;
    int pixels;
    int cx;
    int cy;
} camera_data_t;

// Global instance to store the latest camera data
extern camera_data_t g_camera_data;

// Function to initialize UART for camera communication
void uart_camera_init();

// Task to read data from UART and push to decode buffer
void uart_camera_task(void *pvParameters);

// Function to decode a complete line of camera data
void decodeCameraData(const char *data_to_decode);

#endif //IR_CAMERA_H
