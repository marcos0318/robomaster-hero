#ifndef DRIVER_CAMERA_H
#define DRIVER_CAMERA_H

#include <stm32f4xx.h>

#ifndef DRIVER_CAMERA_FILE
  #define DRIVER_CAMERA_EXT extern
#else
  #define DRIVER_CAMERA_EXT
#endif

#define CAMERA_SIG_CNT 5

DRIVER_CAMERA_EXT uint8_t CameraState[CAMERA_SIG_CNT];

/*
 * Initialize camera
 */
void CameraInit(void);

/*
 * Control camera signal multiplexing
 * signal: 1-5
 * channel: 0-3 (0 is all off)
 */
void SetCameraChannel(int32_t signal, int32_t channel);

/*
 * Get current camera channel
 */
int32_t GetCameraChannel(int32_t signal);

void goOnStageMode();
void onStageMode();
void offStageMode();
void runningMode();

#endif

