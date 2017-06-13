#define DRIVER_CAMERA_FILE

#include <Driver_Camera.h>

static void BSP_CameraInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // Camera Control
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_2MHz;
    // Sig1, PB3 PB4 PB5
		// see LeftFront Wheel
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
    // Sig2, PC9 PC8 PC7
		// see RightFront Wheel
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7);
    // Sig3, PC6 PB15 PB14
		// watch Gripper
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_6;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_15 | GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15 | GPIO_Pin_14);
    // Sig4, PC0 PC1 PC2
		// watch Shooter
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
    // Sig5, PC3 PA0 PA1
		// for Resource Climbing usage
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_3;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_3);
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
}

void CameraInit(void) {
  BSP_CameraInit();
  
  for (int32_t i = 0; i < CAMERA_SIG_CNT; ++i)
    CameraState[i] = 0;
}

void SetCameraChannel(int32_t signal, int32_t channel) {
  if (signal < 1 || signal > 5) return;
	// select from 1 to 5
  if (channel < 0 || channel > 3) return;
	// select from 1 to 3, channel = 0 is for all reset usage
  CameraState[signal-1] = channel;
  switch (signal) {
    case 1:
      switch (channel) {
        case 0:
          GPIO_ResetBits(GPIOB, GPIO_Pin_3);
          GPIO_ResetBits(GPIOB, GPIO_Pin_4);
          GPIO_ResetBits(GPIOB, GPIO_Pin_5);
          return;

        case 1:
          GPIO_SetBits(GPIOB, GPIO_Pin_3);
          GPIO_ResetBits(GPIOB, GPIO_Pin_4);
          GPIO_ResetBits(GPIOB, GPIO_Pin_5);
          return;

        case 2:
          GPIO_ResetBits(GPIOB, GPIO_Pin_3);
          GPIO_SetBits(GPIOB, GPIO_Pin_4);
          GPIO_ResetBits(GPIOB, GPIO_Pin_5);
          return;

        case 3:
          GPIO_ResetBits(GPIOB, GPIO_Pin_3);
          GPIO_ResetBits(GPIOB, GPIO_Pin_4);
          GPIO_SetBits(GPIOB, GPIO_Pin_5);
          return;
      }return;

    case 2:
      switch (channel) {
        case 0:
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);
          GPIO_ResetBits(GPIOC, GPIO_Pin_8);
          GPIO_ResetBits(GPIOC, GPIO_Pin_7);
          return;

        case 1:
          GPIO_SetBits(GPIOC, GPIO_Pin_9);
          GPIO_ResetBits(GPIOC, GPIO_Pin_8);
          GPIO_ResetBits(GPIOC, GPIO_Pin_7);
          return;

        case 2:
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);
          GPIO_SetBits(GPIOC, GPIO_Pin_8);
          GPIO_ResetBits(GPIOC, GPIO_Pin_7);
          return;

        case 3:
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);
          GPIO_ResetBits(GPIOC, GPIO_Pin_8);
          GPIO_SetBits(GPIOC, GPIO_Pin_7);
          return;
      }return;

    case 3:
      switch (channel) {
        case 0:
          GPIO_ResetBits(GPIOC, GPIO_Pin_6);
          GPIO_ResetBits(GPIOB, GPIO_Pin_15);
          GPIO_ResetBits(GPIOB, GPIO_Pin_14);
          return;

        case 1:
          GPIO_SetBits(GPIOC, GPIO_Pin_6);
          GPIO_ResetBits(GPIOB, GPIO_Pin_15);
          GPIO_ResetBits(GPIOB, GPIO_Pin_14);
          return;

        case 2:
          GPIO_ResetBits(GPIOC, GPIO_Pin_6);
          GPIO_SetBits(GPIOB, GPIO_Pin_15);
          GPIO_ResetBits(GPIOB, GPIO_Pin_14);
          return;

        case 3:
          GPIO_ResetBits(GPIOC, GPIO_Pin_6);
          GPIO_ResetBits(GPIOB, GPIO_Pin_15);
          GPIO_SetBits(GPIOB, GPIO_Pin_14);
          return;
      }return;

    case 4:
      switch (channel) {
        case 0:
          GPIO_ResetBits(GPIOC, GPIO_Pin_0);
          GPIO_ResetBits(GPIOC, GPIO_Pin_1);
          GPIO_ResetBits(GPIOC, GPIO_Pin_2);
          return;

        case 1:
          GPIO_SetBits(GPIOC, GPIO_Pin_0);
          GPIO_ResetBits(GPIOC, GPIO_Pin_1);
          GPIO_ResetBits(GPIOC, GPIO_Pin_2);
          return;

        case 2:
          GPIO_ResetBits(GPIOC, GPIO_Pin_0);
          GPIO_SetBits(GPIOC, GPIO_Pin_1);
          GPIO_ResetBits(GPIOC, GPIO_Pin_2);
          return;

        case 3:
          GPIO_ResetBits(GPIOC, GPIO_Pin_0);
          GPIO_ResetBits(GPIOC, GPIO_Pin_1);
          GPIO_SetBits(GPIOC, GPIO_Pin_2);
          return;
      }return;

    case 5:
      switch (channel) {
        case 0:
          GPIO_ResetBits(GPIOC, GPIO_Pin_3);
          GPIO_ResetBits(GPIOA, GPIO_Pin_0);
          GPIO_ResetBits(GPIOA, GPIO_Pin_1);
          return;

        case 1:
          GPIO_SetBits(GPIOC, GPIO_Pin_3);
          GPIO_ResetBits(GPIOA, GPIO_Pin_0);
          GPIO_ResetBits(GPIOA, GPIO_Pin_1);
          return;

        case 2:
          GPIO_ResetBits(GPIOC, GPIO_Pin_3);
          GPIO_SetBits(GPIOA, GPIO_Pin_0);
          GPIO_ResetBits(GPIOA, GPIO_Pin_1);
          return;

        case 3:
          GPIO_ResetBits(GPIOC, GPIO_Pin_3);
          GPIO_ResetBits(GPIOA, GPIO_Pin_0);
          GPIO_SetBits(GPIOA, GPIO_Pin_1);
          return;
      }return;
  }
}

int32_t GetCameraChannel(int32_t signal) {
  if (signal < 1 || signal > 5) return 0;
  return CameraState[signal-1];
	// will return a number from 0 to 3
	// if in the range 1 to 3, then means one of the channel in this signal is on
	// if zero is returned, then means all channels in this signal are turned off
}
