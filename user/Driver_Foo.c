#define FOO_FILE

#include "Driver_Foo.h"

void Foo_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = USART1_PIN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(USART1_PORT, &GPIO_InitStruct);
  GPIO_ResetBits(USART1_PORT, USART1_PIN);
	fooRemainCnt = 0;
	fooo = fooState = 0;
}

int32_t Foo_Press(int32_t cnt) {
	if (fooRemainCnt) return 1;
	fooRemainCnt = cnt;
  return 0;
}