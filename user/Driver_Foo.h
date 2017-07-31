#ifndef FOO_H
#define FOO_H

#include "stm32f4xx.h"

#define USART1_PORT GPIOB
#define USART1_PIN (GPIO_Pin_6 | GPIO_Pin_7)

#ifndef FOO_FILE
  #define FOO_EXT extern
#else
  #define FOO_EXT
#endif

FOO_EXT int32_t fooRemainCnt, fooo, fooState;

void Foo_Init(void);
	
/*
	How to use:
	1.press n times
		Foo_Press(n) // if last call not finished, there's no effect
	2.test whether last call finished
		if (Foo_Press(0)==0) {
		}
		// 0 => finished 1 => not finished
*/
int32_t Foo_Press(int32_t cnt);

#endif
