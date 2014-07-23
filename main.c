#include "stm32f4xx.h"


void RCC_clock_set(void);
int main()
{
	RCC_DeInit();
	RCC_clock_set();

	for(;;)
	{
	__asm{NOP};
	GPIO_SetBits(GPIOD,GPIO_Pin_13);
	__asm{NOP};
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);

	}
}

void RCC_clock_set(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOC,GPIOD and GPIOI Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCO1Config(RCC_MCO1Source_HSE,RCC_MCO1Div_1);
	//RCC_HSICmd(ENABLE); //enable internal clock 16M
	RCC_HSEConfig(RCC_HSE_ON);
}


void LED_set(void)
{
	  GPIO_InitTypeDef  GPIO_InitStructure;
	  /* Configure PD.02 CMD line */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
