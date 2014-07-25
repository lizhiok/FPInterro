#include "main.h"
#include "stm32f4xx.h"
#include "spi_AD7980_ADC.h"
#include "spi_MAX5541_DAC.h"

void RCC_clock_set(void);
void LED_set(void);

static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);

int main()
{
	uint16_t dac_data;
	RCC_DeInit();
	RCC_clock_set();
	LED_set();
	sMAX5541_DAC_Init();
	sAD7980_ADC_Init();

#define	dac_step 1
	dac_data=0;

	  if (SysTick_Config(500))
	  {
	    /* Capture error */
	    while (1);
	  }
	  Delay(1);
	for(;;)
	{
//		dac_data+=dac_step;
		sMAX5541_DAC_CS_LOW();
		SPI_I2S_SendData(SPI2,65535/2);
		sMAX5541_DAC_CS_HIGH();
//		Delay(10);
	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};

//	sAD7980_ADC_CS_LOW();
	SPI_I2S_SendData(SPI5,65535);
	__asm__{NOP};
//	sAD7980_ADC_CS_HIGH();

//	GPIO_SetBits(sAD7980_ADC_SPI_SCK_GPIO_PORT, sAD7980_ADC_SPI_SCK_PIN);
	__asm__{NOP};
//	GPIO_ResetBits(sAD7980_ADC_SPI_SCK_GPIO_PORT, sAD7980_ADC_SPI_SCK_PIN);
	__asm__{NOP};
//	GPIO_SetBits(GPIOD,GPIO_Pin_13);
//	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
//	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
//	__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};__asm{NOP};
//	GPIO_ResetBits(GPIOD,GPIO_Pin_13);

	}
}

//void spi_gpio_set(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//	RCC_APB1PeriphClockCmd();
//
//	RCC_AHB1PeriphClockCmd(sFLASH_SPI_SCK_GPIO_CLK | sFLASH_SPI_MISO_GPIO_CLK |
//	                         sFLASH_SPI_MOSI_GPIO_CLK | sFLASH_CS_GPIO_CLK, ENABLE);
//}
//void spi_dac(void)
//{
//	  SPI_InitTypeDef  SPI_InitStructure;
//
//	  //sFLASH_LowLevel_Init();
//	  spi_gpio_set();
//
//	  /*!< Deselect the FLASH: Chip Select high */
//	  //sFLASH_CS_HIGH();
//
//	  /*!< SPI configuration */
//	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
//	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
//	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
//	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//
//	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//	  SPI_InitStructure.SPI_CRCPolynomial = 7;
//	  SPI_Init(SPI2, &SPI_InitStructure);
//
//	  /*!< Enable the sFLASH_SPI  */
//	  SPI_Cmd(SPI2, ENABLE);
//}
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

	  /* GPIOC,GPIOD and GPIOI Periph clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0)
  {
    TimingDelay--;
  }
}
