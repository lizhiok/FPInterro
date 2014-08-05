#include "main.h"
#include "stdint.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "spi_AD7980_ADC.h"
#include "spi_MAX5541_DAC.h"
#include "function.h"
#include "uart_com1.h"

#define dac_max		39321
//39321  
//test git
#define dac_min		0

void Delay(__IO uint32_t nTime);
uint16_t dac_data,adc_data1[dac_max],adc_data2[dac_max];
int8_t dac_step=1;

void RCC_clock_set(void);
void LED_set(void);
static void USART_Config(void);

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

static __IO uint32_t TimingDelay;




int main()
{

	RCC_DeInit();
	RCC_clock_set();
	LED_set();
	sMAX5541_DAC_Init();
	sAD7980_ADC_Init();

	  /* USART configuration */
	  USART_Config();

//#define	dac_step 1
	dac_data=1;

//	  if (SysTick_Config(500))
//	  {
//	    /* Capture error */
//	    while (1);
//	  }
//	  Delay(1);

	  /* Output a message on Hyperterminal using printf function */
//	  printf("\n\rUSART Printf Example: retarget the C library printf function to the USARTlizhi\n\r");
	for(;;)
	{

		//26214=12V
#if 1
		if (dac_data <= dac_min) {
			int i;
			dac_step = 1;
			dac_data = dac_min;

			if (adc_data1[dac_max - 100] != 0 && adc_data1[dac_max - 150] != 0)
			{
				for (i = dac_min; i < dac_max; i++) {
					//printf("%d,%d\n", i, adc_data1[i]);
					printf("%d,%d,%d\n", i, adc_data1[i],adc_data2[i]);
				}
			}
		} else if (dac_data >= dac_max) {
			int i;
			dac_step = -1;
			dac_data = dac_max;

			if (adc_data1[dac_max - 100] != 0 && adc_data1[dac_max - 150] != 0)
			{
				for (i = dac_min; i < dac_max; i++) {
					//printf("%d,%d\n", i, adc_data1[i]);
					printf("%d,%d,%d\n", i, adc_data1[i],adc_data2[i]);
				}
			}
		}
#else

#endif

		dac_data+=dac_step;
//		Delay(10);

	_delay_us(3);
	sMAX5541_DAC_CS_LOW();
	_delay_us(3);
	SPI_I2S_SendData(SPI2,dac_data);
	_delay_us(3);
	sMAX5541_DAC_CS_HIGH();
//	sAD7980_ADC_CS_LOW();
//	SPI_I2S_SendData(SPI5,65535);
//	adc_data=SPI_I2S_ReceiveData(SPI5);
//	__asm__{NOP};
//	sAD7980_ADC_CS_HIGH();
//	sAD7980_ADC_CS_LOW();
#if 1
	sAD7980_ADC_CS_HIGH();
		while (GPIO_ReadInputDataBit(sAD7980_IRQ_GPIO_PORT, sAD7980_IRQ_PIN)!= 0) {
			uint16_t data_temp[3];
			uint8_t i;
			for (i = 0; i < 3; i++) {
				SPI_I2S_SendData(SPI5, 0);
				_delay_us(10);
				data_temp[i] = SPI_I2S_ReceiveData(SPI5);
			}
			adc_data1[dac_data] = data_temp[0];
			adc_data2[dac_data] = data_temp[2];
			sAD7980_ADC_CS_LOW();
//			printf("%d,%d\n", 0, adc_data[dac_data]);  //test adc
//		adc_data[0]=data_temp[0]<<1|data_temp[1]&0x8000;
//		adc_data[1]=data_temp[1]<<1|data_temp[2]&0x8000;
//		adc_data[2]=data_temp[2]<<1|data_temp[3]&0x8000;
		}
#endif
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


/*************************************************
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;

  /* USARTx configured as follows:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  STM_EVAL_COMInit(COM1, &USART_InitStructure);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(EVAL_COM1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

//#ifdef  USE_FULL_ASSERT
//
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t* file, uint32_t line)
//{
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//
//  /* Infinite loop */
//  while (1)
//  {
//  }
//}
//#endif
