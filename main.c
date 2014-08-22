#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "stdint.h"
#include <stdio.h>

#include "spi_AD7980_ADC.h"
#include "spi_MAX5541_DAC.h"
#include "function.h"
#include "uart_com1.h"

#include "lwip/tcp.h"
#include "lwip/sockets.h"
#include "lwip/tcp_impl.h"

#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#define dac_max  65535
//39321
//39321  
#define dac_min		0

#define SYSTEMTICK_PERIOD_MS  10
uint16_t dac_data,adc_data1[dac_max];//,adc_data2[dac_max];
int8_t dac_step=1;
volatile uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;

void RCC_clock_set(void);
void LED_set(void);
void STM_EVAL_PBInit(void);//(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
void Trig_set(void);
static void USART_Config(void);
void tcp_echoclient_connect2(void);
static err_t tcp_connected2(void *arg, struct tcp_pcb *pcb, err_t err);
void TIM6_Config(void);

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

static __IO uint32_t TimingDelay;

uint8_t tcp_conneced=0;
struct tcp_pcb *echoclient_pcb2;



int main()
{

	RCC_DeInit();
	RCC_clock_set();
	LED_set();
	STM_EVAL_PBInit();//(2, 1); //init S2 button for interrupt
	Trig_set();
	sMAX5541_DAC_Init();
	sAD7980_ADC_Init();
	ETH_BSP_Config();
	LwIP_Init();
	TIM6_Config();


	USART_Config();

//#define	dac_step 1
	dac_data=1;

//	  if (SysTick_Config(500))
//	  {
//	    /* Capture error */
//	    while (1);
//	  }
//	printf("uart ok");

//	{
//	  struct ip_addr DestIPaddr;
//
//	  /* create new tcp pcb */
//	  echoclient_pcb = tcp_new();
//
//	  if (echoclient_pcb != NULL)
//	  {
//	    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
//
//	    /* connect to destination address/port */
//	    tcp_connect(echoclient_pcb,&DestIPaddr,DEST_PORT,tcp_connected);
//	  }
//
//	  uint16_t test_tcp[100]={0};
//	  uint8_t i;
//	  for(i=0;i<100;i++)
//	    {
//	      test_tcp[i]=i;
//	    }
//	tcp_write(echoclient_pcb2,(const void *)dac_data,sizeof(dac_data),1);
//	tcp_output(echoclient_pcb2);
//	}


	for(;;)
	{
	    int i,jj;
		// Sawtooth
      if (dac_data >= dac_max)
	{

	  dac_step = 1;
	  dac_data = dac_min;
#if 0
	  if (adc_data1[dac_max - 1000] != 0 || adc_data1[dac_max - 1001] != 0
	      || adc_data1[dac_max - 1002] != 0)
	    {
	      for (i = dac_min; i < dac_max; i++)
		{
		  //printf("%d,%d\n", i, adc_data1[i]);
		  printf ("%d,%d\n\r", i, adc_data1[i]);
		  //printf("%d,%d,%d\n", i, adc_data1[i],adc_data2[i]);
		}
	    }
#endif
	}
#if     0
		dac_data+=dac_step;
	_delay_us(3);
	sMAX5541_DAC_CS_LOW();
	_delay_us(3);
	SPI_I2S_SendData(SPI2,dac_data);
	_delay_us(3);
	sMAX5541_DAC_CS_HIGH();

#endif

#if 0
		{
#define adc_times	1
			uint32_t data_temp[3]={0};
			int i;
			for (i = 0; i < adc_times; i++) {
				sAD7980_ADC_CS_HIGH();
				while (GPIO_ReadInputDataBit(sAD7980_IRQ_GPIO_PORT,
						sAD7980_IRQ_PIN) != 0) {
					uint8_t i;
					for (i = 0; i < 3; i++) {
//						uint16_t temp1;
						SPI_I2S_SendData(SPI5, 0);
						_delay_us(20);
						data_temp[i] += SPI_I2S_ReceiveData(SPI5);
//						if(data_temp[i]<temp1)
//							data_temp[i]=temp1;
					}
					sAD7980_ADC_CS_LOW();
				}
			}
			adc_data1[dac_data] = data_temp[0]/adc_times;
			//adc_data1[dac_data] = data_temp[2]/adc_times;
		}
#endif

//	if(jj<5002)
//	  jj++;
//
//      if (jj == 3000||jj==1000||jj==2000)
//	{
//	  _delay_ms (100);
//	  tcp_echoclient_connect2 ();
//	}
	}
}

void tcp_echoclient_connect2(void)
{
////////////////////////////////////////
  struct ip_addr DestIPaddr;

  /* create new tcp pcb */
  echoclient_pcb2 = tcp_new();

  if (echoclient_pcb2 != NULL)
  {
    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );

    /* connect to destination address/port */
    tcp_connect(echoclient_pcb2,&DestIPaddr,DEST_PORT,tcp_connected2);
  }
  else
  {
    /* deallocate the pcb */
    memp_free(MEMP_TCP_PCB, echoclient_pcb2);
#ifdef SERIAL_DEBUG
    printf("\n\r can not create tcp pcb");
#endif
  }
  /////////
}
static err_t tcp_connected2(void *arg, struct tcp_pcb *pcb, err_t err)
{
#define data_length	1000
//u8_t   data[100];
  uint16_t data[data_length];

  uint32_t i;
  s8_t tcp_send_stat = -1;
  for (i = 0; i < data_length; i++)
    {
      data[i] = i;
    }

  tcp_write (pcb, data, sizeof(data), 1); /* ·¢ËÍÊý¾Ý */
//  tcp_output(pcb);
  tcp_close (pcb);
//	tcp_conneced=1;
  return ERR_OK;
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

  RCC_MCO1Config (RCC_MCO1Source_HSE, RCC_MCO1Div_5);
  //RCC_HSICmd(ENABLE); //enable internal clock 16M
//  RCC_HSEConfig (RCC_HSE_ON);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//set PC9 for MCO2
  RCC_MCO2Config (RCC_MCO2Source_SYSCLK, RCC_MCO2Div_5);

  RCC_PLLConfig(RCC_PLLSource_HSE,5,144,5,2);	//set system clock to 180mHz
  RCC_PLLCmd(ENABLE);
//  RCC_PLLSAIConfig();
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  RCC_HSEConfig (RCC_HSE_ON);
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
void Trig_set(void)
{
	  GPIO_InitTypeDef  GPIO_InitStructure;

	  /* GPIOC,GPIOD and GPIOI Periph clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOH, &GPIO_InitStructure);
}


/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of 10ms periods to wait for.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Capture the current local time */
  timingdelay = LocalTime + nCount;

  /* wait until the desired delay finish */
  while(timingdelay > LocalTime)
  {
  }
}
/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}

void TIM6_Config(void)
{	//set to 250ms per times for ethenet
  uint16_t PrescalerValue = 0;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 3000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 60000;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM6, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* TIM Interrupts enable */
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM6, ENABLE);
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
  USART_InitStructure.USART_BaudRate = 921600;//256000;//115200;
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

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_WAKEUP: Wakeup Push Button
  *     @arg BUTTON_TAMPER: Tamper Push Button
  *     @arg BUTTON_TAMPER: Tamper Push Button
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                     generation capability
  * @retval None
  */

//#define KEY_BUTTON_PIN                   GPIO_Pin_13
//#define KEY_BUTTON_GPIO_PORT             GPIOC
//#define KEY_BUTTON_GPIO_CLK              RCC_AHB1Periph_GPIOC
//#define KEY_BUTTON_EXTI_LINE             EXTI_Line13
//#define KEY_BUTTON_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOC
//#define KEY_BUTTON_EXTI_PIN_SOURCE       EXTI_PinSource13
//#define KEY_BUTTON_EXTI_IRQn             EXTI15_10_IRQn

void STM_EVAL_PBInit(void)//(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;


  /* Enable the BUTTON Clock */
  RCC_AHB1PeriphClockCmd(KEY_BUTTON_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = KEY_BUTTON_PIN;
  GPIO_Init(KEY_BUTTON_GPIO_PORT, &GPIO_InitStructure);


 // if (Button_Mode == BUTTON_MODE_EXTI)
  {
    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(KEY_BUTTON_EXTI_PORT_SOURCE, KEY_BUTTON_EXTI_PIN_SOURCE);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = KEY_BUTTON_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

    if(1)//(Button != BUTTON_WAKEUP)
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    }
    else
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    }
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = KEY_BUTTON_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
  }
}
