/*
 * uart_com1.h
 *
 *  Created on: 2014Äê8ÔÂ4ÈÕ
 *      Author: zli
 */


/**
  * @}
  */

/** @addtogroup STM324x9I_EVAL_LOW_LEVEL_COM
  * @{
  */

#ifndef _UART_COM1_H
#define _UART_COM1_H

#include"stm32f4xx_usart.h"
#define COMn                             1

/**
 * @brief Definition for COM port1, connected to USART1
 */

typedef enum
{
  COM1 = 0,
  COM2 = 1
} COM_TypeDef;

#define EVAL_COM1                        USART1
#define EVAL_COM1_CLK                    RCC_APB2Periph_USART1
#define EVAL_COM1_TX_PIN                 GPIO_Pin_9
#define EVAL_COM1_TX_GPIO_PORT           GPIOA
#define EVAL_COM1_TX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define EVAL_COM1_TX_SOURCE              GPIO_PinSource9
#define EVAL_COM1_TX_AF                  GPIO_AF_USART1
#define EVAL_COM1_RX_PIN                 GPIO_Pin_10
#define EVAL_COM1_RX_GPIO_PORT           GPIOA
#define EVAL_COM1_RX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define EVAL_COM1_RX_SOURCE              GPIO_PinSource10
#define EVAL_COM1_RX_AF                  GPIO_AF_USART1
#define EVAL_COM1_IRQn                   USART1_IRQn

void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct);
	
#endif
