/*
 * sdram.h
 *
 *  Created on: 2014Äê11ÔÂ18ÈÕ
 *      Author: zli
 */

#ifndef SDRAM_H_
#define SDRAM_H_
#include "stm32f4xx_hal_sdram.h"
#include "stdint.h"

/* Private variables ---------------------------------------------------------*/

#define SDRAM_MEMORY_WIDTH               FMC_SDRAM_MEM_BUS_WIDTH_32

#define SDCLOCK_PERIOD                   FMC_SDRAM_CLOCK_PERIOD_2


/* DMA definitions for SDRAM DMA transfer */
#define __DMAx_CLK_ENABLE                 __DMA2_CLK_ENABLE
#define SDRAM_DMAx_CHANNEL                DMA_CHANNEL_0
#define SDRAM_DMAx_STREAM                 DMA2_Stream0
#define SDRAM_DMAx_IRQn                   DMA2_Stream0_IRQn
#define SDRAM_DMAx_IRQHandler             DMA2_Stream0_IRQHandler

#define   SDRAM_OK         0x00
#define   SDRAM_ERROR      0x01

#define REFRESH_COUNT                    ((uint32_t)0x0569)   /* SDRAM refresh counter (90Mhz SD clock) */
#define SDRAM_TIMEOUT     ((uint32_t)0xFFFF)

//static SD_HandleTypeDef uSdHandle;
/**
  * @brief  FMC SDRAM Mode definition register defines
  */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

//cut from hal_gpio.h
#define __FMC_CLK_ENABLE()   (RCC->AHB3ENR |= (RCC_AHB3ENR_FMCEN))
#define __FMC_CLK_DISABLE()  (RCC->AHB3ENR &= ~(RCC_AHB3ENR_FMCEN))

#define __GPIOA_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN))
#define __GPIOB_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN))
#define __GPIOC_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN))
#define __GPIOD_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN))
#define __GPIOE_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOEEN))
#define __GPIOH_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOHEN))
#define __CRC_CLK_ENABLE()           (RCC->AHB1ENR |= (RCC_AHB1ENR_CRCEN))
#define __BKPSRAM_CLK_ENABLE()       (RCC->AHB1ENR |= (RCC_AHB1ENR_BKPSRAMEN))
#define __CCMDATARAMEN_CLK_ENABLE()  (RCC->AHB1ENR |= (RCC_AHB1ENR_CCMDATARAMEN))
#define __DMA1_CLK_ENABLE()          (RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN))
#define __DMA2_CLK_ENABLE()          (RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2EN))

#define __GPIOA_CLK_DISABLE()        (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN))
#define __GPIOB_CLK_DISABLE()        (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOBEN))
#define __GPIOC_CLK_DISABLE()        (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOCEN))
#define __GPIOD_CLK_DISABLE()        (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIODEN))
#define __GPIOE_CLK_DISABLE()        (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOEEN))
#define __GPIOH_CLK_DISABLE()        (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOHEN))
#define __CRC_CLK_DISABLE()          (RCC->AHB1ENR &= ~(RCC_AHB1ENR_CRCEN))
#define __BKPSRAM_CLK_DISABLE()      (RCC->AHB1ENR &= ~(RCC_AHB1ENR_BKPSRAMEN))
#define __CCMDATARAMEN_CLK_DISABLE() (RCC->AHB1ENR &= ~(RCC_AHB1ENR_CCMDATARAMEN))
#define __DMA1_CLK_DISABLE()         (RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA1EN))
#define __DMA2_CLK_DISABLE()         (RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA2EN))

#define __GPIOI_CLK_ENABLE()            (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOIEN))
#define __GPIOF_CLK_ENABLE()            (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOFEN))
#define __GPIOG_CLK_ENABLE()            (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOGEN))
#define __GPIOJ_CLK_ENABLE()            (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOJEN))
#define __GPIOK_CLK_ENABLE()            (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOKEN))
#define __DMA2D_CLK_ENABLE()            (RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2DEN))
#define __ETHMAC_CLK_ENABLE()           (RCC->AHB1ENR |= (RCC_AHB1ENR_ETHMACEN))
#define __ETHMACTX_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_ETHMACTXEN))
#define __ETHMACRX_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_ETHMACRXEN))
#define __ETHMACPTP_CLK_ENABLE()        (RCC->AHB1ENR |= (RCC_AHB1ENR_ETHMACPTPEN))
#define __USB_OTG_HS_CLK_ENABLE()       (RCC->AHB1ENR |= (RCC_AHB1ENR_OTGHSEN))
#define __USB_OTG_HS_ULPI_CLK_ENABLE()  (RCC->AHB1ENR |= (RCC_AHB1ENR_OTGHSULPIEN))

#define __GPIOF_CLK_DISABLE()           (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOFEN))
#define __GPIOG_CLK_DISABLE()           (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOGEN))
#define __GPIOI_CLK_DISABLE()           (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOIEN))
#define __GPIOJ_CLK_DISABLE()           (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOJEN))
#define __GPIOK_CLK_DISABLE()           (RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOKEN))
#define __DMA2D_CLK_DISABLE()           (RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA2DEN))
#define __ETHMAC_CLK_DISABLE()          (RCC->AHB1ENR &= ~(RCC_AHB1ENR_ETHMACEN))
#define __ETHMACTX_CLK_DISABLE()        (RCC->AHB1ENR &= ~(RCC_AHB1ENR_ETHMACTXEN))
#define __ETHMACRX_CLK_DISABLE()        (RCC->AHB1ENR &= ~(RCC_AHB1ENR_ETHMACRXEN))
#define __ETHMACPTP_CLK_DISABLE()       (RCC->AHB1ENR &= ~(RCC_AHB1ENR_ETHMACPTPEN))
#define __USB_OTG_HS_CLK_DISABLE()      (RCC->AHB1ENR &= ~(RCC_AHB1ENR_OTGHSEN))
#define __USB_OTG_HS_ULPI_CLK_DISABLE() (RCC->AHB1ENR &= ~(RCC_AHB1ENR_OTGHSULPIEN))

#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */

#define  GPIO_MODE_INPUT                        ((uint32_t)0x00000000)   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    ((uint32_t)0x00000001)   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    ((uint32_t)0x00000011)   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        ((uint32_t)0x00000002)   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        ((uint32_t)0x00000012)   /*!< Alternate Function Open Drain Mode    */

#define  GPIO_NOPULL        ((uint32_t)0x00000000)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        ((uint32_t)0x00000001)   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      ((uint32_t)0x00000002)   /*!< Pull-down activation                */

#define  GPIO_SPEED_LOW         ((uint32_t)0x00000000)  /*!< Low speed     */
#define  GPIO_SPEED_MEDIUM      ((uint32_t)0x00000001)  /*!< Medium speed  */
#define  GPIO_SPEED_FAST        ((uint32_t)0x00000002)  /*!< Fast speed    */
#define  GPIO_SPEED_HIGH        ((uint32_t)0x00000003)  /*!< High speed    */

uint8_t BSP_SDRAM_Init(void);

#endif /* SDRAM_H_ */
