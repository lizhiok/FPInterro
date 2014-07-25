/**
  ******************************************************************************
  * @file    SPI/SPI_AD7980_ADC/spi_AD7980_ADC.h
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   This file contains all the functions prototypes for the spi_AD7980_ADC
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_AD7980_ADC_H
#define __SPI_AD7980_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* M25P SPI AD7980_ADC supported commands */
#define sAD7980_ADC_CMD_WRITE          0x02  /* Write to Memory instruction */
#define sAD7980_ADC_CMD_WRSR           0x01  /* Write Status Register instruction */
#define sAD7980_ADC_CMD_WREN           0x06  /* Write enable instruction */
#define sAD7980_ADC_CMD_READ           0x03  /* Read from Memory instruction */
#define sAD7980_ADC_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sAD7980_ADC_CMD_RDID           0x9F  /* Read identification */
#define sAD7980_ADC_CMD_SE             0xD8  /* Sector Erase instruction */
#define sAD7980_ADC_CMD_BE             0xC7  /* Bulk Erase instruction */

#define sAD7980_ADC_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */

#define sAD7980_ADC_DUMMY_BYTE         0xA5
#define sAD7980_ADC_SPI_PAGESIZE       0x100

#define sAD7980_ADC_M25P128_ID         0x202018
#define sAD7980_ADC_M25P64_ID          0x202017

 /* AD7980 ADC SPI Interface pins  */
 #define sAD7980_ADC_SPI                           SPI5
 #define sAD7980_ADC_SPI_CLK                       RCC_APB2Periph_SPI5
 #define sAD7980_ADC_SPI_CLK_INIT                  RCC_APB2PeriphClockCmd

 #define sAD7980_ADC_SPI_SCK_PIN                   GPIO_Pin_7
 #define sAD7980_ADC_SPI_SCK_GPIO_PORT             GPIOF
 #define sAD7980_ADC_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOF
 #define sAD7980_ADC_SPI_SCK_SOURCE                GPIO_PinSource7
 #define sAD7980_ADC_SPI_SCK_AF                    GPIO_AF_SPI5

 #define sAD7980_ADC_SPI_MISO_PIN                  GPIO_Pin_8
 #define sAD7980_ADC_SPI_MISO_GPIO_PORT            GPIOF
 #define sAD7980_ADC_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOF
 #define sAD7980_ADC_SPI_MISO_SOURCE               GPIO_PinSource8
 #define sAD7980_ADC_SPI_MISO_AF                   GPIO_AF_SPI5

 #define sAD7980_ADC_SPI_MOSI_PIN                  GPIO_Pin_9
 #define sAD7980_ADC_SPI_MOSI_GPIO_PORT            GPIOF
 #define sAD7980_ADC_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOF
 #define sAD7980_ADC_SPI_MOSI_SOURCE               GPIO_PinSource9
 #define sAD7980_ADC_SPI_MOSI_AF                   GPIO_AF_SPI5

 #define sAD7980_ADC_CS_PIN                        GPIO_Pin_6
 #define sAD7980_ADC_CS_GPIO_PORT                  GPIOF
 #define sAD7980_ADC_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOF

/* Exported macro ------------------------------------------------------------*/
/* Select sAD7980_ADC: Chip Select pin low */
#define sAD7980_ADC_CS_LOW()       GPIO_ResetBits(sAD7980_ADC_CS_GPIO_PORT, sAD7980_ADC_CS_PIN)
/* Deselect sAD7980_ADC: Chip Select pin high */
#define sAD7980_ADC_CS_HIGH()      GPIO_SetBits(sAD7980_ADC_CS_GPIO_PORT, sAD7980_ADC_CS_PIN)

/* Exported functions ------------------------------------------------------- */

/* High layer functions  */
void sAD7980_ADC_DeInit(void);
void sAD7980_ADC_Init(void);
void sAD7980_ADC_EraseSector(uint32_t SectorAddr);
void sAD7980_ADC_EraseBulk(void);
void sAD7980_ADC_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sAD7980_ADC_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sAD7980_ADC_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sAD7980_ADC_ReadID(void);
void sAD7980_ADC_StartReadSequence(uint32_t ReadAddr);

/* Low layer functions */
uint8_t sAD7980_ADC_ReadByte(void);
uint8_t sAD7980_ADC_SendByte(uint8_t byte);
uint16_t sAD7980_ADC_SendHalfWord(uint16_t HalfWord);
void sAD7980_ADC_WriteEnable(void);
void sAD7980_ADC_WaitForWriteEnd(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_AD7980_ADC_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
