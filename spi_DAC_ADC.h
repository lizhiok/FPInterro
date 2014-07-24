/**
  ******************************************************************************
  * @file    SPI/SPI_DAC_ADC/spi_DAC_ADC.h
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   This file contains all the functions prototypes for the spi_DAC_ADC
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
#ifndef __SPI_DAC_ADC_H
#define __SPI_DAC_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* M25P SPI DAC_ADC supported commands */
#define sDAC_ADC_CMD_WRITE          0x02  /* Write to Memory instruction */
#define sDAC_ADC_CMD_WRSR           0x01  /* Write Status Register instruction */
#define sDAC_ADC_CMD_WREN           0x06  /* Write enable instruction */
#define sDAC_ADC_CMD_READ           0x03  /* Read from Memory instruction */
#define sDAC_ADC_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sDAC_ADC_CMD_RDID           0x9F  /* Read identification */
#define sDAC_ADC_CMD_SE             0xD8  /* Sector Erase instruction */
#define sDAC_ADC_CMD_BE             0xC7  /* Bulk Erase instruction */

#define sDAC_ADC_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */

#define sDAC_ADC_DUMMY_BYTE         0xA5
#define sDAC_ADC_SPI_PAGESIZE       0x100

#define sDAC_ADC_M25P128_ID         0x202018
#define sDAC_ADC_M25P64_ID          0x202017
  
/* M25P DAC_ADC SPI Interface pins  */
#define sDAC_ADC_SPI                           SPI2
#define sDAC_ADC_SPI_CLK                       RCC_APB1Periph_SPI2
#define sDAC_ADC_SPI_CLK_INIT                  RCC_APB1PeriphClockCmd

#define sDAC_ADC_SPI_SCK_PIN                   GPIO_Pin_1
#define sDAC_ADC_SPI_SCK_GPIO_PORT             GPIOI
#define sDAC_ADC_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOI
#define sDAC_ADC_SPI_SCK_SOURCE                GPIO_PinSource1
#define sDAC_ADC_SPI_SCK_AF                    GPIO_AF_SPI2

#define sDAC_ADC_SPI_MISO_PIN                  GPIO_Pin_2
#define sDAC_ADC_SPI_MISO_GPIO_PORT            GPIOI
#define sDAC_ADC_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOI
#define sDAC_ADC_SPI_MISO_SOURCE               GPIO_PinSource2
#define sDAC_ADC_SPI_MISO_AF                   GPIO_AF_SPI2

#define sDAC_ADC_SPI_MOSI_PIN                  GPIO_Pin_3
#define sDAC_ADC_SPI_MOSI_GPIO_PORT            GPIOI
#define sDAC_ADC_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOI
#define sDAC_ADC_SPI_MOSI_SOURCE               GPIO_PinSource3
#define sDAC_ADC_SPI_MOSI_AF                   GPIO_AF_SPI2

#define sDAC_ADC_CS_PIN                        GPIO_Pin_0
#define sDAC_ADC_CS_GPIO_PORT                  GPIOI
#define sDAC_ADC_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOI

/* Exported macro ------------------------------------------------------------*/
/* Select sDAC_ADC: Chip Select pin low */
#define sDAC_ADC_CS_LOW()       GPIO_ResetBits(sDAC_ADC_CS_GPIO_PORT, sDAC_ADC_CS_PIN)
/* Deselect sDAC_ADC: Chip Select pin high */
#define sDAC_ADC_CS_HIGH()      GPIO_SetBits(sDAC_ADC_CS_GPIO_PORT, sDAC_ADC_CS_PIN)

/* Exported functions ------------------------------------------------------- */

/* High layer functions  */
void sDAC_ADC_DeInit(void);
void sDAC_ADC_Init(void);
void sDAC_ADC_EraseSector(uint32_t SectorAddr);
void sDAC_ADC_EraseBulk(void);
void sDAC_ADC_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sDAC_ADC_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sDAC_ADC_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sDAC_ADC_ReadID(void);
void sDAC_ADC_StartReadSequence(uint32_t ReadAddr);

/* Low layer functions */
uint8_t sDAC_ADC_ReadByte(void);
uint8_t sDAC_ADC_SendByte(uint8_t byte);
uint16_t sDAC_ADC_SendHalfWord(uint16_t HalfWord);
void sDAC_ADC_WriteEnable(void);
void sDAC_ADC_WaitForWriteEnd(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_DAC_ADC_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
