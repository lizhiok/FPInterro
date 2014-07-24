/**
  ******************************************************************************
  * @file    SPI/SPI_MAX5541_DAC/spi_MAX5541_DAC.h
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   This file contains all the functions prototypes for the spi_MAX5541_DAC
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
#ifndef __SPI_MAX5541_DAC_H
#define __SPI_MAX5541_DAC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* M25P SPI MAX5541_DAC supported commands */
#define sMAX5541_DAC_CMD_WRITE          0x02  /* Write to Memory instruction */
#define sMAX5541_DAC_CMD_WRSR           0x01  /* Write Status Register instruction */
#define sMAX5541_DAC_CMD_WREN           0x06  /* Write enable instruction */
#define sMAX5541_DAC_CMD_READ           0x03  /* Read from Memory instruction */
#define sMAX5541_DAC_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sMAX5541_DAC_CMD_RDID           0x9F  /* Read identification */
#define sMAX5541_DAC_CMD_SE             0xD8  /* Sector Erase instruction */
#define sMAX5541_DAC_CMD_BE             0xC7  /* Bulk Erase instruction */

#define sMAX5541_DAC_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */

#define sMAX5541_DAC_DUMMY_BYTE         0xA5
#define sMAX5541_DAC_SPI_PAGESIZE       0x100

#define sMAX5541_DAC_M25P128_ID         0x202018
#define sMAX5541_DAC_M25P64_ID          0x202017
  
/* MAX5541 DAC SPI Interface pins  */
#define sMAX5541_DAC_SPI                           SPI2
#define sMAX5541_DAC_SPI_CLK                       RCC_APB1Periph_SPI2
#define sMAX5541_DAC_SPI_CLK_INIT                  RCC_APB1PeriphClockCmd

#define sMAX5541_DAC_SPI_SCK_PIN                   GPIO_Pin_1
#define sMAX5541_DAC_SPI_SCK_GPIO_PORT             GPIOI
#define sMAX5541_DAC_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOI
#define sMAX5541_DAC_SPI_SCK_SOURCE                GPIO_PinSource1
#define sMAX5541_DAC_SPI_SCK_AF                    GPIO_AF_SPI2

#define sMAX5541_DAC_SPI_MISO_PIN                  GPIO_Pin_2
#define sMAX5541_DAC_SPI_MISO_GPIO_PORT            GPIOI
#define sMAX5541_DAC_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOI
#define sMAX5541_DAC_SPI_MISO_SOURCE               GPIO_PinSource2
#define sMAX5541_DAC_SPI_MISO_AF                   GPIO_AF_SPI2

#define sMAX5541_DAC_SPI_MOSI_PIN                  GPIO_Pin_3
#define sMAX5541_DAC_SPI_MOSI_GPIO_PORT            GPIOI
#define sMAX5541_DAC_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOI
#define sMAX5541_DAC_SPI_MOSI_SOURCE               GPIO_PinSource3
#define sMAX5541_DAC_SPI_MOSI_AF                   GPIO_AF_SPI2

#define sMAX5541_DAC_CS_PIN                        GPIO_Pin_0
#define sMAX5541_DAC_CS_GPIO_PORT                  GPIOI
#define sMAX5541_DAC_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOI

/* Exported macro ------------------------------------------------------------*/
/* Select sMAX5541_DAC: Chip Select pin low */
#define sMAX5541_DAC_CS_LOW()       GPIO_ResetBits(sMAX5541_DAC_CS_GPIO_PORT, sMAX5541_DAC_CS_PIN)
/* Deselect sMAX5541_DAC: Chip Select pin high */
#define sMAX5541_DAC_CS_HIGH()      GPIO_SetBits(sMAX5541_DAC_CS_GPIO_PORT, sMAX5541_DAC_CS_PIN)

/* Exported functions ------------------------------------------------------- */

/* High layer functions  */
void sMAX5541_DAC_DeInit(void);
void sMAX5541_DAC_Init(void);
void sMAX5541_DAC_EraseSector(uint32_t SectorAddr);
void sMAX5541_DAC_EraseBulk(void);
void sMAX5541_DAC_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sMAX5541_DAC_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sMAX5541_DAC_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sMAX5541_DAC_ReadID(void);
void sMAX5541_DAC_StartReadSequence(uint32_t ReadAddr);

/* Low layer functions */
uint8_t sMAX5541_DAC_ReadByte(void);
uint8_t sMAX5541_DAC_SendByte(uint8_t byte);
uint16_t sMAX5541_DAC_SendHalfWord(uint16_t HalfWord);
void sMAX5541_DAC_WriteEnable(void);
void sMAX5541_DAC_WaitForWriteEnd(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_MAX5541_DAC_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
