/**
  ******************************************************************************
  * @file    SPI/SPI_DAC_ADC/spi_DAC_ADC.c
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   This file provides a set of functions needed to manage the SPI M25Pxxx
  *          DAC_ADC memory.
  *            
  *          ===================================================================      
  *          Notes: 
  *           - There is no SPI DAC_ADC memory available in STM324xG-EVAL board,
  *             to use this driver you have to build your own hardware.     
  *          ===================================================================
  *   
  *          It implements a high level communication layer for read and write 
  *          from/to this memory. The needed STM32 hardware resources (SPI and 
  *          GPIO) are defined in spi_DAC_ADC.h file, and the initialization is
  *          performed in sDAC_ADC_LowLevel_Init() function.
  *            
  *          You can easily tailor this driver to any development board, by just
  *          adapting the defines for hardware resources and sDAC_ADC_LowLevel_Init()
  *          function.
  *            
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+---------------+-------------+
  *          |  STM32 SPI Pins             |     sDAC_ADC    |    Pin      |
  *          +-----------------------------+---------------+-------------+
  *          | sDAC_ADC_CS_PIN               | ChipSelect(/S)|    1        |
  *          | sDAC_ADC_SPI_MISO_PIN / MISO  |   DataOut(Q)  |    2        |
  *          |                             |   VCC         |    3 (3.3 V)|
  *          |                             |   GND         |    4 (0 V)  |
  *          | sDAC_ADC_SPI_MOSI_PIN / MOSI  |   DataIn(D)   |    5        |
  *          | sDAC_ADC_SPI_SCK_PIN / SCK    |   Clock(C)    |    6        |
  *          |                             |    VCC        |    7 (3.3 V)|
  *          |                             |    VCC        |    8 (3.3 V)|  
  *          +-----------------------------+---------------+-------------+  
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

/* Includes ------------------------------------------------------------------*/
#include "spi_DAC_ADC.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup SPI_DAC_ADC
  * @{
  */  

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void sDAC_ADC_LowLevel_DeInit(void);
void sDAC_ADC_LowLevel_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  DeInitializes the peripherals used by the SPI DAC_ADC driver.
  * @param  None
  * @retval None
  */
void sDAC_ADC_DeInit(void)
{
  sDAC_ADC_LowLevel_DeInit();
}

/**
  * @brief  Initializes the peripherals used by the SPI DAC_ADC driver.
  * @param  None
  * @retval None
  */
void sDAC_ADC_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  sDAC_ADC_LowLevel_Init();//(RCC_APB1Periph_SPI2,ENABLE);
    
  /*!< Deselect the DAC_ADC: Chip Select high */
  sDAC_ADC_CS_HIGH();

  /*!< SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;

  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(sDAC_ADC_SPI, &SPI_InitStructure);

  /*!< Enable the sDAC_ADC_SPI  */
  SPI_Cmd(sDAC_ADC_SPI, ENABLE);
}

/**
  * @brief  Erases the specified DAC_ADC sector.
  * @param  SectorAddr: address of the sector to erase.
  * @retval None
  */
void sDAC_ADC_EraseSector(uint32_t SectorAddr)
{
  /*!< Send write enable instruction */
  sDAC_ADC_WriteEnable();

  /*!< Sector Erase */
  /*!< Select the DAC_ADC: Chip Select low */
  sDAC_ADC_CS_LOW();
  /*!< Send Sector Erase instruction */
  sDAC_ADC_SendByte(sDAC_ADC_CMD_SE);
  /*!< Send SectorAddr high nibble address byte */
  sDAC_ADC_SendByte((SectorAddr & 0xFF0000) >> 16);
  /*!< Send SectorAddr medium nibble address byte */
  sDAC_ADC_SendByte((SectorAddr & 0xFF00) >> 8);
  /*!< Send SectorAddr low nibble address byte */
  sDAC_ADC_SendByte(SectorAddr & 0xFF);
  /*!< Deselect the DAC_ADC: Chip Select high */
  sDAC_ADC_CS_HIGH();

  /*!< Wait the end of DAC_ADC writing */
  sDAC_ADC_WaitForWriteEnd();
}

/**
  * @brief  Erases the entire DAC_ADC.
  * @param  None
  * @retval None
  */
void sDAC_ADC_EraseBulk(void)
{
  /*!< Send write enable instruction */
  sDAC_ADC_WriteEnable();

  /*!< Bulk Erase */
  /*!< Select the DAC_ADC: Chip Select low */
  sDAC_ADC_CS_LOW();
  /*!< Send Bulk Erase instruction  */
  sDAC_ADC_SendByte(sDAC_ADC_CMD_BE);
  /*!< Deselect the DAC_ADC: Chip Select high */
  sDAC_ADC_CS_HIGH();

  /*!< Wait the end of DAC_ADC writing */
  sDAC_ADC_WaitForWriteEnd();
}

/**
  * @brief  Writes more than one byte to the DAC_ADC with a single WRITE cycle
  *         (Page WRITE sequence).
  * @note   The number of byte can't exceed the DAC_ADC page size.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the DAC_ADC.
  * @param  WriteAddr: DAC_ADC's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the DAC_ADC, must be equal
  *         or less than "sDAC_ADC_PAGESIZE" value.
  * @retval None
  */
void sDAC_ADC_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  /*!< Enable the write access to the DAC_ADC */
  sDAC_ADC_WriteEnable();

  /*!< Select the DAC_ADC: Chip Select low */
  sDAC_ADC_CS_LOW();
  /*!< Send "Write to Memory " instruction */
  sDAC_ADC_SendByte(sDAC_ADC_CMD_WRITE);
  /*!< Send WriteAddr high nibble address byte to write to */
  sDAC_ADC_SendByte((WriteAddr & 0xFF0000) >> 16);
  /*!< Send WriteAddr medium nibble address byte to write to */
  sDAC_ADC_SendByte((WriteAddr & 0xFF00) >> 8);
  /*!< Send WriteAddr low nibble address byte to write to */
  sDAC_ADC_SendByte(WriteAddr & 0xFF);

  /*!< while there is data to be written on the DAC_ADC */
  while (NumByteToWrite--)
  {
    /*!< Send the current byte */
    sDAC_ADC_SendByte(*pBuffer);
    /*!< Point on the next byte to be written */
    pBuffer++;
  }

  /*!< Deselect the DAC_ADC: Chip Select high */
  sDAC_ADC_CS_HIGH();

  /*!< Wait the end of DAC_ADC writing */
  sDAC_ADC_WaitForWriteEnd();
}

/**
  * @brief  Writes block of data to the DAC_ADC. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the DAC_ADC.
  * @param  WriteAddr: DAC_ADC's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the DAC_ADC.
  * @retval None
  */
void sDAC_ADC_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % sDAC_ADC_SPI_PAGESIZE;
  count = sDAC_ADC_SPI_PAGESIZE - Addr;
  NumOfPage =  NumByteToWrite / sDAC_ADC_SPI_PAGESIZE;
  NumOfSingle = NumByteToWrite % sDAC_ADC_SPI_PAGESIZE;

  if (Addr == 0) /*!< WriteAddr is sDAC_ADC_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< NumByteToWrite < sDAC_ADC_PAGESIZE */
    {
      sDAC_ADC_WritePage(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /*!< NumByteToWrite > sDAC_ADC_PAGESIZE */
    {
      while (NumOfPage--)
      {
        sDAC_ADC_WritePage(pBuffer, WriteAddr, sDAC_ADC_SPI_PAGESIZE);
        WriteAddr +=  sDAC_ADC_SPI_PAGESIZE;
        pBuffer += sDAC_ADC_SPI_PAGESIZE;
      }

      sDAC_ADC_WritePage(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /*!< WriteAddr is not sDAC_ADC_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< NumByteToWrite < sDAC_ADC_PAGESIZE */
    {
      if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > sDAC_ADC_PAGESIZE */
      {
        temp = NumOfSingle - count;

        sDAC_ADC_WritePage(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        sDAC_ADC_WritePage(pBuffer, WriteAddr, temp);
      }
      else
      {
        sDAC_ADC_WritePage(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /*!< NumByteToWrite > sDAC_ADC_PAGESIZE */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / sDAC_ADC_SPI_PAGESIZE;
      NumOfSingle = NumByteToWrite % sDAC_ADC_SPI_PAGESIZE;

      sDAC_ADC_WritePage(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        sDAC_ADC_WritePage(pBuffer, WriteAddr, sDAC_ADC_SPI_PAGESIZE);
        WriteAddr +=  sDAC_ADC_SPI_PAGESIZE;
        pBuffer += sDAC_ADC_SPI_PAGESIZE;
      }

      if (NumOfSingle != 0)
      {
        sDAC_ADC_WritePage(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

/**
  * @brief  Reads a block of data from the DAC_ADC.
  * @param  pBuffer: pointer to the buffer that receives the data read from the DAC_ADC.
  * @param  ReadAddr: DAC_ADC's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the DAC_ADC.
  * @retval None
  */
void sDAC_ADC_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  /*!< Select the DAC_ADC: Chip Select low */
  sDAC_ADC_CS_LOW();

  /*!< Send "Read from Memory " instruction */
  sDAC_ADC_SendByte(sDAC_ADC_CMD_READ);

  /*!< Send ReadAddr high nibble address byte to read from */
  sDAC_ADC_SendByte((ReadAddr & 0xFF0000) >> 16);
  /*!< Send ReadAddr medium nibble address byte to read from */
  sDAC_ADC_SendByte((ReadAddr& 0xFF00) >> 8);
  /*!< Send ReadAddr low nibble address byte to read from */
  sDAC_ADC_SendByte(ReadAddr & 0xFF);

  while (NumByteToRead--) /*!< while there is data to be read */
  {
    /*!< Read a byte from the DAC_ADC */
    *pBuffer = sDAC_ADC_SendByte(sDAC_ADC_DUMMY_BYTE);
    /*!< Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /*!< Deselect the DAC_ADC: Chip Select high */
  sDAC_ADC_CS_HIGH();
}

/**
  * @brief  Reads DAC_ADC identification.
  * @param  None
  * @retval DAC_ADC identification
  */
uint32_t sDAC_ADC_ReadID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  /*!< Select the DAC_ADC: Chip Select low */
  sDAC_ADC_CS_LOW();

  /*!< Send "RDID " instruction */
  sDAC_ADC_SendByte(0x9F);

  /*!< Read a byte from the DAC_ADC */
  Temp0 = sDAC_ADC_SendByte(sDAC_ADC_DUMMY_BYTE);

  /*!< Read a byte from the DAC_ADC */
  Temp1 = sDAC_ADC_SendByte(sDAC_ADC_DUMMY_BYTE);

  /*!< Read a byte from the DAC_ADC */
  Temp2 = sDAC_ADC_SendByte(sDAC_ADC_DUMMY_BYTE);

  /*!< Deselect the DAC_ADC: Chip Select high */
  sDAC_ADC_CS_HIGH();

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

  return Temp;
}

/**
  * @brief  Initiates a read data byte (READ) sequence from the DAC_ADC.
  *   This is done by driving the /CS line low to select the device, then the READ
  *   instruction is transmitted followed by 3 bytes address. This function exit
  *   and keep the /CS line low, so the DAC_ADC still being selected. With this
  *   technique the whole content of the DAC_ADC is read with a single READ instruction.
  * @param  ReadAddr: DAC_ADC's internal address to read from.
  * @retval None
  */
void sDAC_ADC_StartReadSequence(uint32_t ReadAddr)
{
  /*!< Select the DAC_ADC: Chip Select low */
  sDAC_ADC_CS_LOW();

  /*!< Send "Read from Memory " instruction */
  sDAC_ADC_SendByte(sDAC_ADC_CMD_READ);

  /*!< Send the 24-bit address of the address to read from -------------------*/
  /*!< Send ReadAddr high nibble address byte */
  sDAC_ADC_SendByte((ReadAddr & 0xFF0000) >> 16);
  /*!< Send ReadAddr medium nibble address byte */
  sDAC_ADC_SendByte((ReadAddr& 0xFF00) >> 8);
  /*!< Send ReadAddr low nibble address byte */
  sDAC_ADC_SendByte(ReadAddr & 0xFF);
}

/**
  * @brief  Reads a byte from the SPI DAC_ADC.
  * @note   This function must be used only if the Start_Read_Sequence function
  *         has been previously called.
  * @param  None
  * @retval Byte Read from the SPI DAC_ADC.
  */
uint8_t sDAC_ADC_ReadByte(void)
{
  return (sDAC_ADC_SendByte(sDAC_ADC_DUMMY_BYTE));
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t sDAC_ADC_SendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(sDAC_ADC_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(sDAC_ADC_SPI, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(sDAC_ADC_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(sDAC_ADC_SPI);
}

/**
  * @brief  Sends a Half Word through the SPI interface and return the Half Word
  *         received from the SPI bus.
  * @param  HalfWord: Half Word to send.
  * @retval The value of the received Half Word.
  */
uint16_t sDAC_ADC_SendHalfWord(uint16_t HalfWord)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(sDAC_ADC_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send Half Word through the sDAC_ADC peripheral */
  SPI_I2S_SendData(sDAC_ADC_SPI, HalfWord);

  /*!< Wait to receive a Half Word */
  while (SPI_I2S_GetFlagStatus(sDAC_ADC_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the Half Word read from the SPI bus */
  return SPI_I2S_ReceiveData(sDAC_ADC_SPI);
}

/**
  * @brief  Enables the write access to the DAC_ADC.
  * @param  None
  * @retval None
  */
void sDAC_ADC_WriteEnable(void)
{
  /*!< Select the DAC_ADC: Chip Select low */
  sDAC_ADC_CS_LOW();

  /*!< Send "Write Enable" instruction */
  sDAC_ADC_SendByte(sDAC_ADC_CMD_WREN);

  /*!< Deselect the DAC_ADC: Chip Select high */
  sDAC_ADC_CS_HIGH();
}

/**
  * @brief  Polls the status of the Write In Progress (WIP) flag in the DAC_ADC's
  *         status register and loop until write opertaion has completed.
  * @param  None
  * @retval None
  */
void sDAC_ADC_WaitForWriteEnd(void)
{
  uint8_t DAC_ADCstatus = 0;

  /*!< Select the DAC_ADC: Chip Select low */
  sDAC_ADC_CS_LOW();

  /*!< Send "Read Status Register" instruction */
  sDAC_ADC_SendByte(sDAC_ADC_CMD_RDSR);

  /*!< Loop as long as the memory is busy with a write cycle */
  do
  {
    /*!< Send a dummy byte to generate the clock needed by the DAC_ADC
    and put the value of the status register in DAC_ADC_Status variable */
    DAC_ADCstatus = sDAC_ADC_SendByte(sDAC_ADC_DUMMY_BYTE);

  }
  while ((DAC_ADCstatus & sDAC_ADC_WIP_FLAG) == SET); /* Write in progress */

  /*!< Deselect the DAC_ADC: Chip Select high */
  sDAC_ADC_CS_HIGH();
}

/**
  * @brief  Initializes the peripherals used by the SPI DAC_ADC driver.
  * @param  None
  * @retval None
  */
void sDAC_ADC_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< Enable the SPI clock */
  sDAC_ADC_SPI_CLK_INIT(sDAC_ADC_SPI_CLK, ENABLE);

  /*!< Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(sDAC_ADC_SPI_SCK_GPIO_CLK | sDAC_ADC_SPI_MISO_GPIO_CLK |
                         sDAC_ADC_SPI_MOSI_GPIO_CLK | sDAC_ADC_CS_GPIO_CLK, ENABLE);
  
  /*!< SPI pins configuration *************************************************/

  /*!< Connect SPI pins to AF5 */  
  GPIO_PinAFConfig(sDAC_ADC_SPI_SCK_GPIO_PORT, sDAC_ADC_SPI_SCK_SOURCE, sDAC_ADC_SPI_SCK_AF);
  GPIO_PinAFConfig(sDAC_ADC_SPI_MISO_GPIO_PORT, sDAC_ADC_SPI_MISO_SOURCE, sDAC_ADC_SPI_MISO_AF);
  GPIO_PinAFConfig(sDAC_ADC_SPI_MOSI_GPIO_PORT, sDAC_ADC_SPI_MOSI_SOURCE, sDAC_ADC_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
        
  /*!< SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = sDAC_ADC_SPI_SCK_PIN;
  GPIO_Init(sDAC_ADC_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /*!< SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  sDAC_ADC_SPI_MOSI_PIN;
  GPIO_Init(sDAC_ADC_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*!< SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin =  sDAC_ADC_SPI_MISO_PIN;
  GPIO_Init(sDAC_ADC_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sDAC_ADC Card CS pin in output pushpull mode ********************/
  GPIO_InitStructure.GPIO_Pin = sDAC_ADC_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(sDAC_ADC_CS_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  DeInitializes the peripherals used by the SPI DAC_ADC driver.
  * @param  None
  * @retval None
  */
void sDAC_ADC_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< Disable the sDAC_ADC_SPI  ************************************************/
  SPI_Cmd(sDAC_ADC_SPI, DISABLE);
  
  /*!< DeInitializes the sDAC_ADC_SPI *******************************************/
  SPI_I2S_DeInit(sDAC_ADC_SPI);
  
  /*!< sDAC_ADC_SPI Periph clock disable ****************************************/
  sDAC_ADC_SPI_CLK_INIT(sDAC_ADC_SPI_CLK, DISABLE);
      
  /*!< Configure all pins used by the SPI as input floating *******************/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_InitStructure.GPIO_Pin = sDAC_ADC_SPI_SCK_PIN;
  GPIO_Init(sDAC_ADC_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = sDAC_ADC_SPI_MISO_PIN;
  GPIO_Init(sDAC_ADC_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = sDAC_ADC_SPI_MOSI_PIN;
  GPIO_Init(sDAC_ADC_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = sDAC_ADC_CS_PIN;
  GPIO_Init(sDAC_ADC_CS_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
