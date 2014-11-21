/*
 * sdram.c
 *
 *  Created on: 2014Äê11ÔÂ18ÈÕ
 *      Author: zli
 */
#include "sdram.h"
#include "stm32f4xx_hal_sdram.h"
#include "stm32f4xx_gpio.h"
#include "function.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_gpio.h"

static void SDRAM_MspInit(void);
void BSP_SDRAM_Initialization_sequence(uint32_t RefreshCount);

static SDRAM_HandleTypeDef sdramHandle;
static FMC_SDRAM_TimingTypeDef Timing;
static FMC_SDRAM_CommandTypeDef Command;

SDRAM_HandleTypeDef hsdram1;

uint8_t BSP_SDRAM_Init(void)
{
  static uint8_t sdramstatus = SDRAM_ERROR;
  /* SDRAM device configuration */
  sdramHandle.Instance = FMC_SDRAM_DEVICE;

  /* Timing configuration for 90Mhz as SD clock frequency (System clock is up to 180Mhz */
  Timing.LoadToActiveDelay    = 2;
  Timing.ExitSelfRefreshDelay = 6;
  Timing.SelfRefreshTime      = 4;
  Timing.RowCycleDelay        = 6;
  Timing.WriteRecoveryTime    = 2;
  Timing.RPDelay              = 2;
  Timing.RCDDelay             = 2;

  sdramHandle.Init.SDBank             = FMC_SDRAM_BANK1;
  sdramHandle.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;
  sdramHandle.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_11;
  sdramHandle.Init.MemoryDataWidth    = SDRAM_MEMORY_WIDTH;
  sdramHandle.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  sdramHandle.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
  sdramHandle.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  sdramHandle.Init.SDClockPeriod      = SDCLOCK_PERIOD;
  sdramHandle.Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
  sdramHandle.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0;

  /* SDRAM controller initialization */
  SDRAM_MspInit();
  if(HAL_SDRAM_Init(&sdramHandle, &Timing) != HAL_OK)
  {
    sdramstatus = SDRAM_ERROR;
  }
  else
  {
    sdramstatus = SDRAM_OK;
  }

  /* SDRAM initialization sequence */
  BSP_SDRAM_Initialization_sequence(REFRESH_COUNT);

  return sdramstatus;
}

void BSP_SDRAM_Initialization_sequence(uint32_t RefreshCount)
{
  __IO uint32_t tmpmrd = 0;

  /* Step 1: Configure a clock configuration enable command */
  Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
//  HAL_Delay(1);
	_delay_ms(10);

  /* Step 3: Configure a PALL (precharge all) command */
  Command.CommandMode            = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 4: Configure an Auto Refresh command */
  Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 4;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |\
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
                     SDRAM_MODEREG_CAS_LATENCY_3           |\
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(&sdramHandle, RefreshCount);
}



static void SDRAM_MspInit(void)
{
  static DMA_HandleTypeDef dmaHandle;
  GPIO_InitTypeDef GPIO_Init_Structure;
  SDRAM_HandleTypeDef  *hsdram = &sdramHandle;

  /* Enable FMC clock */
  __FMC_CLK_ENABLE();

  /* Enable chosen DMAx clock */
  __DMAx_CLK_ENABLE();

  /* Enable GPIOs clock */
  __GPIOD_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOI_CLK_ENABLE();


//  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
//  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
//  GPIO_Init_Structure.Speed     = GPIO_SPEED_FAST;
//  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;

  /* Common GPIO configuration */
  GPIO_Init_Structure.GPIO_Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.GPIO_PuPd      = GPIO_PULLUP;
  GPIO_Init_Structure.GPIO_Speed     = GPIO_SPEED_FAST;
//  GPIO_Init_Structure.GPIO_OType = GPIO_AF12_FMC; GPIO_AF_FMC

  /* GPIOD configuration */

  GPIO_Init_Structure.GPIO_Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8| GPIO_PIN_9 | GPIO_PIN_10 |\
                              GPIO_PIN_14 | GPIO_PIN_15;

  GPIOD->AFR[0]=0x000000cc;GPIOD->AFR[1]=0xcc000ccc;
  GPIO_Init(GPIOD, &GPIO_Init_Structure);
  //GPIO_PinAFConfig(GPIOD,GPIO_Init_Structure.GPIO_Pin,GPIO_AF_FMC);
  /* GPIOE configuration */
  GPIO_Init_Structure.GPIO_Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9 |\
                              GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                              GPIO_PIN_15;

  GPIOE->AFR[0]=0xc00000cc;GPIOE->AFR[1]=0xcccccccc;
  GPIO_Init(GPIOE, &GPIO_Init_Structure);
  //GPIO_PinAFConfig(GPIOE,GPIO_Init_Structure.GPIO_Pin,GPIO_AF_FMC);
  /* GPIOF configuration */
  GPIO_Init_Structure.GPIO_Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4 |\
                              GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                              GPIO_PIN_15;
  GPIOF->AFR[0]=0x0000ccccc;GPIOF->AFR[1]=0xccccc000;
  GPIO_Init(GPIOF, &GPIO_Init_Structure);
  //GPIO_PinAFConfig(GPIOF,GPIO_Init_Structure.GPIO_Pin,GPIO_AF_FMC);
  /* GPIOG configuration */
  GPIO_Init_Structure.GPIO_Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4| GPIO_PIN_5 | GPIO_PIN_8 |\
                              GPIO_PIN_15;
  GPIOG->AFR[0]=0x00cc00cc;GPIOG->AFR[1]=0xc000000c;
  GPIO_Init(GPIOG, &GPIO_Init_Structure);
  //GPIO_PinAFConfig(GPIOG,GPIO_Init_Structure.GPIO_Pin,GPIO_AF_FMC);
  /* GPIOH configuration */
  GPIO_Init_Structure.GPIO_Pin   = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 |\
                              GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                              GPIO_PIN_15;
  GPIOH->AFR[0]=0x00c0cc00;GPIOG->AFR[1]=0xcccccccc;
  GPIO_Init(GPIOH, &GPIO_Init_Structure);
  //GPIO_PinAFConfig(GPIOH,GPIO_Init_Structure.GPIO_Pin,GPIO_AF_FMC);
  /* GPIOI configuration */
  GPIO_Init_Structure.GPIO_Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |\
                              GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIOI->AFR[0]=0xcccccccc;GPIOI->AFR[1]=0x00000cc0;
  GPIO_Init(GPIOI, &GPIO_Init_Structure);
  //GPIO_PinAFConfig(GPIOI,GPIO_Init_Structure.GPIO_Pin,GPIO_AF_FMC);
  /* Configure common DMA parameters */
  dmaHandle.Init.Channel             = SDRAM_DMAx_CHANNEL;
  dmaHandle.Init.Direction           = DMA_MEMORY_TO_MEMORY;
  dmaHandle.Init.PeriphInc           = DMA_PINC_ENABLE;
  dmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  dmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dmaHandle.Init.Mode                = DMA_NORMAL;
  dmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
  dmaHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  dmaHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dmaHandle.Init.MemBurst            = DMA_MBURST_SINGLE;
  dmaHandle.Init.PeriphBurst         = DMA_PBURST_SINGLE;

  dmaHandle.Instance = SDRAM_DMAx_STREAM;

   /* Associate the DMA handle */
  __HAL_LINKDMA(hsdram, hdma, dmaHandle);

  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dmaHandle);

  /* Configure the DMA stream */
  HAL_DMA_Init(&dmaHandle);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SDRAM_DMAx_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SDRAM_DMAx_IRQn);//NVIC_EnableIRQ(SDRAM_DMAx_IRQn);//

}
