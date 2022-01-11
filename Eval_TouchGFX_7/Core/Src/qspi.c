/**
  ******************************************************************************
  * @file    stm32f7508_discovery_qspi.c
  * @author  MCD Application Team
  * @brief   This file includes a standard driver for the W25Q128 QSPI
  *          memory mounted on STM32F7508-Discovery board.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) This driver is used to drive the W25Q128 QSPI external
       memory mounted on STM32F7508-Discovery board.

   (#) This driver need a specific component driver (W25Q128) to be included with.
   (#) Initialization steps:
       (++) Initialize the QPSI external memory using the BSP_QSPI_Init() function. This
            function includes the MSP layer hardware resources initialization and the
            QSPI interface with the external memory.

   (#) QSPI memory operations
       (++) QSPI memory can be accessed with read/write operations once it is
            initialized.
            Read/write operation can be performed with AHB access using the functions
            BSP_QSPI_Fast_Read()/BSP_QSPI_Write().
       (++) The function BSP_QSPI_GetInfo() returns the configuration of the QSPI memory.
            (see the QSPI memory data sheet)
       (++) Perform erase block operation using the function BSP_QSPI_Erase_Block() and by
            specifying the block address. You can perform an erase operation of the whole
            chip by calling the function BSP_QSPI_Erase_Chip().
       (++) The function BSP_QSPI_GetStatus() returns the current status of the QSPI memory.
            (see the QSPI memory data sheet)
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Dependencies
- stm32f7xx_hal_qspi.c
- stm32f7xx_hal_gpio.c
- stm32f7xx_hal_cortex.c
- stm32f7xx_hal_rcc_ex.h
- W25Q128.h
EndDependencies */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "qspi.h"
#include "ExFlash.h"


/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F7508_DISCOVERY
  * @{
  */

/** @defgroup STM32F7508_DISCOVERY_QSPI STM32F7508_DISCOVERY_QSPI
  * @{
  */


/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32F7508_DISCOVERY_QSPI_Private_Variables STM32F7508_DISCOVERY QSPI Private Variables
  * @{
  */
extern QSPI_HandleTypeDef hqspi;

/**
  * @}
  */



/* Private functions ---------------------------------------------------------*/

/** @defgroup STM32F7508_DISCOVERY_QSPI_Private_Functions STM32F7508_DISCOVERY QSPI Private Functions
  * @{
  */
static uint8_t QSPI_ResetMemory          ( void );
static uint8_t QSPI_DummyCyclesCfg       ( void );
static uint8_t QSPI_WriteEnable          ( void );
static uint8_t QSPI_AutoPollingMemReady  ( uint32_t Timeout );

/**
  * @}
  */

/** @defgroup STM32F7508_DISCOVERY_QSPI_Exported_Functions STM32F7508_DISCOVERY QSPI Exported Functions
  * @{
  */

/**
  * @brief  Initializes the QSPI interface.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Init( void )
{  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  hqspi.Instance = QUADSPI;

  /* Call the DeInit function to reset the driver */
  if (HAL_QSPI_DeInit(&hqspi) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* System level initialization */

  HAL_QSPI_MspInit(&hqspi);

  /* QSPI initialization */
  hqspi.Init.ClockPrescaler     = 1; /* QSPI freq = 216 MHz/(1+1) = 108 Mhz */
  hqspi.Init.FifoThreshold      = 4;
  hqspi.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE; /* Min 50ns for nonRead */
  hqspi.Init.ClockMode          = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID            = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;
  
  if( flash_info.type == no_flash )
	  hqspi.Init.FlashSize      = POSITION_VAL(0x1000000) - 1;
  else
	  hqspi.Init.FlashSize      = POSITION_VAL(flash_info.size.total) - 1;


  if( HAL_QSPI_Init(&hqspi) != HAL_OK )
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  De-Initializes the QSPI interface.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_DeInit( )
{
  hqspi.Instance = QUADSPI;

  /* Call the DeInit function to reset the driver */
  if (HAL_QSPI_DeInit( &hqspi ) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* System level De-initialization */
  BSP_QSPI_MspDeInit( NULL );

  return QSPI_OK;
}

/**
  * @brief  Reads an amount of data from the QSPI memory.
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Fast_Read( uint8_t* pData, uint32_t ReadAddr, uint32_t Size )
{
  QSPI_CommandTypeDef s_command;

  if( flash_info.type == w25q128 )
  {
	  /* Initialize the read command */
	  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	  s_command.Instruction       = 0x6B;
	  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
	  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
	  s_command.Address           = ReadAddr;
	  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	  s_command.DataMode          = QSPI_DATA_4_LINES;
	  s_command.DummyCycles       = 8;
	  s_command.NbData            = Size;
	  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  }
  
  else if( flash_info.type == mx25l25645g )
  {
	  /* Initialize the read command */
	  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	  s_command.Instruction       = 0xEB;
	  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
	  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
	  s_command.Address           = ReadAddr;
	  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
	  s_command.AlternateBytesSize = 1;
	  s_command.AlternateBytes 	  = 0x00;
	  s_command.DataMode          = QSPI_DATA_4_LINES;
	  s_command.DummyCycles       = 4;
	  s_command.NbData            = Size;
	  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  }
  
  else
  {
	  return QSPI_ERROR;
  }

  /* Configure the command */
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Set S# timing for Read command */
  MODIFY_REG(hqspi.Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_3_CYCLE);

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Restore S# timing for nonRead commands */
  MODIFY_REG(hqspi.Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_6_CYCLE);

  return QSPI_OK;
}

/**
  * @brief  Writes an amount of data to the QSPI memory.
  * @param  pData: Pointer to data to be written
  * @param  WriteAddr: Write start address
  * @param  Size: Size of data to write
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Write( uint8_t* pData, uint32_t WriteAddr, uint32_t Size )
{
  QSPI_CommandTypeDef s_command;
  uint32_t end_addr, current_size, current_addr;

  /* Calculation of the size between the write address and the end of the page */
  current_size = flash_info.size.page - (WriteAddr % flash_info.size.page);

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > Size)
  {
    current_size = Size;
  }

  /* Initialize the adress variables */
  current_addr = WriteAddr;
  end_addr = WriteAddr + Size;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x32;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if( flash_info.type == w25q128 )
  	  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  else if( flash_info.type == mx25l25645g )
  	  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  else
  	  return QSPI_ERROR;

  /* Perform the write page by page */
  do
  {
    s_command.Address = current_addr;
    s_command.NbData  = current_size;

    /* Enable write operations */
    if( QSPI_WriteEnable( ) != QSPI_OK )
    {
      return QSPI_ERROR;
    }

    /* Configure the command */
    if( HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK )
    {
      return QSPI_ERROR;
    }

    /* Transmission of the data */
    if( HAL_QSPI_Transmit( &hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
    {
      return QSPI_ERROR;
    }

    /* Configure automatic polling mode to wait for end of program */
    if( QSPI_AutoPollingMemReady( HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != QSPI_OK )
    {
      return QSPI_ERROR;
    }

    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + flash_info.size.page) > end_addr) ? (end_addr - current_addr) : flash_info.size.page;
  } while (current_addr < end_addr);

  return QSPI_OK;
}

uint8_t BSP_QSPI_Erase_Sector( uint32_t SectorAddress )
{
  uint32_t timeout;
  QSPI_CommandTypeDef s_command;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x20;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.Address           = SectorAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if( flash_info.type == w25q128 )
  {
  	  s_command.AddressSize   = QSPI_ADDRESS_24_BITS;
  	  timeout = 400;
  }
  else if( flash_info.type == mx25l25645g )
  {
	  s_command.AddressSize   = QSPI_ADDRESS_32_BITS;
	  timeout = 400;
  }
  else
	  return QSPI_ERROR;

  /* Enable write operations */
  if( QSPI_WriteEnable( ) != QSPI_OK )
  {
    return QSPI_ERROR;
  }

  /* Send the command */
  if( HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if( QSPI_AutoPollingMemReady( timeout ) != QSPI_OK )
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Erases the specified block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Erase_Block( uint32_t BlockAddress )
{
  uint32_t timeout;
  QSPI_CommandTypeDef s_command;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0xD8;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.Address           = BlockAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if( flash_info.type == w25q128 )
  {
	  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
	  timeout = 3000;
  }
  else if( flash_info.type == mx25l25645g )
  {
  	  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  	  timeout = 2000;
  }
  else
  	  return QSPI_ERROR;

  /* Enable write operations */
  if( QSPI_WriteEnable( ) != QSPI_OK )
  {
    return QSPI_ERROR;
  }

  /* Send the command */
  if( HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if( QSPI_AutoPollingMemReady( timeout ) != QSPI_OK )
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Erases the entire QSPI memory.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Erase_Chip ( void )
{
  uint32_t timeout;
  QSPI_CommandTypeDef s_command;

  /* Initialize the erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0xC7;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if( flash_info.type == w25q128 )
  	  timeout = 250000;
  else if( flash_info.type == mx25l25645g )
  	  timeout = 220000;
  else
  	  return QSPI_ERROR;

  /* Enable write operations */
  if( QSPI_WriteEnable( ) != QSPI_OK )
  {
    return QSPI_ERROR;
  }

  /* Send the command */
  if(HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if( QSPI_AutoPollingMemReady( timeout ) != QSPI_OK )
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}


/**
  * @brief  Reads current status of the QSPI memory.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_GetStatus( void )
{
  QSPI_CommandTypeDef s_command;
  uint8_t reg;

  /* Initialize the read flag status register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x70;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Check the value of the register */
  if ((reg & (0x02 | 0x08 | 0x10 | 0x20)) != 0)
  {
    return QSPI_ERROR;
  }
  else if ((reg & (0x04 | 0x40)) != 0)
  {
    return QSPI_SUSPENDED;
  }
  else if ((reg & 0x80) != 0)
  {
    return QSPI_OK;
  }
  else
  {
    return QSPI_BUSY;
  }
}

/**
  * @brief  Return the configuration of the QSPI memory.
  * @param  pInfo: pointer on the configuration structure
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_GetInfo(QSPI_Info* pInfo)
{
  /* Configure the structure with the memory configuration */
  pInfo->FlashSize          = flash_info.size.total;
  pInfo->EraseSectorSize    = flash_info.size.subsector;
  pInfo->EraseSectorsNumber = ( flash_info.size.total / flash_info.size.subsector );
  pInfo->ProgPageSize       = flash_info.size.page;
  pInfo->ProgPagesNumber    = ( flash_info.size.total / flash_info.size.page );

  return QSPI_OK;
}

/**
  * @brief  Configure the QSPI in memory-mapped mode
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_EnableMemoryMappedMode( void )
{
  QSPI_CommandTypeDef      s_command;
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x6B;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 8;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if( flash_info.type == w25q128 )
  	  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  else if( flash_info.type == mx25l25645g )
  	  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  else
  	  return QSPI_ERROR;

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeOutPeriod     = 0;

  if (HAL_QSPI_MemoryMapped(&hqspi, &s_command, &s_mem_mapped_cfg) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @}
  */

/** @addtogroup STM32F7508_DISCOVERY_QSPI_Private_Functions
  * @{
  */

/**
  * @brief QSPI MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for QSPI interrupt
  * @retval None
  */
__weak void BSP_QSPI_MspInit( void *Params )
{
  GPIO_InitTypeDef gpio_init_structure;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable the QuadSPI memory interface clock */
  QSPI_CLK_ENABLE();
  /* Reset the QuadSPI memory interface */
  QSPI_FORCE_RESET();
  QSPI_RELEASE_RESET();
  /* Enable GPIO clocks */
  QSPI_CS_GPIO_CLK_ENABLE();
  QSPI_CLK_GPIO_CLK_ENABLE();
  QSPI_D0_GPIO_CLK_ENABLE();
  QSPI_D1_GPIO_CLK_ENABLE();
  QSPI_D2_GPIO_CLK_ENABLE();
  QSPI_D3_GPIO_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* QSPI CS GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_CS_PIN;
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_CS_GPIO_PORT, &gpio_init_structure);

  /* QSPI CLK GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_CLK_PIN;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_CLK_GPIO_PORT, &gpio_init_structure);

  /* QSPI D0 GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_D0_PIN;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D0_GPIO_PORT, &gpio_init_structure);

  /* QSPI D1 GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_D1_PIN;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D1_GPIO_PORT, &gpio_init_structure);

  /* QSPI D2 GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_D2_PIN;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_PORT, &gpio_init_structure);

  /* QSPI D3 GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_D3_PIN;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D3_GPIO_PORT, &gpio_init_structure);

  /*##-3- Configure the NVIC for QSPI #########################################*/
  /* NVIC configuration for QSPI interrupt */
  HAL_NVIC_SetPriority(QUADSPI_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(QUADSPI_IRQn);
}

/**
  * @brief QSPI MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @retval None
  */
__weak void BSP_QSPI_MspDeInit( void *Params )
{
  /*##-1- Disable the NVIC for QSPI ###########################################*/
  HAL_NVIC_DisableIRQ(QUADSPI_IRQn);

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-Configure QSPI pins */
  HAL_GPIO_DeInit(QSPI_CS_GPIO_PORT, QSPI_CS_PIN);
  HAL_GPIO_DeInit(QSPI_CLK_GPIO_PORT, QSPI_CLK_PIN);
  HAL_GPIO_DeInit(QSPI_D0_GPIO_PORT, QSPI_D0_PIN);
  HAL_GPIO_DeInit(QSPI_D1_GPIO_PORT, QSPI_D1_PIN);
  HAL_GPIO_DeInit(QSPI_D2_GPIO_PORT, QSPI_D2_PIN);
  HAL_GPIO_DeInit(QSPI_D3_GPIO_PORT, QSPI_D3_PIN);

  /*##-3- Reset peripherals ##################################################*/
  /* Reset the QuadSPI memory interface */
  QSPI_FORCE_RESET();
  QSPI_RELEASE_RESET();

  /* Disable the QuadSPI memory interface clock */
  QSPI_CLK_DISABLE();
}

/**
  * @brief  This function reset the QSPI memory.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint8_t QSPI_ResetMemory( void )
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reset enable command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x66;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  /* Send the command */
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the reset memory command */
  s_command.Instruction = 0x99;
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Configure automatic polling mode to wait for end of program */
  if( QSPI_AutoPollingMemReady( HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != QSPI_OK )
  {
    return QSPI_ERROR;
  }
  
  return QSPI_OK;
}

/**
  * @brief  This function configure the dummy cycles on memory side.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint8_t QSPI_DummyCyclesCfg( void )
{
  QSPI_CommandTypeDef s_command;
  uint8_t reg;

  /* Initialize the read volatile configuration register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x85;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive( &hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Enable write operations */
  if (QSPI_WriteEnable( ) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Update volatile configuration register (with new dummy cycles) */
  s_command.Instruction = 0x81;
  MODIFY_REG(reg, 0xF0, (10 << POSITION_VAL(0xF0)));

  /* Configure the write volatile configuration register command */
  if(HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK )
  {
    return QSPI_ERROR;
  }

  /* Transmission of the data */
  if( HAL_QSPI_Transmit(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK )
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint8_t QSPI_WriteEnable( void )
{
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  /* Enable write operations */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x06;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  s_config.Match           = 0x02;
  s_config.Mask            = 0x02;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;
  s_command.Instruction    = 0x05;
  s_command.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling( &hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function read the SR of the memory and wait the EOP.
  * @param  hqspi: QSPI handle
  * @param  Timeout
  * @retval None
  */
static uint8_t QSPI_AutoPollingMemReady( uint32_t Timeout )
{
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;
	
  /* Configure automatic polling mode to wait for memory ready */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x05;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  s_config.Match           = 0;
  s_config.Mask            = 0x01;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling( &hqspi, &s_command, &s_config, Timeout) != HAL_OK)
  {
	  return QSPI_ERROR;
  }

  return QSPI_OK;
}

uint8_t BSP_QSPI_Read_ID( uint8_t *buf )
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x9F;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 3;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, buf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  return QSPI_OK;
}

uint8_t BSP_QSPI_Enter4ByteMode( void )
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = 0xB7;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if( QSPI_WriteEnable( ) != QSPI_OK )
  {
	return QSPI_ERROR;
  }

  /* Configure the command */
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Configure automatic polling mode to wait for end of program */
  if( QSPI_AutoPollingMemReady( HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != QSPI_OK )
  {
	return QSPI_ERROR;
  }

  return QSPI_OK;
}

uint8_t BSP_QSPI_EnterQPIMode( void )
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x35;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  return QSPI_OK;
}

uint8_t BSP_QSPI_ExitQPIMode( void )
{  
  QSPI_CommandTypeDef s_command;

  /* Initialize the read command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = 0xF5;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  return QSPI_OK;
}

uint8_t BSP_QSPI_Reset( void )
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reset enable command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x66;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  /* Send the command */
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the reset memory command */
  s_command.Instruction = 0x99;
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  HAL_Delay( 150 );

  /* Initialize the reset enable command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = 0x66;
  /* Send the command */
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the reset memory command */
  s_command.Instruction = 0x99;
  if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  HAL_Delay( 150 );
  
  return QSPI_OK;
}

uint8_t Read_Status_Register ( uint8_t *d )
{
	QSPI_CommandTypeDef     s_command;

	/* Initialize the read command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.DummyCycles       = 0;
	s_command.NbData            = 1;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/******** Read Status Register-1 ********/
	s_command.Instruction       = 0x05;

	/* Configure the command */
	if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
	    return QSPI_ERROR;
	}

	if( HAL_QSPI_Receive( &hqspi, &d[0], 100 ) != HAL_OK )
	{
		return QSPI_ERROR;
	}

	/******** Read Status Register-2 ********/
	s_command.Instruction       = 0x35;

	/* Configure the command */
	if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
	    return QSPI_ERROR;
	}

	if( HAL_QSPI_Receive( &hqspi, &d[1], 100 ) != HAL_OK )
	{
		return QSPI_ERROR;
	}

	/******** Read Status Register-3 ********/
	s_command.Instruction       = 0x15;

	/* Configure the command */
	if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
	    return QSPI_ERROR;
	}

	if( HAL_QSPI_Receive( &hqspi, &d[2], 100 ) != HAL_OK )
	{
		return QSPI_ERROR;
	}
	return QSPI_OK;
}

uint8_t Write_Status_Register ( uint8_t *d )
{
	QSPI_CommandTypeDef     s_command;

	/* Initialize the read command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.DummyCycles       = 0;
	s_command.NbData            = 1;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/******** Write Status Register-1 ********/
    /* Enable write operations */
    if( QSPI_WriteEnable( ) != QSPI_OK )
    {
      return QSPI_ERROR;
    }

	s_command.Instruction       = 0x01;

	/* Configure the command */
	if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
	    return QSPI_ERROR;
	}

	if( HAL_QSPI_Transmit( &hqspi, &d[0], 100 ) != HAL_OK )
	{
		return QSPI_ERROR;
	}

    /* Configure automatic polling mode to wait for end of program */
    if( QSPI_AutoPollingMemReady( HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != QSPI_OK )
    {
      return QSPI_ERROR;
    }

	/******** Write Status Register-2 ********/
    /* Enable write operations */
    if( QSPI_WriteEnable( ) != QSPI_OK )
    {
      return QSPI_ERROR;
    }

	s_command.Instruction       = 0x31;

	/* Configure the command */
	if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
	    return QSPI_ERROR;
	}

	if( HAL_QSPI_Transmit( &hqspi, &d[1], 100 ) != HAL_OK )
	{
		return QSPI_ERROR;
	}

    /* Configure automatic polling mode to wait for end of program */
    if( QSPI_AutoPollingMemReady( HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != QSPI_OK )
    {
      return QSPI_ERROR;
    }

	/******** Write Status Register-3 ********/
    /* Enable write operations */
    if( QSPI_WriteEnable( ) != QSPI_OK )
    {
      return QSPI_ERROR;
    }

	s_command.Instruction       = 0x11;

	/* Configure the command */
	if (HAL_QSPI_Command( &hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
	    return QSPI_ERROR;
	}

	if( HAL_QSPI_Transmit( &hqspi, &d[2], 100 ) != HAL_OK )
	{
		return QSPI_ERROR;
	}

    /* Configure automatic polling mode to wait for end of program */
    if( QSPI_AutoPollingMemReady( HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != QSPI_OK )
    {
      return QSPI_ERROR;
    }

	return QSPI_OK;
}

uint8_t Check_Status_Register ( void )
{
	union{
		uint8_t buffer[ 3 ];
		struct{
			uint8_t BUSY :1; 	// S0
			uint8_t WEL :1;		// S1
			uint8_t BP0 :1; 	// S2
			uint8_t BP1 :1; 	// S3
			uint8_t BP2 :1; 	// S4
			uint8_t TB :1; 		// S5
			uint8_t SEC :1; 	// S6
			uint8_t SRP :1; 	// S7

			uint8_t SRL :1; 	// S8
			uint8_t QE :1;		// S9
			uint8_t :1; 		// S10
			uint8_t LB1 :1; 	// S11
			uint8_t LB2 :1; 	// S12
			uint8_t LB3 :1; 	// S13
			uint8_t CMP :1; 	// S14
			uint8_t SUS :1; 	// S15

			uint8_t :1; 		// S16
			uint8_t :1;			// S17
			uint8_t WPS :1; 	// S18
			uint8_t :1; 		// S19
			uint8_t :1; 		// S20
			uint8_t DR2 :1; 	// S21
			uint8_t DR1 :1; 	// S22
			uint8_t :1; 		// S23
		}resgister;
	}status;

	if( Read_Status_Register( &status.buffer[0] ) != QSPI_OK )
	{
		return QSPI_ERROR;
	}

	if( (status.resgister.CMP == 1) ||
		(status.resgister.BP0 == 1) ||
		(status.resgister.BP1 == 1) ||
		(status.resgister.BP2 == 1) )
	{
		status.resgister.CMP = 0;
		status.resgister.BP0 = 0;
		status.resgister.BP1 = 0;
		status.resgister.BP2 = 0;

		if( Write_Status_Register( &status.buffer[0] ) != QSPI_OK )
		{
			return QSPI_ERROR;
		}
	}

	if( Read_Status_Register( &status.buffer[0] ) != QSPI_OK )
	{
		return QSPI_ERROR;
	}

	return QSPI_OK;
}

/*Enable quad mode and set dummy cycles count and 4 Bytes mode*/
uint8_t QSPI_Configuration ( void )
{
	QSPI_CommandTypeDef s_command;
	QSPI_AutoPollingTypeDef s_config;
	uint8_t test_buffer[4] = { 0 };
	/*read status register*/
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = 0x05;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	s_command.NbData = 1;

	if( HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK )
	{
		return HAL_ERROR;
	}
	if( HAL_QSPI_Receive(&hqspi, test_buffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK )
	{
		return HAL_ERROR;
	}
	/*read configuration register*/
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = 0x15;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	s_command.NbData = 1;

	if( HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK )
	{
		return HAL_ERROR;
	}
	if( HAL_QSPI_Receive(&hqspi, &(test_buffer[1]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK )
	{
		return HAL_ERROR;
	}

	if( QSPI_WriteEnable( ) != QSPI_OK )
	{
	  return QSPI_ERROR;
	}

	/* modify buffer to enable quad mode */
	test_buffer[0] |= 0x40;

	/* modify buffer to close protection */
	test_buffer[0] &= 0xC3;

	/* set dummy cycles */
	test_buffer[1] = 0xC0;

	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressSize = QSPI_ADDRESS_32_BITS;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	s_command.Instruction = 0x01;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.DummyCycles = 0;
	s_command.NbData = 2;

	if( HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)	!= HAL_OK )
	{
		return HAL_ERROR;
	}

	if( HAL_QSPI_Transmit(&hqspi, test_buffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK )
	{
		return HAL_ERROR;
	}

	/* Configure automatic polling mode to wait for end of program */
	if( QSPI_AutoPollingMemReady( HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != QSPI_OK )
	{
		return QSPI_ERROR;
	}

	/* Configure automatic polling mode to wait for quad mode is enabled */
	s_command.Instruction       = 0x05;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DummyCycles       = 0;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	s_config.Match           = 0x40;
	s_config.Mask            = 0x40;
	s_config.MatchMode       = QSPI_MATCH_MODE_AND;
	s_config.StatusBytesSize = 1;
	s_config.Interval        = 0x10;
	s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling( &hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return QSPI_ERROR;
	}

	/* Configure automatic polling mode to wait for dummy cycles is set and 4-Bytes mode is enabled */
	s_command.Instruction       = 0x15;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DummyCycles       = 0;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	s_config.Match           = 0xC0;
	s_config.Mask            = 0xC0;
	s_config.MatchMode       = QSPI_MATCH_MODE_AND;
	s_config.StatusBytesSize = 1;
	s_config.Interval        = 0x10;
	s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling( &hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return QSPI_ERROR;
	}

	return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
