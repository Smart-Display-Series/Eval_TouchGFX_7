/**
  ******************************************************************************
  * File Name          : STM32TouchController.cpp
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* USER CODE BEGIN STM32TouchController */

#include <STM32TouchController.hpp>
#include "main.h"

extern "C" I2C_HandleTypeDef hi2c1;

void STM32TouchController::init()
{
    /**
     * Initialize touch controller and driver
     *
     */
	TP.type = CTP;
	TP.driver = unknown;
	TP.range.x = 0;
	TP.range.y = 0;
	TP.version = 0;

	HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin, GPIO_PIN_RESET); // CTP reset pin set low
	HAL_Delay( 20 );
	HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin, GPIO_PIN_SET); // CTP reset pin set high
	HAL_Delay( 20 );

	getDriver( );
	getVersion( );
	getRange( );
}

bool STM32TouchController::sampleTouch(int32_t& x, int32_t& y)
{
    /**
     * By default sampleTouch returns false,
     * return true if a touch has been detected, otherwise false.
     *
     * Coordinates are passed to the caller by reference by x and y.
     *
     * This function is called by the TouchGFX framework.
     * By default sampleTouch is called every tick, this can be adjusted by HAL::setTouchSampleRate(int8_t);
     *
     */

	switch( TP.type )
	{
		//====================================================================//
		case RTP:
			break;
		//====================================================================//
		case CTP:

			if( TP.driver == unknown )
			{
				getDriver( );
				return false;
			}
			else if( TP.version == 0 )
			{
				getVersion( );
				return false;
			}
			else if( (TP.range.x == 0) || (TP.range.y == 0) )
			{
				getRange( );
				return false;
			}

			if( TP.driver == FT5426 )
				return FT5426_Read( x, y );
			else if( TP.driver == ILI2130 )
				return ILI2130_Read( x, y );
			else if( TP.driver == ILI2511 )
				return ILI2511_Read( x, y );
			else {}

			break;
		//====================================================================//
		default:
			break;
		//====================================================================//
	}
    return false;
}

void STM32TouchController::getDriver ( void )
{
	uint8_t Buf[ 10 ];

	TP.driver = unknown;

	if( HAL_I2C_Mem_Read( &hi2c1, 0x70, 0xA3, I2C_MEMADD_SIZE_8BIT, &Buf[0], 1, 10 ) == HAL_OK )
	{
		if( Buf[0] == 0x0A ) // FT5x16
		{
		}
		else if( Buf[0] == 0x54 )
			TP.driver = FT5426;
		else if( Buf[0] == 0x55 ) // FT5x06
		{
		}
		else {}
	}
	else if( HAL_I2C_Mem_Read( &hi2c1, 0x82, 0x40, I2C_MEMADD_SIZE_8BIT, &Buf[0], 8, 10 ) == HAL_OK )
	{
		if( Buf[0] == 0x06 )
			TP.driver = ILI2511;
		if( Buf[0] == 0x07 )
			TP.driver = ILI2130;
	}
	else {}

}

void STM32TouchController::getRange ( void )
{
	uint8_t Buf[ 15 ];

	switch( TP.driver )
	{
		case FT5426:
			TP.range.x = 1792;
			TP.range.y = 1024;
			break;
		case ILI2130:
			if( HAL_I2C_Mem_Read( &hi2c1, 0x82, 0x20, I2C_MEMADD_SIZE_8BIT, &Buf[0], 10, 10 ) != HAL_OK )
				return;

			if( (Buf[8] == 0) || (Buf[8] > 32) )
				return;

			TP.range.x = *(uint16_t*)&Buf[0];
			TP.range.y = *(uint16_t*)&Buf[2];
			break;
		case ILI2511:
			if( HAL_I2C_Mem_Read( &hi2c1, 0x82, 0x20, I2C_MEMADD_SIZE_8BIT, &Buf[0], 10, 10 ) != HAL_OK )
				return;

			if( (Buf[6] == 0) || (Buf[6] == 255) )
				return;

			TP.range.x = *(uint16_t*)&Buf[0];
			TP.range.y = *(uint16_t*)&Buf[2];
			break;
		default:
			return;
	}
}

void STM32TouchController::getVersion ( void )
{
	uint8_t Buf[ 10 ];
	switch( TP.driver )
	{
		case FT5426:
			if( HAL_I2C_Mem_Read( &hi2c1, 0x70, 0xA1, I2C_MEMADD_SIZE_8BIT, &Buf[0], 2, 10 ) == HAL_OK )
			{
				TP.version = ( Buf[0] << 8 ) + Buf[1];
			}
			break;
		case ILI2130:
			if( HAL_I2C_Mem_Read( &hi2c1, 0x82, 0x42, I2C_MEMADD_SIZE_8BIT, &Buf[0], 3, 10 ) == HAL_OK )
			{
				if( (0 < Buf[ 0 ]) && (Buf[ 0 ] < 255) )
					TP.version = Buf[0];
			}
			break;
		case ILI2511:
			if( HAL_I2C_Mem_Read( &hi2c1, 0x82, 0x42, I2C_MEMADD_SIZE_8BIT, &Buf[0], 3, 10 ) == HAL_OK )
			{
				if( (0 < Buf[ 0 ]) && (Buf[ 0 ] < 255) )
					TP.version = Buf[0];
			}
			break;
		default:
			return;
	}
}

bool STM32TouchController::FT5426_Read ( int32_t& x, int32_t& y )
{
		// +------+-----------+--------+--------+--------+--------+--------+--------+--------+--------+
		// | Addr |    Name   |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
		// +------+-----------+--------+--------+--------+--------+--------+--------+--------+--------+
		// | 0x02 | Cur Point |      			  Number of touch points[7:0]  						  |
		// +------+-----------+-----------------+--------+--------+-----------------------------------+
		// | 0x03 | TOUCH1_XH | 1st Event Flag  |        |		  | 	1st Touch X Position[11:8]    |
		// +------+-----------+-----------------+--------+--------+-----------------------------------+
		// | 0x04 | TOUCH1_XL | 				  1st Touch X Position[7:0]							  |
		// +------+-----------+-----------------------------------------------------------------------+
		// | 0x05 | TOUCH1_YH | 	  1st Touch ID[3:0]			  |     1st Touch Y Position[11:8]    |
		// +------+-----------+-----------------------------------+-----------------------------------+
		// | 0x06 | TOUCH1_YL | 				  1st Touch Y Position[7:0]							  |
		// +------+-----------+-----------------------------------------------------------------------+

		#pragma pack( 1 )
		union{
			uint8_t Buf[ 5 ];
			struct{
				/*-------- Byte0 --------*/
				uint8_t point_number :8;
				/*-------- Byte1 --------*/
				uint8_t X_H :4;
				uint8_t :2;
				uint8_t Event :2;
				/*-------- Byte2 --------*/
				uint8_t X_L :8;
				/*-------- Byte3 --------*/
				uint8_t Y_H :4;
				uint8_t ID :4;
				/*-------- Byte4 --------*/
				uint8_t Y_L :8;
			}reg;
		}Data;
		#pragma pack( )

		if(HAL_I2C_Mem_Read( &hi2c1, 0x70, 0x02, I2C_MEMADD_SIZE_8BIT, &Data.Buf[0], 5, 10 ) != HAL_OK)
			return false;

		if( Data.reg.ID != 0 )
			return false;

		if( ( Data.reg.Event != 0x00 ) && ( Data.reg.Event != 0x02 ) )
			return false;

		uint32_t sx,sy;

		sx = ((Data.reg.X_H & 0x0F) << 8) | Data.reg.X_L;
		sy = ((Data.reg.Y_H & 0x0F) << 8) | Data.reg.Y_L;
		x = (sx * 1024) / TP.range.x;
		y = (sy * 600) / TP.range.y;

		return true;
}

bool STM32TouchController::ILI2130_Read ( int32_t& x, int32_t& y )
{
	// I2C Packet Format
	// +-----------------+---------------- +---------------------------------+
	// | Number of Bytes |	 Description   |              Notes              |
	// +-----------------+-----------------+---------------------------------+
	// | 	   0		 |    Report ID    | I2C = 0x48                      |
	// +-----------------+-----------------+---------------------------------+
	// |       1~5       |     Point 1     | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       6~10      |     Point 2     | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       11~15     |     Point 3     | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       16~20     |     Point 4     | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       21~25     |     Point 5     | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       26~30     |     Point 6     | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       31~35     |     Point 7     | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       36~40     |     Point 8     | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       41~45     |     Point 9     | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       46~50     |     Point 10    | Please refer Report ID Format   |
	// +-----------------+-----------------+---------------------------------+
	// |       51~60     |     Reserve     |                                 |
	// +-----------------+-----------------+---------------------------------+
	// |       61        |  Contact Count  |                                 |
	// +-----------------+-----------------+---------------------------------+
	// |       62        |     Reserve     |                                 |
	// +-----------------+-----------------+---------------------------------+
	// |       63        |    Check Sum    |                                 |
	// +-----------------+-----------------+---------------------------------+

	// Report ID Format
	// +-----------------+-------------------------+------+------+------+------+------+------+------+------+
	// | Number of Bytes |	     Description       |  b7  |  b6  |  b5  |  b4  |  b3  |  b2  |  b1  |  b0  |
	// +-----------------+-------------------------+------+------+------+------+------+------+------+------+
	// |        1        | Tip Switch and Point ID |  0   | Tip  |                 Point ID                |
	// +-----------------+-------------------------+------+------+------+------+------+------+------+------+
	// |        2        |                         | X Position (LSB)                                      |
	// +-----------------+ X direction coordinate  +------+------+------+------+------+------+------+------+
	// |        3        |                         | X Position (MSB)                                      |
	// +-----------------+-------------------------+------+------+------+------+------+------+------+------+
	// |        4        |                         | Y Position (LSB)                                      |
	// +-----------------+ Y direction coordinate  +------+------+------+------+------+------+------+------+
	// |        5        |                         | Y Position (MSB)                                      |
	// +-----------------+-------------------------+------+------+------+------+------+------+------+------+

	#pragma pack( 1 )
	union{
		uint8_t Buf[ 64 ];
		struct{
			/*-------- Byte0 --------*/
			uint8_t Report_ID :8;
			/*-------- Byte1 to Byte50 --------*/
			struct{
				uint8_t Point_ID :6;
				uint8_t Tip :1;
				uint8_t :1;

				uint16_t x;

				uint16_t y;
			}point[ 10 ];
			/*-------- Byte51 to Byte60 --------*/
			uint8_t Reserve[ 10 ];
			/*-------- Byte61 --------*/
			uint8_t Contact_Count :8;
			/*-------- Byte62 --------*/
			uint8_t :8;
			/*-------- Byte63 --------*/
			uint8_t Check_Sum :8;
		}reg;
	}Data;
	#pragma pack( )

	if( HAL_I2C_Mem_Read( &hi2c1, 0x82, 0x10, I2C_MEMADD_SIZE_8BIT, &Data.Buf[0], 64, 10 ) != HAL_OK )
		return false;

	if( Data.reg.Report_ID != 0x48 )
		return false;

	if( Data.reg.point[0].Tip == 0 ) // Tip switch
		return false;

	if( Data.reg.point[0].Point_ID != 0x00 ) // Point ID = 0
		return false;

	x = ( Data.reg.point[0].x * 1024 ) / TP.range.x;
	y = ( Data.reg.point[0].y * 600 ) / TP.range.y;

	return true;
}

bool STM32TouchController::ILI2511_Read ( int32_t& x, int32_t& y )
{
	// +----------+--------------+------+---------------+------+------+------+------+------+------+------+
	// | CMD Code |     Name     | Note |      b7       |  b6  |  b5  |  b4  |  b3  |  b2  |  b1  |  b0  |
	// +----------+--------------+------+---------------+------+------+------+------+------+------+------+
	// |          | Touch        |      | 0: No touch	                                                 |
	// |          | Information  |      | 1: Last Report as ID0 to ID5(include release ststus)           |
	// |          |              |      | 1: Last Report as ID0 to ID5(include release ststus)           |
	// |          |              +------+---------------+------+-----------------------------------------+
	// |          |              | ID0  | 0: Touch Off  |  0   | X_High direction coordinate             |
	// |          |              |      | 1: Touch Down |      |                                         |
	// |          |              |      +---------------+------+-----------------------------------------+
	// |          |              |      | X_Low direction coordinate                                     |
	// |          |              |      +---------------+------+-----------------------------------------+
	// |          |              |      |       0       |  0   | Y_High direction coordinate             |
	// |          |              |      +---------------+------+-----------------------------------------+
	// |          |              |      | Y_Low direction coordinate                                     |
	// |          |              +------+----------------------------------------------------------------+
	// |          |              |   .  |                                .                               |
	// |          |              |   .  |                                .                               |
	// |          |              |   .  |                                .                               |
	// |          |              |   .  |                                .                               |
	// |          |              +------+---------------+------+-----------------------------------------+
	// |          |              | ID5  | 0: Touch Off  |  0   | X_High direction coordinate             |
	// |          |              |      | 1: Touch Down |      |                                         |
	// |          |              |      +---------------+------+-----------------------------------------+
	// |          |              |      | X_Low direction coordinate                                     |
	// |          |              |      +---------------+------+-----------------------------------------+
	// |          |              |      |       0       |  0   | Y_High direction coordinate             |
	// |          |              |      +---------------+------+-----------------------------------------+
	// |          |              |      | Y_Low direction coordinate                                     |
	// +----------+--------------+------+----------------------------------------------------------------+

	#pragma pack( 1 )
	union{
		uint8_t Buf[ 31 ];
		struct{
			/*-------- Byte0 --------*/
			uint8_t Packet_Number :8;
			/*-------- Byte1 to Byte50 --------*/
			struct{
				uint8_t X_H:6;
				uint8_t :1;
				uint8_t Touch_Status :1;

				uint8_t X_L :8;

				uint8_t Y_H:6;
				uint8_t :2;

				uint8_t Y_L :8;

				uint8_t Touch_Pressure :8;
			}point[ 6 ];
		}reg;
	}Data;
	#pragma pack( )

	uint32_t sx,sy;

	switch( TP.version )
	{
		case 2:
			break;
		case 3:
			if( HAL_I2C_Mem_Read( &hi2c1, 0x82, 0x10, I2C_MEMADD_SIZE_8BIT, &Data.Buf[0], 31, 10 ) != HAL_OK )
				return false;

			if( Data.reg.point[0].Touch_Status != 1 )
				return false;

			sx = ( (Data.reg.point[0].X_H & 0x3F) << 8 ) | Data.reg.point[0].X_L;
			sy = ( (Data.reg.point[0].Y_H & 0x3F) << 8 ) | Data.reg.point[0].Y_L;

			x = ( sx * 1024 ) / TP.range.x;
			y = ( sy * 600 ) / TP.range.y;

			return true;
		default:
			break;
	}
	return false;
}

/* USER CODE END STM32TouchController */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
