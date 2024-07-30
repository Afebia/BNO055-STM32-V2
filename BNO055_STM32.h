/*
 * BNO055_STM32.h
 *
 *  Created on: Apr 3, 2024
 *      Author: berat
 */

#ifndef INC_BNO055_STM32_H_
#define INC_BNO055_STM32_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c2;
#define bno_i2c (hi2c2)


#define P_BNO055 				    (0x28<<1)			/*!I2C Address of BNO055, which is 0x50*/
#define BNO055_ID 					0xA0				/*!Chip ID of BNO055*/

/*===========================================================================================================================*/
/*									*[Page 0 Register Addresses Macro Definition]*											 */
/*===========================================================================================================================*/

/*
 *  	Radius registers
 */
#define MAG_RADIUS_MSB_ADDR 		0x6A
#define MAG_RADIUS_LSB_ADDR 		0x69
#define ACC_RADIUS_MSB_ADDR 		0x68
#define ACC_RADIUS_LSB_ADDR 		0x67

/*
 *	Gyroscope offset register address macros
 */
#define GYRO_OFFSET_Z_MSB_ADDR 		0x66
#define GYRO_OFFSET_Z_LSB_ADDR 		0x65
#define GYRO_OFFSET_Y_MSB_ADDR 		0x64
#define GYRO_OFFSET_Y_LSB_ADDR 		0x63
#define GYRO_OFFSET_X_MSB_ADDR 		0x62
#define GYRO_OFFSET_X_LSB_ADDR 		0x61

/*
 *	Magnetometer offset register address macros
 */
#define MAG_OFFSET_Z_MSB_ADDR 		0x60
#define MAG_OFFSET_Z_LSB_ADDR 		0x5F
#define MAG_OFFSET_Y_MSB_ADDR 		0x5E
#define MAG_OFFSET_Y_LSB_ADDR 		0x5D
#define MAG_OFFSET_X_MSB_ADDR 		0x5C
#define MAG_OFFSET_X_LSB_ADDR 		0x5B

/*
 *	Accelerometer offset register address macros
 */
#define ACC_OFFSET_Z_MSB_ADDR 		0x5A
#define ACC_OFFSET_Z_LSB_ADDR 		0x59
#define ACC_OFFSET_Y_MSB_ADDR 		0x58
#define ACC_OFFSET_Y_LSB_ADDR 		0x57
#define ACC_OFFSET_X_MSB_ADDR 		0x56
#define ACC_OFFSET_X_LSB_ADDR 		0x55

#define AXIS_MAP_SIGN_ADDR 			0x42   /*!To configure sign of axis: Positive=0 and Negative=1*/
#define AXIS_MAP_CONFIG_ADDR 		0x41   /*!Configure Axis of device to the new reference axis*/
#define TEMP_SOURCE_ADDR			0x40   /*!Configure ...*/
#define SYS_TRIGGER_ADDR 			0x3F   /*!External Crystal, Reset Status Bit and System, Self Test*/
#define PWR_MODE_ADDR 				0x3E   /*!To select power mode of BNO055*/
#define OPR_MODE_ADDR 				0x3D   /*!Operation modes(Non-fusion, fusion and config) can be selected by this register*/

#define UNIT_SEL_ADDR 				0x3B   /*!Data output format and the measurement unit of Accelerometer,
											Gyroscope, Euler etc. can be configured by this register*/

#define SYS_ERR_ADDR				0x3A   /*!Read-only, contain the error status if SYS_STATUS register is SYSTEM_ERROR */
#define SYS_STATUS_ADDR				0x39   /*!Read-only register which contains system status code for detail info refer to 4.3.58*/
#define SYS_CLK_STATUS_ADDR			0x38   /*!0 indicates it is free to configure the clock source(External or Internal)*/
#define INT_STA_ADDR				0x37
#define ST_RESULT_ADDR				0x36   /*!Read-only register which contains self test results of MCU, gyro, mag, accel.....*/
#define CALIB_STAT_ADDR 			0x35   /*!Read-only register which contains calibration status of system, gyro, accel, mag*/
#define TEMP_ADDR					0x34   /*!Read-only register which contains temperature data. Output source can be selected by TEMP_SOURCE register*/

/*
 * Address macros of gravity vector data output registers
 * Note: Read-only register access
 */
#define GRV_DATA_Z_MSB_ADDR 		0x33
#define GRV_DATA_Z_LSB_ADDR 		0x32
#define GRV_DATA_Y_MSB_ADDR 		0x31
#define GRV_DATA_Y_LSB_ADDR 		0x30
#define GRV_DATA_X_MSB_ADDR 		0x2F
#define GRV_DATA_X_LSB_ADDR 		0x2E
#define GRV_DATA_BASEADDR			GRV_DATA_X_LSB_ADDR

/*
 * Address macros of linear acceleration data output registers
 * Note: Read-only register access
 */
#define LIA_DATA_Z_MSB_ADDR 		0x2D
#define LIA_DATA_Z_LSB_ADDR 		0x2C
#define LIA_DATA_Y_MSB_ADDR 		0x2B
#define LIA_DATA_Y_LSB_ADDR 		0x2A
#define LIA_DATA_X_MSB_ADDR 		0x29
#define LIA_DATA_X_LSB_ADDR 		0x28
#define LIA_DATA_BASEADDR			LIA_DATA_X_LSB_ADDR

/*
 * Address macros of quaternion data output registers
 * Note: Read-only register access
 */
#define QUA_DATA_Z_MSB_ADDR 		0x27
#define QUA_DATA_Z_LSB_ADDR 		0x26
#define QUA_DATA_Y_MSB_ADDR 		0x25
#define QUA_DATA_Y_LSB_ADDR 		0x24
#define QUA_DATA_X_MSB_ADDR 		0x23
#define QUA_DATA_X_LSB_ADDR 		0x22
#define QUA_DATA_W_MSB_ADDR 		0x21
#define QUA_DATA_W_LSB_ADDR 		0x20
#define QUA_DATA_BASEADDR			QUA_DATA_W_LSB_ADDR

/*
 * Address macros of euler data output registers
 * Note: Read-only register access
 */
#define EUL_PITCH_MSB_ADDR 			0x1F
#define EUL_PITCH_LSB_ADDR 			0x1E
#define EUL_ROLL_MSB_ADDR 			0x1D
#define EUL_ROLL_LSB_ADDR 			0x1C
#define EUL_HEADING_MSB_ADDR 		0x1B
#define EUL_HEADING_LSB_ADDR 		0x1A
#define EUL_DATA_BASEADDR			EUL_HEADING_LSB_ADDR

/*
 * Address macros of gyroscope data output registers
 * Note: Read-only register access
 */
#define GYRO_DATA_Z_MSB_ADDR 		0x19
#define GYRO_DATA_Z_LSB_ADDR 		0x18
#define GYRO_DATA_Y_MSB_ADDR 		0x17
#define GYRO_DATA_Y_LSB_ADDR 		0x16
#define GYRO_DATA_X_MSB_ADDR 		0x15
#define GYRO_DATA_X_LSB_ADDR 		0x14
#define GYRO_DATA_BASEADDR			GYRO_DATA_X_LSB_ADDR

/*
 * Address macros of magnetometer data output registers
 * Note: Read-only register access
 */
#define MAG_DATA_Z_MSB_ADDR 		0x13
#define MAG_DATA_Z_LSB_ADDR 		0x12
#define MAG_DATA_Y_MSB_ADDR 		0x11
#define MAG_DATA_Y_LSB_ADDR 		0x10
#define MAG_DATA_X_MSB_ADDR 		0x0F
#define MAG_DATA_X_LSB_ADDR 		0x0E
#define MAG_DATA_BASEADDR			MAG_DATA_X_LSB_ADDR

/*
 * Address macros of accelerometer data output registers
 * Note: Read-only register access
 */
#define ACCEL_DATA_Z_MSB_ADDR 		0x0D
#define ACCEL_DATA_Z_LSB_ADDR 		0x0C
#define ACCEL_DATA_Y_MSB_ADDR 		0x0B
#define ACCEL_DATA_Y_LSB_ADDR 		0x0A
#define ACCEL_DATA_X_MSB_ADDR 		0x09
#define ACCEL_DATA_X_LSB_ADDR 		0x08
#define ACCEL_DATA_BASEADDR		ACCEL_DATA_X_LSB_ADDR

/*
 * Address macros of register which contains various ID parameters
 * Note: Read-only register access
 */
#define PAGE_ID_ADDR 				0x07		/*!Read: Number of currently selected register page, Write: Change Page*/
#define BL_REV_ID_ADDR				0x06		/*!Contains version of the bootloader in the MCU */
#define SW_REV_ID_MSB_ADDR			0x05		/*!Contains upper byte of SW revision ID, depending on SW revision programmed on MCU */
#define SW_REV_ID_LSB_ADDR			0x04		/*!Contains lower byte of SW revision ID, depending on SW revision programmed on MCU */
#define GYR_ID_ADDR					0x03		/*!Contains Chip id of Gyroscope id, which is 0x0F */
#define MAG_ID_ADDR					0x02		/*!Contains Chip id of Magnetometer id, which is 0x32 */
#define ACC_ID_ADDR					0x01		/*!Contains Chip id of Accelerometer id, which is 0xFB */
#define CHIP_ID_ADDR 				0x00		/*!Contains Chip id of device, which is 0xA0 */

/*===========================================================================================================================*/
/*									*[ Page 1 Register Addresses Macro Definition]*											 */
/*===========================================================================================================================*/

/*
 * NOTE: All page 1 registers have read-only access
 */
#define UNIQUE_ID_BASEADDR			0x50	/*!Starts from 0x5F to 0x50, contains BNO055's unique ID */
#define GYR_AM_SET_ADDR 			0x1F	/*! */
#define GYR_AM_THRES_ADDR 			0x1E	/*! */
#define GYR_DUR_Z_ADDR 				0x1D	/*! */
#define GYR_HR_Z_SET_ADDR 			0x1C	/*! */
#define GYR_DUR_Y_ADDR 				0x1B	/*! */
#define GYR_HR_Y_SET_ADDR 			0x1A	/*! */
#define GYR_DUR_X_ADDR 				0x19	/*! */
#define GYR_HR_X_SET_ADDR 			0x18	/*! */
#define GYR_INT_SETING_ADDR 		0x17	/*! */
#define ACC_NM_SET_ADDR 			0x16	/*! */
#define ACC_NM_THRES_ADDR 			0x15	/*! */
#define ACC_HG_THRES_ADDR 			0x14	/*! */
#define ACC_HG_DURATION_ADDR 		0x13	/*! */
#define ACC_INT_Setting_ADDR 		0x12	/*! */
#define ACC_AM_THRES_ADDR 			0x11	/*! */
#define INT_EN_ADDR 				0x10	/*! */
#define INT_MSK_ADDR 				0x0F	/*! */
#define GYR_Sleep_Config_ADDR 		0x0D	/*! */
#define ACC_Sleep_Config_ADDR 		0x0C	/*! */
#define GYRO_CONFIG_1_ADDR 			0x0B	/*! */
#define GYRO_CONFIG_0_ADDR 			0x0A	/*! */
#define MAG_CONFIG_ADDR 			0x09	/*! */
#define ACC_CONFIG_ADDR 			0x08	/*! */

/*
 * Page macros
 */
#define PAGE_0 						0x00
#define PAGE_1 						0x01

/** ----------------------------------------------------------------------------------------------------
  * 			 					  BNO055 status macros definition
  * ----------------------------------------------------------------------------------------------------
  */
typedef struct{
	uint8_t STresult;	//First 4 bit[0:3] indicates self test resulst 3:ST_MCU, 2:ST_GYRO, 1:ST_MAG, 0:ST_ACCEL
	uint8_t SYSError;	//Contains system error type If SYSStatus is System_Error(0x01)
	uint8_t SYSStatus;
}BNO_Status_t;

typedef struct{
	uint8_t System;
	uint8_t Gyro;
	uint8_t Acc;
	uint8_t MAG;
}Calib_status_t;

/** ----------------------------------------------------------------------------------------------------
  * 			 BNO055 Data output structures and  macros definition for ReadData function
  * ----------------------------------------------------------------------------------------------------
  */
typedef struct{ //SENSOR DATA AXIS X, Y and Z
	float X;
	float Y;
	float Z;
}BNO055_Data_XYZ_t;

typedef struct{ //SENSOR DATA AXIS W, X, Y and Z (Only for quaternion data)
	float W;
	float X;
	float Y;
	float Z;
}BNO055_QuaData_WXYZ_t;

typedef struct{ //SENSOR DATAS
	BNO055_Data_XYZ_t Accel;
	BNO055_Data_XYZ_t Gyro;
	BNO055_Data_XYZ_t Magneto;
	BNO055_Data_XYZ_t Euler;
	BNO055_Data_XYZ_t LineerAcc;
	BNO055_Data_XYZ_t Gravity;
	BNO055_QuaData_WXYZ_t Quaternion;
}BNO055_Sensors_t;

typedef enum {
    SENSOR_GRAVITY     = 0x01,
    SENSOR_QUATERNION  = 0x02,
    SENSOR_LINACC      = 0x04,
    SENSOR_GYRO        = 0x08,
    SENSOR_ACCEL       = 0x10,
    SENSOR_MAG         = 0x20,
    SENSOR_EULER       = 0x40,
} BNO055_Sensor_Type;

//Base Addresses of output data register
#define BNO_ACCEL 					ACCEL_DATA_BASEADDR
#define BNO_GYRO 					GYRO_DATA_BASEADDR
#define BNO_MAG 					MAG_DATA_BASEADDR
#define BNO_EULER 					EUL_DATA_BASEADDR
#define BNO_LINACC 					LIA_DATA_BASEADDR
#define BNO_GRAVITY					GRV_DATA_BASEADDR
#define BNO_QUATERNION				QUA_DATA_BASEADDR

/** ----------------------------------------------------------------------------------------------------
  * 			 			   BNO055 Init structure and macros definition
  * ----------------------------------------------------------------------------------------------------
  */
typedef struct
{
	uint8_t Unit_Sel;
	uint8_t Axis;
	uint8_t Axis_sign;
	uint8_t Mode;
	uint8_t OP_Modes;
	uint8_t Clock_Source;
	uint8_t ACC_Range;
}BNO055_Init_t;

typedef enum{// OPERATION MODES
	CONFIG_MODE =0x00,		//This is the only mode in which all the writable register map entries can be changed (Except INT , INT_MASK, OPR_MODE)
	ACC_ONLY =0x01,
	MAG_ONLY =0x02,
	GYRO_ONLY =0x03,
	ACC_MAG =0x04,
	ACC_GYRO =0x05,
	MAG_GYRO =0x06,
	AMG= 0x07,
	IMU= 0x08,
	COMPASS =0x09,
	M4G= 0x0A,
	NDOF_FMC_OFF =0x0B,
	NDOF= 0x0C
}Op_Modes_t;

#define BNO055_NORMAL_MODE			0
#define BNO055_LOWPOWER_MODE		1
#define BNO055_SUSPEND_MODE			2

#define DEFAULT_AXIS_REMAP			0x24
#define DEFAULT_AXIS_SIGN			0x00

#define CLOCK_EXTERNAL				(1 << 7)
#define CLOCK_INTERNAL				(0 << 7)

/*
 * UNIT_SELECT register macros definition
 */
#define UNIT_ORI_ANDROID 			(1 << 7)
#define UNIT_ORI_WINDOWS 			(0 << 7)
#define UNIT_TEMP_CELCIUS 			(0 << 4)
#define UNIT_TEMP_FAHRENHEIT 		(1 << 4)
#define UNIT_EUL_DEG 				(0 << 2)
#define UNIT_EUL_RAD 				(1 << 2)
#define UNIT_GYRO_DPS 				(0 << 1)
#define UNIT_GYRO_RPS 				(1 << 1)
#define UNIT_ACC_MS2				(0 << 0)
#define UNIT_ACC_MG 				(1 << 0)

/*
 * ACCEL range macros definition for ACC_CONFIG register
 */
#define Range_2G 					0x00
#define Range_4G 					0x01
#define Range_8G 					0x02
#define Range_16G 					0x03

/*
 * BNO055 library function declaration
 */
void getCalibration(Calib_status_t *calib);
Op_Modes_t getCurrentMode(void);
bool isFullyCalibrated(void);
void setSensorOffsets(const uint8_t *calibData) ;
void getSensorOffsets(uint8_t *calibData);
bool Calibrate_BNO055(void);

void Set_Operation_Mode(Op_Modes_t Mode);
void Clock_Source(uint8_t source);
void SetPowerMODE(uint8_t BNO055_);
void BNO055_Axis(uint8_t remap, uint8_t sign);
void SelectPage(uint8_t page);
void SET_Accel_Range(uint8_t range);
void BNO055_Init(BNO055_Init_t Init);
void ReadData(BNO055_Sensors_t *sensorData,BNO055_Sensor_Type sensors);
void Check_Status(BNO_Status_t *result);
void ResetBNO055(void);

#endif /* INC_BNO055_STM32_H_ */
