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

extern I2C_HandleTypeDef hi2c1;


#define P_BNO055 (0x28<<1)
#define BNO055_ID 0xA0
//-------------------------------------[Register Page 0]-------------------------------------

//========================================================================= DataSheet PAGE=50
#define P_MAG_RADIUS_MSB 0x6A
#define P_MAG_RADIUS_LSB 0x69
#define P_ACC_RADIUS_MSB 0x68
#define P_ACC_RADIUS_LSB 0x67

#define P_GYRO_OFFSET_Z_MSB 0x66
#define P_GYRO_OFFSET_Z_LSB 0x65
#define P_GYRO_OFFSET_Y_MSB 0x64
#define P_GYRO_OFFSET_Y_LSB 0x63
#define P_GYRO_OFFSET_X_MSB 0x62
#define P_GYRO_OFFSET_X_LSB 0x61

#define P_MAG_OFFSET_Z_MSB 0x60
#define P_MAG_OFFSET_Z_LSB 0x5F
#define P_MAG_OFFSET_Y_MSB 0x5E
#define P_MAG_OFFSET_Y_LSB 0x5D
#define P_MAG_OFFSET_X_MSB 0x5C
#define P_MAG_OFFSET_X_LSB 0x5B

#define P_ACC_OFFSET_Z_MSB 0x5A
#define P_ACC_OFFSET_Z_LSB 0x59
#define P_ACC_OFFSET_Y_MSB 0x58
#define P_ACC_OFFSET_Y_LSB 0x57
#define P_ACC_OFFSET_X_MSB 0x56
#define P_ACC_OFFSET_X_LSB 0x55
//========================================================================= DataSheet PAGE=51
#define P_AXIS_MAP_SIGN 0x42   //To configure sign of axis: Positive=0 and Negative=1
#define P_AXIS_MAP_CONFIG 0x41 //
#define P_SYS_TRIGGER 0x3F // External Crystal, Reset Status Bit and System, Self Test
#define P_PWR_MODE 0x3E
#define P_OPR_MODE 0x3D  //Can change data output type Accelerometer, Gyroscope, Magnetometer, Euler

//-------------------------------------------------------------------------- UNIT SEL
#define P_UNIT_SEL 0x3B  //Can change data output units Accelerometer, Gyroscope, Magnetometer, Euler

#define ANDROID (1<<0x07)
#define WINDOWS (0<<0x07)
#define TEMP_CELCIUS (0<<0x04)
#define TEMP_FAHRENHEIT (1<<0x04)
#define EUL_DEG (0<<0x02)
#define EUL_RAD (1<<0x02)
#define GYRO_DPS (0<<0x01)
#define GYRO_RPS (1<<0x01)
#define ACC_MS2 0
#define ACC_MG 1
//-------------------------------------------------------------------------- UNIT SEL

#define P_CALIB_STAT 0x35

#define P_GRV_DATA_Z_MSB 0x33
#define P_GRV_DATA_Z_LSB 0x32
#define P_GRV_DATA_Y_MSB 0x31
#define P_GRV_DATA_Y_LSB 0x30
#define P_GRV_DATA_X_MSB 0x2F
#define P_GRV_DATA_X_LSB 0x2E

#define P_LIA_DATA_Z_MSB 0x2D
#define P_LIA_DATA_Z_LSB 0x2C
#define P_LIA_DATA_Y_MSB 0x2B
#define P_LIA_DATA_Y_LSB 0x2A
#define P_LIA_DATA_X_MSB 0x29
#define P_LIA_DATA_X_LSB 0x28

#define P_QUA_DATA_Z_MSB 0x27
#define P_QUA_DATA_Z_LSB 0x26
#define P_QUA_DATA_Y_MSB 0x25
#define P_QUA_DATA_Y_LSB 0x24
#define P_QUA_DATA_X_MSB 0x23
#define P_QUA_DATA_X_LSB 0x22
#define P_QUA_DATA_W_MSB 0x21
#define P_QUA_DATA_W_LSB 0x20
//========================================================================= DataSheet PAGE=52
#define P_EUL_PITCH_MSB 0x1F
#define P_EUL_PITCH_LSB 0x1E
#define P_EUL_ROLL_MSB 0x1D
#define P_EUL_ROLL_LSB 0x1C
#define P_EUL_HEADING_MSB 0x1B
#define P_EUL_HEADING_LSB 0x1A

#define P_GYRO_DATA_Z_MSB 0x19
#define P_GYRO_DATA_Z_LSB 0x18
#define P_GYRO_DATA_Y_MSB 0x17
#define P_GYRO_DATA_Y_LSB 0x16
#define P_GYRO_DATA_X_MSB 0x15
#define P_GYRO_DATA_X_LSB 0x14

#define P_MAG_DATA_Z_MSB 0x13
#define P_MAG_DATA_Z_LSB 0x12
#define P_MAG_DATA_Y_MSB 0x11
#define P_MAG_DATA_Y_LSB 0x10
#define P_MAG_DATA_X_MSB 0x0F
#define P_MAG_DATA_X_LSB 0x0E

#define P_ACCEL_DATA_Z_MSB 0x0D
#define P_ACCEL_DATA_Z_LSB 0x0C
#define P_ACCEL_DATA_Y_MSB 0x0B
#define P_ACCEL_DATA_Y_LSB 0x0A
#define P_ACCEL_DATA_X_MSB 0x09
#define P_ACCEL_DATA_X_LSB 0x08
#define P_PAGE_ID 0x07
#define P_CHIP_ID 0x00

#define Page_0 0x00
#define Page_1 0x01

//-------------------------------------[Register Page 0]-------------------------------------

//-------------------------------------[Register Page 1]-------------------------------------

#define P_GYR_AM_SET 0x1F
#define P_GYR_AM_THRES 0x1E
#define P_GYR_DUR_Z 0x1D
#define P_GYR_HR_Z_SET 0x1C
#define P_GYR_DUR_Y 0x1B
#define P_GYR_HR_Y_SET 0x1A
#define P_GYR_DUR_X 0x19
#define P_GYR_HR_X_SET 0x18
#define P_GYR_INT_SETING 0x17
#define P_ACC_NM_SET 0x16
#define P_ACC_NM_THRES 0x15
#define P_ACC_HG_THRES 0x14
#define P_ACC_HG_DURATION 0x13
#define P_ACC_INT_Setting 0x12
#define P_ACC_AM_THRES 0x11
#define P_INT_EN 0x10
#define P_INT_MSK 0x0F
#define P_GYR_Sleep_Config 0x0D
#define P_ACC_Sleep_Config 0x0C
#define P_GYRO_CONFIG_1 0x0B
#define P_GYRO_CONFIG_0 0x0A
#define P_MAG_CONFIG 0x09
#define P_ACC_CONFIG 0x08

//-------------------------------------[Register Page 1]-------------------------------------


typedef enum{// OPERATION MODES
	CONFIG_MODE =0x00,
	ACC_ONLY =0x01,
	MAG_ONLY =0x02,
	GYRO_ONLY =0x03,
	ACC_MAG =0x04,
	ACC_GYRO =0x05,
	MAG_GYRO =0x06,
	AccMagGyro= 0x07,
	IMU= 0x08,
	COMPASS =0x09,
	M4G= 0x0A,
	NDOF_FMC_OFF =0x0,
	NDOF= 0x0C
}op_modes_t;

typedef struct{ //CALIBRATED STATUS
	uint8_t system_cal;
	uint8_t gyro_cal;
	uint8_t acc_cal;
	uint8_t mag_cal;
}calib_stat_t;

typedef enum{  //BNO055 AXIS REMAP
	P0=0,
	P1,
	P2,
	P3,
	P4,
	P5,
	P6,
	P7
}axis_position_t;

typedef struct{ //SENSOR DATAS X,Y and Z
	int16_t x;
	int16_t y;
	int16_t z;
}bno055_data_xyz_t;

typedef struct{ //SENSOR DATAS
	bno055_data_xyz_t accel;
	bno055_data_xyz_t gyro;
	bno055_data_xyz_t magneto;
	bno055_data_xyz_t euler;
	bno055_data_xyz_t lineeracc;
	bno055_data_xyz_t gravity;
}bno055_sensors_t;

typedef enum{ //ACCEL RANGES
	Range_2G=0x00,
	Range_4G=0x01,
	Range_8G=0x02,
	Range_16G=0x03
}accel_range_t;

void AccelData(void);
void MagnetoData(void);
void GyroData(void);
void Euler_Angle(void);
void Lineer_Accel(void);
void Unit_Select(uint8_t ORI,uint8_t TEMP,uint8_t EUL,uint8_t GYRO, uint8_t ACC);
void Status_Calibrated(void);
void Gravity_Vector(void);
void Set_Operation_Mode(op_modes_t Mode);
void External_Crystal(bool use);
void SetPowerTo_NORMAL(void);
void BNO055_Remap_Axis(axis_position_t position);
void BNO055_Axis(uint8_t config, uint8_t sign);
void SelectPage(uint8_t page);
void SET_Accel_Range(accel_range_t range);
bool check_id(void);
void BNO055_Init(void);

#endif /* INC_BNO055_STM32_H_ */
