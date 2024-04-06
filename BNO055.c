/*
 * BNO055.c
 *
 *  Created on: Mar 6, 2024
 *      Author: Berat Bayram
 */
#include "BNO055_STM32.h"

bno055_sensors_t SensorData;

calib_stat_t status;
float Accel_x, Gyro_x, Magneto_x, Heading, LinAcc_x, Gravity_x;
float Accel_y, Gyro_y, Magneto_y, Roll,    LinAcc_y, Gravity_y;
float Accel_z, Gyro_z, Magneto_z, Pitch,   LinAcc_z, Gravity_z;

void SelectPage(uint8_t page){

	if(HAL_I2C_Mem_Write(&hi2c1, P_BNO055, P_PAGE_ID, 1, &page, 1, HAL_MAX_DELAY) != HAL_OK){
		printf("Page ayarlanamadi!\n");
	}
	else{
		printf("Page ayarlandi\n");
	}
	HAL_Delay(100);
}

void AccelData(void){

	uint8_t ACCELDATA[6];
	HAL_I2C_Mem_Read(&hi2c1, P_BNO055, P_ACCEL_DATA_X_LSB, 1, ACCELDATA, 6, HAL_MAX_DELAY);

	SensorData.accel.x = (int16_t)((ACCELDATA[1] << 8) | ACCELDATA[0]);
	SensorData.accel.y = (int16_t)((ACCELDATA[3] << 8) | ACCELDATA[2]);
	SensorData.accel.z = (int16_t)((ACCELDATA[5] << 8) | ACCELDATA[4]);

	Accel_x = (float)(SensorData.accel.x /100.0);
	Accel_y = (float)(SensorData.accel.y /100.0);
	Accel_z = (float)(SensorData.accel.z /100.0);


}

void MagnetoData(void){

	uint8_t MAGDATA[6];
	HAL_I2C_Mem_Read(&hi2c1, P_BNO055, P_MAG_DATA_X_LSB, 1, MAGDATA, 6, HAL_MAX_DELAY);

	SensorData.magneto.x = (int16_t)((MAGDATA[1] << 8) | MAGDATA[0]);
	SensorData.magneto.y = (int16_t)((MAGDATA[3] << 8) | MAGDATA[2]);
	SensorData.magneto.z = (int16_t)((MAGDATA[5] << 8) | MAGDATA[4]);

	Magneto_x = (float)(SensorData.magneto.x /16.0);
	Magneto_y = (float)(SensorData.magneto.y /16.0);
	Magneto_z = (float)(SensorData.magneto.z /16.0);

}

void GyroData(void){

	uint8_t GYRODATA[6];
	HAL_I2C_Mem_Read(&hi2c1, P_BNO055, P_GYRO_DATA_X_LSB, 1, GYRODATA, 6, HAL_MAX_DELAY);

	SensorData.gyro.x = (int16_t)((GYRODATA[1] << 8) | GYRODATA[0]);
	SensorData.gyro.y =(int16_t)((GYRODATA[3] << 8) | GYRODATA[2]);
	SensorData.gyro.z = (int16_t)((GYRODATA[5] << 8) | GYRODATA[4]);

	Gyro_x = (float)((SensorData.gyro.x) /16.0);
	Gyro_y = (float)((SensorData.gyro.y) /16.0);
	Gyro_z = (float)((SensorData.gyro.z) /16.0);

}

void Euler_Angle(void){

	uint8_t EULERDATA[6];
	HAL_I2C_Mem_Read(&hi2c1, P_BNO055, P_EUL_HEADING_LSB, 1, EULERDATA, 6, HAL_MAX_DELAY);

	SensorData.euler.x = (int16_t)(EULERDATA[0] | (EULERDATA[1] << 8));
	SensorData.euler.y = (int16_t)(EULERDATA[2] | (EULERDATA[3] << 8));
	SensorData.euler.z = (int16_t)(EULERDATA[4] | (EULERDATA[5] << 8));

	Heading = (float)(SensorData.euler.x /16.0);
	Roll    = (float)(SensorData.euler.y /16.0);
	Pitch   = (float)(SensorData.euler.z /16.0);
	printf("Euler Angles\n");
	printf("Heading: %f\t",Heading);
	printf("Roll: %f\t",Roll);
	printf("Pitch: %f\t\n",Pitch);

}

void Lineer_Accel(void){

	uint8_t LINEERDATA[6];
	HAL_I2C_Mem_Read(&hi2c1, P_BNO055, P_LIA_DATA_X_LSB, 1, LINEERDATA, 6, HAL_MAX_DELAY);

	SensorData.lineeracc.x = (int16_t)((LINEERDATA[1] << 8) | LINEERDATA[0]);
	SensorData.lineeracc.y = (int16_t)((LINEERDATA[3] << 8) | LINEERDATA[2]);
	SensorData.lineeracc.z = (int16_t)((LINEERDATA[5] << 8) | LINEERDATA[4]);

	LinAcc_x = (float)(SensorData.lineeracc.x /100.0);
	LinAcc_y = (float)(SensorData.lineeracc.y /100.0);
	LinAcc_z = (float)(SensorData.lineeracc.z /100.0);

}

void Gravity_Vector(void){

	uint8_t GRAVITYDATA[6];
	HAL_I2C_Mem_Read(&hi2c1, P_BNO055, P_GRV_DATA_X_LSB, 1, GRAVITYDATA, 6, HAL_MAX_DELAY);

	SensorData.gravity.x = (int16_t)((GRAVITYDATA[1] << 8) | GRAVITYDATA[0]);
	SensorData.gravity.y = (int16_t)((GRAVITYDATA[3] << 8) | GRAVITYDATA[2]);
	SensorData.gravity.z = (int16_t)((GRAVITYDATA[5] << 8) | GRAVITYDATA[4]);

	Gravity_x = (float)(SensorData.gravity.x /100.0);
	Gravity_y = (float)(SensorData.gravity.y /100.0);
	Gravity_z = (float)(SensorData.gravity.z /100.0);
}

void Status_Calibrated(void){

	SelectPage(Page_0);
	int system,gyro,acc,mag;
	uint8_t result;
	HAL_I2C_Mem_Read(&hi2c1, P_BNO055, P_CALIB_STAT, 1, &result, 6, HAL_MAX_DELAY);

	status.system_cal = (uint8_t)(result >> 6);
	status.gyro_cal   = (uint8_t)((result >> 4)  & 0x3);
	status.acc_cal    = (uint8_t)((result >> 2) & 0x3);
	status.mag_cal    = (uint8_t)(result & 0x3);
	system=status.system_cal;
	gyro=status.gyro_cal;
	acc=	status.acc_cal;
	mag=status.mag_cal;
	printf("Kalibrasyon Durumu\n");
	printf("System: %d\t",system);
	printf("Gyro: %d\t",gyro);
	printf("Accel: %d\t",acc);
	printf("Mag: %d\t\n",mag);

}

void Unit_Select(uint8_t ORI,uint8_t TEMP,uint8_t EUL,uint8_t GYRO, uint8_t ACC){

	uint8_t unit = (ORI | TEMP | EUL | GYRO | ACC);
	if(HAL_I2C_Mem_Write(&hi2c1, P_BNO055, P_UNIT_SEL, 1, &unit, sizeof(unit), HAL_MAX_DELAY) != HAL_OK){
		printf("Unit Ayarlanamadi!\n");
	}
	else{
		printf("Unit Ayarlandi!\n");
	}
}

void Set_Operation_Mode(op_modes_t Mode){

	if(	HAL_I2C_Mem_Write(&hi2c1, P_BNO055, P_OPR_MODE, 1, &Mode, 1, HAL_MAX_DELAY) !=HAL_OK){
		printf("Operation Mode Ayarlanamadi\n");
	}
	else printf("Operation Mode Ayarlandi\n");

	if(Mode == CONFIG_MODE) HAL_Delay(19);

	else HAL_Delay(9);

}

bool check_id(void){
	uint8_t id;
	HAL_I2C_Mem_Read(&hi2c1, P_BNO055, P_CHIP_ID, 1, &id, sizeof(id), HAL_MAX_DELAY);

	if(id == BNO055_ID){
		return true;
	}
	else{
		return false;
	}
}


void SetPowerTo_NORMAL(void){

	uint8_t power_normal = 0x00; // Low power mode 0x1 and Suspend mode 0x2
	if(	HAL_I2C_Mem_Write(&hi2c1, P_BNO055, P_PWR_MODE, 1, &power_normal, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		printf("Power Mode Ayarlanamadi!\n");
	}
	else
	{
		printf("Power Mode Ayarlandi\n");
	}
	HAL_Delay(50);

}

void External_Crystal(bool use) {

	if(use){
		uint8_t data = 0x80; //First bit: External Crystal=1; Internal Crystal=0
		HAL_I2C_Mem_Write(&hi2c1, P_BNO055, P_SYS_TRIGGER, 1, &data, sizeof(data), HAL_MAX_DELAY);
	}
	else{
		uint8_t data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, P_BNO055, P_SYS_TRIGGER, 1, &data, sizeof(data), HAL_MAX_DELAY);
	}
}

void BNO055_Remap_Axis(axis_position_t position){

	switch (position) {
			case P0:
				BNO055_Axis(0x21,0x04);
				break;
			case P1:
				BNO055_Axis(0x21,0x04);
				break;
			case P2:
				BNO055_Axis(0x21,0x04);
				break;
			case P3:
				BNO055_Axis(0x21,0x04);
				break;
			case P4:
				BNO055_Axis(0x21,0x04);
				break;
			case P5:
				BNO055_Axis(0x21,0x04);
				break;
			case P6:
				BNO055_Axis(0x21,0x04);
				break;
			case P7:
				BNO055_Axis(0x21,0x04);
				break;
		}
}

void BNO055_Axis(uint8_t config, uint8_t sign){

	HAL_I2C_Mem_Write(&hi2c1,P_BNO055, P_AXIS_MAP_CONFIG, 1, &config, 1, HAL_MAX_DELAY);
	HAL_Delay(20);
	HAL_I2C_Mem_Write(&hi2c1, P_BNO055, P_AXIS_MAP_SIGN, 1, &sign, 1, HAL_MAX_DELAY);
	HAL_Delay(100);
}

void SET_Accel_Range(accel_range_t range){
	SelectPage(1);
	HAL_I2C_Mem_Write(&hi2c1, P_BNO055, P_ACC_CONFIG, 1, &range, 1, HAL_MAX_DELAY);

}

void BNO055_Init(void){

	SelectPage(Page_0);
	Set_Operation_Mode(CONFIG_MODE);

	uint8_t reset = 0x20;
	HAL_I2C_Mem_Write(&hi2c1, P_BNO055, P_SYS_TRIGGER, 1, &reset, sizeof(reset), HAL_MAX_DELAY);

	HAL_Delay(100);

	SetPowerTo_NORMAL();
	HAL_Delay(100);

	External_Crystal(true);
	HAL_Delay(100);

	Unit_Select(WINDOWS, TEMP_CELCIUS, EUL_DEG, GYRO_DPS, ACC_MS2);
	HAL_Delay(100);
}

