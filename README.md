If you want to measure the speed of a car or the angles of a spacecraft, you can easily use the BNO055 sensor.
Today we created a library for BNO055 to use with STM32. With this library you can configure BNO055 and retrieve data from it.

WARNING: This library is written on an STM32F407-DISC1 so its header file is stm32f4xx_hal.h. İf you use another board, for example STM32F103C8 BluePill, just change that to stm32f1xx_hal.h in our BNO055_STM32.h header file.

```#include "stm32f4xx_hal.h```
````void Lineer_Accel(void){

	uint8_t LINEERDATA[6];
	HAL_I2C_Mem_Read(&hi2c1, P_BNO055, P_LIA_DATA_X_LSB, 1, LINEERDATA, 6, HAL_MAX_DELAY);

	SensorData.lineeracc.x = (int16_t)((LINEERDATA[1] << 8) | LINEERDATA[0]);
	SensorData.lineeracc.y = (int16_t)((LINEERDATA[3] << 8) | LINEERDATA[2]);
	SensorData.lineeracc.z = (int16_t)((LINEERDATA[5] << 8) | LINEERDATA[4]);

	LinAcc_x = (float)(SensorData.lineeracc.x /100.0);
	LinAcc_y = (float)(SensorData.lineeracc.y /100.0);
	LinAcc_z = (float)(SensorData.lineeracc.z /100.0);

}````
