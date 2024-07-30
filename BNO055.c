/*
 * BNO055.c
 *
 *  Created on: Mar 6, 2024
 *      Author: Berat Bayram
 */
#include "BNO055_STM32.h"
#include <string.h>

/*!
 *   @brief  Gets the latest system status info
 *
 *   @param  BNO_status_t structure that contains status information
 *           STresult, SYSError and SYSStatus
 *
 *   @retval None
 */
void Check_Status(BNO_Status_t *result){

	HAL_StatusTypeDef status;
	uint8_t value;

	  /* Self Test Results
	     1 = test passed, 0 = test failed

	     Bit 0 = Accelerometer self test
	     Bit 1 = Magnetometer self test
	     Bit 2 = Gyroscope self test
	     Bit 3 = MCU self test

	     0x0F = all good!
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, ST_RESULT_ADDR, 1, &value, 1, 100);
	if (status != HAL_OK) {
	    printf("I2C Read Error: ST_RESULT_ADDR\n");
	}
	HAL_Delay(50);
	result->STresult = value;
	value=0;

	  /* System Status (see section 4.3.58)
	     0 = Idle
	     1 = System Error
	     2 = Initializing Peripherals
	     3 = System Iniitalization
	     4 = Executing Self-Test
	     5 = Sensor fusio algorithm running
	     6 = System running without fusion algorithms
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, SYS_STATUS_ADDR, 1, &value, 1, 100);
	if (status != HAL_OK) {
	    printf("I2C Read Error: SYS_STATUS_ADDR\n");
	}
	HAL_Delay(50);
	result->SYSStatus = value;
	value=0;
	  /* System Error (see section 4.3.59)
	     0 = No error
	     1 = Peripheral initialization error
	     2 = System initialization error
	     3 = Self test result failed
	     4 = Register map value out of range
	     5 = Register map address out of range
	     6 = Register map write error
	     7 = BNO low power mode not available for selected operation mode
	     8 = Accelerometer power mode not available
	     9 = Fusion algorithm configuration error
	     A = Sensor configuration error
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, SYS_ERR_ADDR, 1, &value, 1, 100);
	if (status != HAL_OK) {
	    printf("I2C Read Error: SYS_ERR_ADDR\n");
	}
	HAL_Delay(50);
	result->SYSError = value;
}

/*!
 *   @brief  Changes register page
 *
 *   @param  Page number
 *   		Possible Arguments
 * 			[PAGE_0
 * 			 PAGE_1]
 *
 * 	 @retval None
 */
void SelectPage(uint8_t page){

	if(HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, PAGE_ID_ADDR, 1, &page, 1, 100) != HAL_OK){
		printf("Register page replacement could not be set\n");
	}
	HAL_Delay(50);
}

/**
  * @brief  Software Reset to BNO055
  *
  * @param  None
  *
  * @retval None
  */
void ResetBNO055(void){

	uint8_t reset = 0x20;
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, SYS_TRIGGER_ADDR, 1, &reset, 1, 100);
	HAL_Delay(500);

	//Checking for is reset process done
	uint8_t chip_id=0;
	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, CHIP_ID_ADDR, 1, &chip_id, 1, 100);

	//If value of id register is not equal to BNO055 chip id which is 0xA0, wait until equal to each other
	while(chip_id != BNO055_ID) {
		printf("BNO055-> Undefined chip id\n");
		HAL_Delay(500);
	}
}

/*!
 *   @brief  Reads various data measured by BNO055
 *
 *   @param  Register base address of the data to be read
 * 			Possible arguments
 * 			[SENSOR_ACCEL
 *			 SENSOR_GYRO
 * 			 SENSOR_MAG
 *			 SENSOR_EULER
 *			 SENSOR_LINACC
 *			 SENSOR_GRAVITY
 *			 SENSOR_QUATERNION]
 *
 *   @retval Structure containing the values ​​of the read data
 */
void ReadData(BNO055_Sensors_t *sensorData,BNO055_Sensor_Type sensors){


	   uint8_t buffer[8];

	    if (sensors & SENSOR_GRAVITY) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_GRAVITY, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Gravity.X = (float)(((int16_t)((buffer[1] << 8) | buffer[0]))/100.0);
	        sensorData->Gravity.Y = (float)(((int16_t)((buffer[3] << 8) | buffer[2]))/100.0);
	        sensorData->Gravity.Z = (float)(((int16_t)((buffer[5] << 8) | buffer[4]))/100.0);
	        memset(buffer, 0, sizeof(buffer));
	    }

	    if (sensors & SENSOR_QUATERNION) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_QUATERNION, 1, buffer, 8, HAL_MAX_DELAY);
	        sensorData->Quaternion.W = (float)(((int16_t)((buffer[1] << 8) | buffer[0]))/(1<<14));
	        sensorData->Quaternion.X = (float)(((int16_t)((buffer[3] << 8) | buffer[2]))/(1<<14));
	        sensorData->Quaternion.Y = (float)(((int16_t)((buffer[5] << 8) | buffer[4]))/(1<<14));
	        sensorData->Quaternion.Z = (float)(((int16_t)((buffer[7] << 8) | buffer[6]))/(1<<14));
	        memset(buffer, 0, sizeof(buffer));
	    }

	    if (sensors & SENSOR_LINACC) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_LINACC, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->LineerAcc.X = (float)(((int16_t)((buffer[1] << 8) | buffer[0]))/100.0);
	        sensorData->LineerAcc.Y = (float)(((int16_t)((buffer[3] << 8) | buffer[2]))/100.0);
	        sensorData->LineerAcc.Z = (float)(((int16_t)((buffer[5] << 8) | buffer[4]))/100.0);
	        memset(buffer, 0, sizeof(buffer));
	    }

	    if (sensors & SENSOR_GYRO) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_GYRO, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Gyro.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/16.0);
	        sensorData->Gyro.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/16.0);
	        sensorData->Gyro.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/16.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
	    if (sensors & SENSOR_ACCEL) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_ACCEL, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Accel.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/100.0);
	        sensorData->Accel.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/100.0);
	        sensorData->Accel.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/100.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
	    if (sensors & SENSOR_MAG) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_MAG, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Magneto.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/16.0);
	        sensorData->Magneto.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/16.0);
	        sensorData->Magneto.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/16.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
	    if (sensors & SENSOR_EULER) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_EULER, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Euler.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/16.0);
	        sensorData->Euler.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/16.0);
	        sensorData->Euler.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/16.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  Operation modes
 *          Mode Values;
 *           [CONFIG_MODE
 *            ACC_ONLY
 *            MAG_ONLY
 *            GYR_ONLY
 *            ACC_MAG
 *            ACC_GYRO
 *            MAG_GYRO
 *            AMG
 *            IMU
 *            COMPASS
 *            M4G
 *            NDOF_FMC_OFF
 *            NDOF]
 *
 *  @retval None
 */
void Set_Operation_Mode(Op_Modes_t Mode){

	SelectPage(PAGE_0);
	if(	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, OPR_MODE_ADDR, 1, &Mode, 1, 100) !=HAL_OK){
		printf("Operation mode could not be set!\n");
	}
	else printf("Operation mode switching succeeded.\n");

	if(Mode == CONFIG_MODE) HAL_Delay(19);

	else HAL_Delay(9);

}

/*!
 *  @brief  Set the power mode of BNO055
 *  @param  power modes
 *          possible values
 *           [BNO055_NORMAL_MODE
 *            BNO055_LOWPOWER_MODE
 *            BNO055_SUSPEND_MODE]
 *
 *  @retval None
 */
void SetPowerMODE(uint8_t BNO055_){

	if(	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, PWR_MODE_ADDR, 1, &BNO055_, 1, 100) != HAL_OK)
	{
		printf("Power mode could not be set!\n");
	}
	else
	{
		printf("Power mode switching succeeded.\n");
	}
	HAL_Delay(50);

}

/*!
 *  @brief  Selects the chip's clock source
 *  @param  Source
 *          possible values
 *           [CLOCK_EXTERNAL
 *            CLOCK_INTERNAL]
 *
 *  @retval None
 */
void Clock_Source(uint8_t source) {

	//7th bit: External Crystal=1; Internal Crystal=0
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, SYS_TRIGGER_ADDR, 1, &source, sizeof(source), 100);
}

/*!
 *  @brief  Changes the chip's axis signs and remap
 *  @param  remapcode and signcode
 *         	Default Parameters:[DEFAULT_AXIS_REMAP(0x24), DEFAULT_AXIS_SIGN(0x00)]
 *
 *  @retval None
 */
void BNO055_Axis(uint8_t remap, uint8_t sign){

	HAL_I2C_Mem_Write(&bno_i2c,P_BNO055, AXIS_MAP_CONFIG_ADDR, 1, &remap, 1, 100);
	HAL_Delay(20);
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, AXIS_MAP_SIGN_ADDR, 1, &sign, 1, 100);
	HAL_Delay(100);
}

/*!
 *  @brief  Sets the accelerometer range
 *  @param  range
 *          possible values
 *           [Range_2G
 *            Range_4G
 *            Range_8G
 *            Range_16G]
 *
 *  @retval None
 */
void SET_Accel_Range(uint8_t range){

	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, ACC_CONFIG_ADDR, 1, &range, 1, 100);
	HAL_Delay(100);

}

/**
  * @brief  Initialization of BNO055
  *
  * @param  Init argument to a BNO055_Init_t structure that contains
  *         the configuration information for the BNO055 device.
  *
  * @retval None
  */
void BNO055_Init(BNO055_Init_t Init){

	//Set operation mode to config_mode for initialize all register
	Set_Operation_Mode(CONFIG_MODE);
	HAL_Delay(50);
	/*
	 * Set register page number to 1
	 * Configure Accelerometer range
	 */
	SelectPage(PAGE_1);
	SET_Accel_Range(Init.ACC_Range);
	HAL_Delay(50);

	//Set register page number to 0
	SelectPage(PAGE_0);
	HAL_Delay(50);

	//Read clock status. If status=0 then it is free to configure the clock source
	uint8_t status;
	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, SYS_CLK_STATUS_ADDR, 1, &status, 1, 100);
	HAL_Delay(50);
	//Checking if the status bit is 0
	if(status == 0)
	{
		//Changing clock source
		Clock_Source(Init.Clock_Source);
		HAL_Delay(100);
	}

	//Configure axis remapping and signing
	BNO055_Axis(Init.Axis, Init.Axis_sign);
	HAL_Delay(100);

	//Configure data output format and the measurement unit
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, UNIT_SEL_ADDR, 1, &Init.Unit_Sel, sizeof(Init.Unit_Sel), 100);
	HAL_Delay(100);

	//Set power mode
	SetPowerMODE(Init.Mode);
	HAL_Delay(100);

	//Set operation mode
	Set_Operation_Mode(Init.OP_Modes);
	HAL_Delay(100);

	printf("BNO055 Initialization process is done!\n");
}

/**
  * @brief  Gets calibration status of accel, gyro, mag and system
  *
  * @param  None
  *
  * @retval Calib_status_t structure that contains
  *         the calibration status of accel, gyro, mag and system.
  */
void getCalibration(Calib_status_t *calib) {
    uint8_t calData;

    // Read calibration status register using I2C
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, P_BNO055, CALIB_STAT_ADDR, 1, &calData, 1, HAL_MAX_DELAY);

    // Check if read was successful
    if (status == HAL_OK) {

        // Extract calibration status values

        	calib->System= (calData >> 6) & 0x03;


        	calib->Gyro = (calData >> 4) & 0x03;


        	calib->Acc = (calData >> 2) & 0x03;


        	calib->MAG = calData & 0x03;

    } else {
        printf("Failed to read calibration status register.\n");
    }
}

/**
  * @brief  Gets sensor offsets
  *
  * @param  22 byte long buffer to hold offset data
  *
  * @retval None
  *
  */
void getSensorOffsets(uint8_t *calibData) {

        // Save the current mode
        uint8_t lastMode = getCurrentMode();

        // Switch to CONFIG mode
        Set_Operation_Mode(CONFIG_MODE);
        printf("Switched to CONFIG mode.\n");

        // Read the offset registers
        HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, ACC_OFFSET_X_LSB_ADDR, 1, calibData, 22, 100);
        printf("Calibration data obtained.\n");

        // Restore the previous mode
        Set_Operation_Mode(lastMode);
        printf("Restored to previous mode.\n");
}

/**
  * @brief  Sets sensor offsets
  *
  * @param  22 byte long buffer containing offset data
  *
  * @retval None
  *
  */
void setSensorOffsets(const uint8_t *calibData) {
    uint8_t lastMode = getCurrentMode();

    // Switch to CONFIG mode
    Set_Operation_Mode(CONFIG_MODE);
    printf("Switched to CONFIG mode.\n");

    // Write calibration data to the sensor's offset registers using memory write
    HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, ACC_OFFSET_X_LSB_ADDR, 1, (uint8_t *)calibData, 22, 100);
    printf("Wrote calibration data to sensor's offset registers.\n");

    // Restore the previous mode
    Set_Operation_Mode(lastMode);
    printf("Restored to previous mode.\n");
}

/**
  * @brief  Checks the calibration status of the sensor
  *
  * @param  None
  *
  * @retval True of False
  *
  */
bool isFullyCalibrated(void) {
//    Calib_status_t calib ={0};
    Calib_status_t calib ={0};
    getCalibration(&calib);


    switch (getCurrentMode()) {
        case ACC_ONLY:
            return (calib.Acc == 3);
        case MAG_ONLY:
            return (calib.MAG == 3);
        case GYRO_ONLY:
        case M4G: /* No magnetometer calibration required. */
            return (calib.Gyro == 3);
        case ACC_MAG:
        case COMPASS:
            return (calib.Acc == 3 && calib.MAG == 3);
        case ACC_GYRO:
        case IMU:
            return (calib.Acc == 3 && calib.Gyro == 3);
        case MAG_GYRO:
            return (calib.MAG == 3 && calib.Gyro == 3);
        default:
            return (calib.System == 3 && calib.Gyro == 3 && calib.Acc == 3 && calib.MAG == 3);
    }
}

/**
  * @brief  Gets the current operating mode of the chip
  *
  * @param  None
  *
  * @retval Operating mode
  *
  */
Op_Modes_t getCurrentMode(void) {

	Op_Modes_t mode;

	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, OPR_MODE_ADDR, 1, &mode, 1, 100);

    return mode;
}

/**
  * @brief  Calibrates BNO055
  *
  * @param  None
  *
  * @retval None
  *
  */
bool Calibrate_BNO055(void) {

		Calib_status_t calib={0};
        printf("Calibrating BNO055 sensor...\n");

        // Set operation mode to FUSION_MODE or appropriate mode for calibration
        Set_Operation_Mode(NDOF);
    	HAL_Delay(100);
        // Gyroscope calibration
        printf("Calibrating gyroscope...\n");
        printf("Place the device in a single stable position\n");
        HAL_Delay(1000);  // Simulated gyroscope calibration time

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.Gyro !=3);
        printf("Gyroscope calibration complete.\n");

        // Accelerometer calibration
        printf("Calibrating accelerometer...\n");
        printf("Place the device in 6 different stable positions\n");
        for (int i = 0; i < 6; i++) {
            printf("Position %d\n", i + 1);
            HAL_Delay(1500);  // Simulated accelerometer calibration time
        }

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.Acc !=3);
        printf("Accelerometer calibration complete.\n");

        // Magnetometer calibration
        printf("Calibrating magnetometer...\n");
        printf("Make some random movements\n");
        HAL_Delay(1000);  // Simulated gyroscope calibration time

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.MAG !=3);
        printf("Magnetometer calibration complete.\n");

        // System calibration
        printf("Calibrating system...\n");
        printf("Keep the device stationary until system calibration reaches level 3\n");
        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.System !=3);
        HAL_Delay(500);

        // Check calibration status
        while(!isFullyCalibrated()) HAL_Delay(500);
        printf("Sensor is fully calibrated.\n");

        printf("System: %d      Gyro: %d       Accel: %d       MAG: %d\n",calib.System,calib.Gyro , calib.Acc, calib.MAG);
        if(isFullyCalibrated()) return true;
        else return false;
}
