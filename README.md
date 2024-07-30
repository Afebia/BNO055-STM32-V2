## Updates

- Added comments to each piece of code.
- Added BNO055 calibration code.
- Users can make detailed parameter adjustments.
- Added BNO055 system status reading function.
- Users can select which sensor data to read with a single function.

If you want to measure the speed of a car or the angles of a spacecraft, you can easily use the BNO055 sensor.
Today I created a library for BNO055 to use with STM32. With this library you can configure BNO055 and retrieve data from it.

**WARNING:** This library is written on an STM32F407-DISC1 so its header file is `stm32f4xx_hal.h`. İf you use another board, for example STM32F103C8 BluePill, just change that to `stm32f1xx_hal.h` in our BNO055_STM32.h header file.
```c
#include "stm32f1xx_hal.h"
```
**WARNING:** We used the `printf()` function in this library. Therefore, you should activate the printf function. If you don't know how to do that just google it :)

Firstly, you must create a project with CubeMX and activate I2C protocol. Then add the header file `BNO055_STM32.h` into the `../Core/Inc` directory and the `BNO055.c` source file into the `../Core/Src` directory. 

![image](https://github.com/user-attachments/assets/15f21c98-b0fa-40ed-b05a-060b219fff28)

**WARNING:** Change this line in the header file for whichever I2C protocol you have opened. For example hi2c1
```c
extern I2C_HandleTypeDef hi2c2;
#define bno_i2c (hi2c2)
```

The first thing you need to do before using the library is to change the I2C address in the `BNO055_STM32.h` header file according to the module you are using (it's `0x28` or `0x29`). If you are using Adafruit BNO055 module, this value is 0x28 or if you have designed a pcb and are using a BNO055 chip, this value changes depending on what you connect the COM3 pin of the chip to.

![image](https://github.com/user-attachments/assets/c96f267d-335d-4509-b448-7a7c3d5876f1)

```c
#define P_BNO055 (0x28<<1)
```

Now, lets explain how can use BNO055 library. With this library, you can obtain Gyro, Accel, Magnet, Euler Angles, Linear Accel, Quaternion and Gravity Vector data. The first thing you need to do is to set the parameters of the BNO055. So came to the main.c file of your project and create a function named like Sensor_Init or something.  
```c
void Sensor_Init(void);

void Sensor_Init(void)
{
	BNO_Status_t Status = {0};

	//Init structure definition section
	BNO055_Init_t BNO055_InitStruct = {0};

	//Reset section
	ResetBNO055();

	/*============================ *BNO055 Initialization* ============================*/

	BNO055_InitStruct.ACC_Range = Range_16G;			//Range_X
	BNO055_InitStruct.Axis = DEFAULT_AXIS_REMAP;			//value will be entered by looking at the data sheet
	BNO055_InitStruct.Axis_sign = DEFAULT_AXIS_SIGN;		//value will be entered by looking at the data sheet
	BNO055_InitStruct.Clock_Source = CLOCK_EXTERNAL;		//CLOCK_EXTERNAL or CLOCK_INTERNAL
	BNO055_InitStruct.Mode = BNO055_NORMAL_MODE;			//BNO055_X_MODE   X:NORMAL, LOWPOWER, SUSPEND
	BNO055_InitStruct.OP_Modes = NDOF;
	BNO055_InitStruct.Unit_Sel = (UNIT_ORI_ANDROID | UNIT_TEMP_CELCIUS | UNIT_EUL_DEG | UNIT_GYRO_DPS | UNIT_ACC_MS2);
									//(UNIT_ORI_X | UNIT_TEMP_X | UNIT_EUL_X | UNIT_GYRO_X | UNIT_ACC_X)
	BNO055_Init(BNO055_InitStruct);

	//------------------BNO055 Calibration------------------

	/*This function allows the sensor offset data obtained after BNO055 is calibrated once to be written
	 *to the registers. In this way, there is no need to calibrate the sensor every time it is powered on.
	 */
	//setSensorOffsets(OffsetDatas);

	/*-=-=-=-=-=-=Calibration Part-=-=-=-=-=-=*/
	if(Calibrate_BNO055())
	{
		getSensorOffsets(OffsetDatas);
		Buzzer(200, 5);
	}
	else
	{
		printf("Sensor calibration failed.\nFailed to retrieve offset data\n");
	}

	Check_Status(&Status);
	printf("Selftest Result: %d\t",Status.STresult);
	printf("System Status: %d\t",Status.SYSStatus);
	printf("System Error: %d\n",Status.SYSError);

}
```
- Save the sensor parameter settings in the structure named `BNO055_InitStruct.` The values ​​that can be written in the comments section for each parameter are shown. For example, if you type `Range_` and then press `Ctrl + space`, you can see the current values.
- Sensor calibration code has been added to the function. If you debug the project, you can perform the calibration on the SWV console. After the sensor is calibrated, the sensor offset data is saved in a buffer. To do this, you must create a buffer named `uint8_t OffsetDatas[22];` somewhere above the main.c file.
- The BNO055 sensor must be calibrated each time it is turned on, or the offset data obtained after calibration must be stored in the offset registers. After calibrating the sensor, store the offset data in the `OffsetDatas` buffer, then remove the `setSensorOffsets(OffsetDatas);` function from comment line and add calibration part in the comment line.

And that's all. To initialize the BNO055 sensor, all you need to do is run the `Sensor Init();` function before the while loop. So how can we access the sensor data? To do this, define a variable named `BNO055_Sensors_t BNO055;` at the top of the main.c file. The `BNO055_Sensors_t` structure holds the sensor datas on each axis ​​and its structure is as follows.
```c
typedef struct{ //SENSOR DATAS
	BNO055_Data_XYZ_t Accel;
	BNO055_Data_XYZ_t Gyro;
	BNO055_Data_XYZ_t Magneto;
	BNO055_Data_XYZ_t Euler;
	BNO055_Data_XYZ_t LineerAcc;
	BNO055_Data_XYZ_t Gravity;
	BNO055_QuaData_WXYZ_t Quaternion;
}BNO055_Sensors_t;
```
So if you want to see the acceleration in the x direction, you need to look at the variable BNO055.Accel.X, which is a float data type in our example. 
Now come to the while loop and write this function.

```c
ReadData(&BNO055, SENSOR_EULER|SENSOR_ACCEL|SENSOR_GYRO);       /*!Possible Values: SENSOR_ACCEL, SENSOR_MAG, SENSOR_LINACC,
								SENSOR_GRAVITY, SENSOR_QUATERNION, SENSOR_GYRO, SENSOR_EULER */	
```
If you want to get whatever data, write the parameters shown in the comment section using the "or bitwise operator" between them. For example, if you want to get only quaternion and gravity data, send `SENSOR_GRAVITY | SENSOR_QUATERNION` as an argument to the function.

Thats all you obtained sensor datas.

![github2](https://github.com/Afebia/BNO055-STM32/assets/121301223/5a1dd1b4-7d8a-45ff-853b-990189a4fa36)


### Full example
```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO055_STM32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

BNO055_Sensors_t BNO055;	/*!BNO055 data structure definition which hold euler, quaternion, linearaccel, gyro etc. parameters*/
uint8_t OffsetDatas[22];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Sensor_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  Sensor_Init();  				/*!Initialization of sensor groups*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  ReadData(&BNO055, SENSOR_EULER|SENSOR_ACCEL|SENSOR_GYRO);     /*!Possible Values: SENSOR_ACCEL, SENSOR_MAG, SENSOR_LINACC,
									SENSOR_GRAVITY, SENSOR_QUATERNION SENSOR_GYRO, SENSOR_EULER	*/
  }
  /* USER CODE END 3 */
}

void Sensor_Init(void)
{
	BNO_Status_t Status = {0};

	//Init structure definition section
	BNO055_Init_t BNO055_InitStruct = {0};

	//Reset section
	ResetBNO055();

	/*============================ *BNO055 Initialization* ============================*/

	BNO055_InitStruct.ACC_Range = Range_16G;			//Range_X
	BNO055_InitStruct.Axis = DEFAULT_AXIS_REMAP;			//value will be entered by looking at the data sheet
	BNO055_InitStruct.Axis_sign = DEFAULT_AXIS_SIGN;		//value will be entered by looking at the data sheet
	BNO055_InitStruct.Clock_Source = CLOCK_EXTERNAL;		//CLOCK_EXTERNAL or CLOCK_INTERNAL
	BNO055_InitStruct.Mode = BNO055_NORMAL_MODE;			//BNO055_X_MODE   X:NORMAL, LOWPOWER, SUSPEND
	BNO055_InitStruct.OP_Modes = NDOF;
	BNO055_InitStruct.Unit_Sel = (UNIT_ORI_ANDROID | UNIT_TEMP_CELCIUS | UNIT_EUL_DEG | UNIT_GYRO_DPS | UNIT_ACC_MS2);
									//(UNIT_ORI_X | UNIT_TEMP_X | UNIT_EUL_X | UNIT_GYRO_X | UNIT_ACC_X)
	BNO055_Init(BNO055_InitStruct);

	//------------------BNO055 Calibration------------------

	/*This function allows the sensor offset data obtained after BNO055 is calibrated once to be written
	 *to the registers. In this way, there is no need to calibrate the sensor every time it is powered on.
	 */
	//setSensorOffsets(OffsetDatas);

	/*-=-=-=-=-=-=Calibration Part-=-=-=-=-=-=*/
	if(Calibrate_BNO055())
	{
		getSensorOffsets(OffsetDatas);
		Buzzer(200, 5);
	}
	else
	{
		printf("Sensor calibration failed.\nFailed to retrieve offset data\n");
	}

	Check_Status(&Status);
	printf("Selftest Result: %d\t",Status.STresult);
	printf("System Status: %d\t",Status.SYSStatus);
	printf("System Error: %d\n",Status.SYSError);

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD10 PD12 PD1 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
```
