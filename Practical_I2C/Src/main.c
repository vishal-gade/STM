/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
	uint8_t Address; 
	uint16_t Value; 
	uint16_t Vol[3]; // 
	uint8_t Slave_Address=0x2A;  // Slave address (FDC address is 0x2A given in data sheet)
	uint8_t Tx_Data[3]; // Transmtting Data
	uint8_t reg_ptr = 0x00; // Slave register pointer
	uint8_t Rx_Data[3]; // Receiving data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void write_register(uint8_t register_pointer, uint16_t register_value)
{
    

    Tx_Data[0] = register_pointer;     // 0x00 pointer to the register
    Tx_Data[1] = register_value>>8;    // MSB byte of 16bit data
    Tx_Data[2] = register_value;       // LSB byte of 16bit data
// Tx_Data is the array containing register pointer (1 byte) and Data (2 bytes)
    HAL_I2C_Master_Transmit(&hi2c1, Slave_Address<<1, Tx_Data, 3, 100);  
}

void read_register(uint8_t register_pointer, uint8_t* receive_buffer)
{
    // Set the register pointer to, the register wanted to be read
    HAL_I2C_Master_Transmit(&hi2c1, Slave_Address<<1, &register_pointer, 1, 100); 
	 // note the & operator which gives us the address of the register_pointer variable

    // receive the two 8-bit data into the receive buffer
    HAL_I2C_Master_Receive(&hi2c1, Slave_Address<<1, receive_buffer, 2, 100);   
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(10);
}

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_StatusTypeDef result; // 
 	uint8_t i;
 	for (i=1; i<128; i++) // Scanning the I2C Bus for Slaves
 	{
		// Checking weather a device is connected to I2C bus or not
		result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2); 
		if (result == HAL_OK)
 	  {	
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(500);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			Address=i;
			//
				write_register(0x08, 0x2089);// 0x08 is the address of R_COUNT register in FDC
				write_register(0x10, 0x000A);// 0x10 is the address of SETTLE_COUNT register in FDC
				write_register(0x14, 0x2002);// 0x14 is the address of CLOCKDIVIDERS register in FDC
				write_register(0x19, 0x0000);// 0x19 is the address of ERROR_CONFIG register in FDC
				write_register(0x1B, 0x020D);// 0x1B is the address of CLOCKDIVIDERS register in FDC
				write_register(0x1E, 0x7C00);// 0x1E is the address of MUX_CONFIG register in FDC
				write_register(0x1A, 0x1481);// 0x1A is the address of CONFIG register in FDC
				HAL_Delay(10);			
		}
 	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		read_register(reg_ptr, Rx_Data); // Reading the data from reg_ptr and storing it in Rx_Data
		Value=Rx_Data[0];
		Value<<=8;
		Value+=Rx_Data[1];
		Vol[0]=Value;
		HAL_Delay(10);
		read_register(reg_ptr, Rx_Data);
		Value=Rx_Data[0];
		Value<<=8;
		Value+=Rx_Data[1];
		Vol[1]=Value;
		
		if ((Vol[0]>=Vol[1]) && (Value>219))
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (GPIO_PinState)SET);//Water
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState)SET);//Motor
			}
		else if ((Vol[0]>=Vol[1]) && (Value<217))
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState)SET);//Motor
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (GPIO_PinState)SET);//HalfWater
		}
		else if ((Vol[0]>=Vol[1]) && (Value<216))
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (GPIO_PinState)RESET);//Water
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState) RESET);//Motor
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (GPIO_PinState) RESET);//Water
		}
		else if ((Vol[0]<Vol[1]) && (Value<217))
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState) RESET);//Motor
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (GPIO_PinState)SET);//HalfWater
		}
		else if ((Vol[0]<=Vol[1]) && (Value>219))
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState)SET);//Motor
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (GPIO_PinState)SET);//LowWater
		}
		
		
  }
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
