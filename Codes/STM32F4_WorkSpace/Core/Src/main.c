/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "MPU6050.h"
#include "DCmotor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char *p, int len){		// printf�???????????????????????????????????? USART6?�� ?���???????????????????????????????????? ?��?�� ?��?��
	for (int i = 0; i < len ; i++){
		while(!LL_USART_IsActiveFlag_TXE(USART6));
		LL_USART_TransmitData8(USART6, *(p+i));
	}
	return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern unsigned char uart_rx_flag;
extern unsigned char uart_rx_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	double target_roll = 0.0;
	double roll_PID = 0.0;
	double roll_err;
	double roll_err_sum = 0;
	double roll_err_dt;
	double roll_err_prev;

	short motor_input;

	double dt_double = 0.001;

	double Kp = 40;
	double Ki = 0;
	double Kd = 0;

	double roll_P;
	double roll_I;
	double roll_D;
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
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  MPU6050_Calibration();
  DCmotor_Init();
  LL_TIM_EnableIT_UPDATE(TIM3);
  LL_TIM_EnableCounter(TIM3);
  printf("Start Balancing!\n\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  getDeltaTime();
//	  if (dt == 0) dt = 1;
//	  dt_double = dt / 1000.f;

	  MPU6050_GetAccel();
	  MPU6050_GetGyro();
	  MPU6050_GetRoll_Acc();
	  MPU6050_GetRoll_Gyr();
	  MPU6050_getRoll_Filtered();

	  if (IMU.roll_filtered > 15.0) IMU.roll_filtered = 15.0;	// roll 최대, 최소 제한
	  else if (IMU.roll_filtered <= -15.0) IMU.roll_filtered = -15.0;

	  roll_err = target_roll - IMU.roll_filtered;				// roll error 계산
	  if (isnan(roll_err) != 0) roll_err = roll_err_prev;		// roll 값이 크게 튀어 nan이 되는 경우 배제

	  roll_err_sum += roll_err * dt_double;						// roll error 적분 계산
	  if (roll_err_sum > 15.0) roll_err_sum = 15.0;				// roll errer 적분 최대, 최소 제한
	  else if (roll_err_sum <= -15.0) roll_err_sum = -15.0;

	  roll_err_dt = (roll_err - roll_err_prev) / dt_double;		// roll error 미분 계산
	  roll_err_prev = roll_err;									// roll error 이전값 저장

	  roll_P = Kp * roll_err;									// roll P, I, D 계산
	  roll_I = Ki * roll_err_sum;
	  roll_D = -Kd * roll_err_dt;

	  roll_PID = roll_P + roll_I + roll_D;						// roll PID 계산

	  if (roll_PID > 0){										// PID 값으로 모터 구동(PWM 최대 제한)
		  motor_input = 100 + roll_PID;
		  if (motor_input > 800) motor_input = 800;
		  DCmotor_Backward(motor_input);
		  printf("%d\n", motor_input);
	  }
	  else {
		  motor_input = 100 - roll_PID;
		  if (motor_input > 800) motor_input = 800;
		  DCmotor_Forward(motor_input);
		  printf("%d\n", motor_input);
	  }

//	  printf("%d\n", motor_input);

//	  printf("%.1f\n", IMU.roll_filtered);

//	  printf("%d  %d  %d\n", IMU.ax, IMU.ay, IMU.az);

//	  printf("Hello World!\n");
//	  HAL_Delay(500);

//	  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
//	  HAL_Delay(500);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
