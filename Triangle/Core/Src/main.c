/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h> 
#include <mpu_6050.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t encoder = 0 ; 
float phi_dot = 0.0f;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define ENCODER_CPR 400
uint16_t motor_duty =  0;
float get_motor_speed(void);
volatile float motor_speed_rpm;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // setup();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  MPU6050_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // char buf[32];
  char buffer[512];
  float accel[3], gyro[3]; 
  motor_duty = 0.5;

  while (1)
  {
      // HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
      encoder = __HAL_TIM_GET_COUNTER(&htim4);    
      // int len = sprintf(buf, "Received Value: %lu\r\n", encoder); 
      // HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);

      MPU6050_ReadData(accel, gyro);

      // sprintf(buffer, "AX:%f AY:%f AZ:%f GX:%f GY:%f GZ:%f\r\n",
      //           accel[0], accel[1], accel[2],
      //           gyro[0], gyro[1], gyro[2]);
      
      uint8_t id = MPU6050_ReadByte(0x75);
      sprintf(buffer, "WHO_AM_I = 0x%02X\r\n", id);
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
      float gx_rad = gyro[0] * (M_PI / 180.0f);
      float theta = atan2(accel[1],accel[2]) * 180.0f / M_PI ;
      float phi = encoder * 2.0f * M_PI/ENCODER_CPR;
      phi_dot = get_motor_speed();
      snprintf(buffer, sizeof(buffer),
            "AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d theta : %f thetadot: %f phi : %f phi_dot : %f\r\n",
            (int)(accel[0]*1000), (int)(accel[1]*1000), (int)(accel[2]*1000),
            (int)(gyro[0]*100), (int)(gyro[1]*100), (int)(gyro[2]*100), theta, gx_rad, phi, phi_dot) ;
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
      htim1.Instance->CCR1 = (uint32_t)(motor_duty * htim1.Instance->ARR);     
      HAL_Delay(10);  
      // encoder = __HAL_TIM_GET_COUNTER(&htim2);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
float get_motor_speed() {
    static int32_t prev_count = 0;
    static uint32_t prev_time = 0;   
    int32_t count = __HAL_TIM_GET_COUNTER(&htim4);
    int32_t delta = count - prev_count;
    prev_count = count;

    uint32_t now_time = HAL_GetTick(); 
    float dt = (now_time - prev_time) / 1000.0f;  
    prev_time = now_time;

    return ((float)delta / ENCODER_CPR) * 2.0f * M_PI / dt; // rad/s
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
