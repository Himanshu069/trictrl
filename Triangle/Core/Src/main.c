/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with step diagnostics
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
#include "mpu_6050.h"
/* USER CODE END Includes */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t encoder = 0; 
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define ENCODER_CPR 400
uint16_t motor_duty = 0;
float get_motor_speed(void);
volatile float motor_speed_rpm;

// --- GLOBAL TRACKING VARIABLES FOR CUBEMONITOR ---
float phi_dot = 0.0f;
float theta = 0.0f;
float phi = 0.0f;
float sandeep = 0.0f;
float thetadot = 0.0f;
float accel[3] = {0.0f, 0.0f, 0.0f};
float gyro[3] = {0.0f, 0.0f, 0.0f};

// --- VISUAL SOFTWARE DIAGNOSTIC FLAG ---
// Starts at 99. Progresses through 10-60 during peripheral init. 
// Becomes 1 after hardware passes, 2 after I2C init, 3 inside main loop.
volatile uint8_t debug_step = 99; 

// Simple delay function for clock frequency matching in software I2C
static void I2C_Delay(void) {
    for (volatile int i = 0; i < 500; i++);
}

// ---------- SOFTWARE BIT-BANG I2C PROTOCOL FUNCTIONS ----------
void I2C_Start(void) {
    SDA_HIGH(); SCL_HIGH(); I2C_Delay();
    SDA_LOW();  I2C_Delay();
    SCL_LOW();  I2C_Delay();
}

void I2C_Stop(void) {
    SDA_LOW(); SCL_HIGH(); I2C_Delay();
    SDA_HIGH(); I2C_Delay();
}

uint8_t I2C_WriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) SDA_HIGH(); else SDA_LOW();
        data <<= 1;
        SCL_HIGH(); I2C_Delay();
        SCL_LOW();  I2C_Delay();
    }
    SDA_HIGH(); 
    SCL_HIGH(); I2C_Delay();
    uint8_t ack = SDA_READ();
    SCL_LOW(); I2C_Delay();
    return ack; 
}

uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t data = 0;
    SDA_HIGH(); 
    for (int i = 0; i < 8; i++) {
        data <<= 1;
        SCL_HIGH(); I2C_Delay();
        if (SDA_READ()) data |= 1;
        SCL_LOW(); I2C_Delay();
    }
    if (ack) SDA_LOW(); else SDA_HIGH();
    SCL_HIGH(); I2C_Delay();
    SCL_LOW();  I2C_Delay();
    SDA_HIGH(); 
    return data;
}

// ---------- MPU6050 HARDWARE FUNCTIONS ----------
void MPU6050_Init(void) {
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 0); 
    I2C_WriteByte(0x6B);              
    I2C_WriteByte(0x00);              
    I2C_Stop();
}

uint8_t MPU6050_ReadByte(uint8_t reg) {
    uint8_t data;
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 0); 
    I2C_WriteByte(reg);
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 1); 
    data = I2C_ReadByte(0);           
    I2C_Stop();
    return data;
}

void MPU6050_ReadData(float* accel_ms2, float* gyro_dps) {
    uint8_t buffer[14];
    int16_t accel_raw[3], gyro_raw[3];

    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 0);
    I2C_WriteByte(0x3B);              
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 1);

    for (int i = 0; i < 13; i++) buffer[i] = I2C_ReadByte(1); 
    buffer[13] = I2C_ReadByte(0);     
    I2C_Stop();

    accel_raw[0] = (buffer[0] << 8)  | buffer[1];
    accel_raw[1] = (buffer[2] << 8)  | buffer[3];
    accel_raw[2] = (buffer[4] << 8)  | buffer[5];
    gyro_raw[0]  = (buffer[8] << 8)  | buffer[9];
    gyro_raw[1]  = (buffer[10] << 8) | buffer[11];
    gyro_raw[2]  = (buffer[12] << 8) | buffer[13];

    for(int i = 0; i < 3; i++){
        accel_ms2[i] = ((float)accel_raw[i] / 16384.0f); 
        gyro_dps[i]  = (float)gyro_raw[i] / 131.0f;       
    }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  // --- STEP DIAGNOSTICS FOR INITIALIZATION BREAKPOINTS ---
  debug_step = 10; 
  MX_GPIO_Init();
  
  debug_step = 20; 
  MX_DMA_Init();
  
  debug_step = 30; 
  MX_USART2_UART_Init();
  
  debug_step = 40; 
  MX_USART1_UART_Init();
  
  debug_step = 50; // If stuck on 50, Timer 4 (Encoder input) is freezing
  MX_TIM4_Init();
  
  debug_step = 60; // If stuck on 60, Timer 1 (PWM output) is freezing
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  debug_step = 1; // Milestone 1: Reached after passing all native peripheral initializations
  HAL_Delay(100);

  // --- BIT-BANG GPIO LAYOUT ALIGNMENT (PB0 = SCL, PB1 = SDA) ---
  __HAL_RCC_GPIOB_CLK_ENABLE(); 
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;    
  GPIO_InitStruct.Pull = GPIO_PULLUP;           
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(50); 

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  
  MPU6050_Init();
  
  debug_step = 2; // Milestone 2: Woke up sensor over software I2C channels safely!
  HAL_Delay(100);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buffer[512];
  motor_duty = 0.5;

  while (1)
  {
      debug_step = 3; // Milestone 3: Actively processing loops
      
      encoder = __HAL_TIM_GET_COUNTER(&htim4);    
      MPU6050_ReadData(accel, gyro);


      sandeep += 10; 
      uint8_t id = MPU6050_ReadByte(0x75); 
      sprintf(buffer, "WHO_AM_I = 0x%02X\r\n", id);
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
      
      thetadot = gyro[0] * (M_PI / 180.0f); 
      theta = atan2(accel[1], accel[2]) * 180.0f / M_PI; 
      phi = encoder * 2.0f * M_PI / ENCODER_CPR; 
      phi_dot = get_motor_speed();
      
      snprintf(buffer, sizeof(buffer),
            "AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d theta : %f thetadot: %f phi : %f phi_dot : %f\r\n",
            (int)(accel[0]*1000), (int)(accel[1]*1000), (int)(accel[2]*1000),
            (int)(gyro[0]*100), (int)(gyro[1]*100), (int)(gyro[2]*100), theta, thetadot, phi, phi_dot);
      
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
      
      htim1.Instance->CCR1 = (uint32_t)(motor_duty * htim1.Instance->ARR);     
      HAL_Delay(10);  

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
    
    if(dt == 0) dt = 0.001f; 
    
    prev_time = now_time;
    return ((float)delta / ENCODER_CPR) * 2.0f * M_PI / dt; 
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}