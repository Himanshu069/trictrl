/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — HSI Clock Fix & Prototype Declared
  *
  * CHANGE LOG:
  * - Added explicit forward declaration prototype for SystemClock_Config()
  * to eliminate implicit compiler warnings and layout errors.
  * - SystemClock_Config() rewritten to use internal HSI oscillator.
  * Eliminates the infinite HAL wait loop that was freezing the chip
  * when the external 8 MHz crystal (HSE) failed to start on the custom PCB.
  * - System clock is now 64 MHz (HSI/2 × PLL×16), within all STM32F103 limits.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mpu_6050.h"

#include "stm32f1xx_hal.h"

/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);  /* Explicit prototype to prevent implicit function warnings */
float get_motor_speed(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t encoder = 0;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ENCODER_CPR 400

uint16_t motor_duty = 0;
volatile float motor_speed_rpm;

/* --- GLOBAL TRACKING VARIABLES FOR CUBEMONITOR --- */
float phi_dot  = 0.0f;
float theta    = 0.0f;
float phi      = 0.0f;
float sandeep  = 0.0f;
float thetadot = 0.0f;
float accel[3] = {0.0f, 0.0f, 0.0f};
float gyro[3]  = {0.0f, 0.0f, 0.0f};




/* --- VISUAL SOFTWARE DIAGNOSTIC FLAG ---
 * Starts at 99. Advances through each init milestone.
 * Monitor this variable in STM32CubeMonitor to diagnose freeze points.
 *
 * 99  = Frozen in SystemClock_Config() — clock hardware problem
 * 10  = Past clock init, GPIO initializing
 * 20  = DMA initializing
 * 30  = USART2 initializing
 * 40  = USART1 initializing
 * 50  = TIM4 (encoder) initializing
 * 60  = TIM1 (PWM) initializing
 * 1  = All peripherals OK
 * 2  = MPU6050 woke up over software I2C
 * 3  = Main loop running normally
 */
volatile uint8_t debug_step = 99;

/* ---------- SOFTWARE I2C TIMING DELAY ---------- */
static void I2C_Delay(void)
{
    for (volatile int i = 0; i < 500; i++);
}

// #ifdef STDIO_USB

// #include "usbd_cdc_if.h"

// int _write(int file, char *data, int len)
// {
//     CDC_Transmit_FS((uint8_t*)data, (uint16_t)len);
//     return len;
// }

// #else

// int _write(int file, char *data, int len)
// {
//     int i  = 0;
//     for (; i < len; ++i)
//     {
//         ITM_SendChar(data[i]);
//     }
//     return i;
// }

// #endif

/* ---------- SOFTWARE BIT-BANG I2C PROTOCOL ---------- */
void I2C_Start(void)
{
    SDA_HIGH(); SCL_HIGH(); I2C_Delay();
    SDA_LOW();              I2C_Delay();
    SCL_LOW();              I2C_Delay();
}

void I2C_Stop(void)
{
    SDA_LOW();
    SCL_HIGH(); I2C_Delay();
    SDA_HIGH(); I2C_Delay();
}

uint8_t I2C_WriteByte(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        if (data & 0x80) SDA_HIGH(); else SDA_LOW();
        data <<= 1;
        SCL_HIGH(); I2C_Delay();
        SCL_LOW();  I2C_Delay();
    }
    SDA_HIGH();
    SCL_HIGH(); I2C_Delay();
    uint8_t ack = SDA_READ();
    SCL_LOW();  I2C_Delay();
    return ack;
}

uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;
    SDA_HIGH();
    for (int i = 0; i < 8; i++)
    {
        data <<= 1;
        SCL_HIGH(); I2C_Delay();
        if (SDA_READ()) data |= 1;
        SCL_LOW();  I2C_Delay();
    }
    if (ack) SDA_LOW(); else SDA_HIGH();
    SCL_HIGH(); I2C_Delay();
    SCL_LOW();  I2C_Delay();
    SDA_HIGH();
    return data;
}

/* ---------- MPU6050 DRIVER FUNCTIONS ---------- */
void MPU6050_Init(void)
{
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 0);  /* Write address    */
    I2C_WriteByte(0x6B);               /* PWR_MGMT_1 reg   */
    I2C_WriteByte(0x00);               /* Clear sleep bit  */
    I2C_Stop();
}

uint8_t MPU6050_ReadByte(uint8_t reg)
{
    uint8_t data;
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 0);  /* Write address    */
    I2C_WriteByte(reg);
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 1);  /* Read address     */
    data = I2C_ReadByte(0);            /* NACK on last byte */
    I2C_Stop();
    return data;
}

void MPU6050_ReadData(float *accel_ms2, float *gyro_dps)
{
    uint8_t  buffer[14];
    int16_t  accel_raw[3], gyro_raw[3];

    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 0);
    I2C_WriteByte(0x3B);               /* Start at ACCEL_XOUT_H */
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 1);

    for (int i = 0; i < 13; i++) buffer[i] = I2C_ReadByte(1);
    buffer[13] = I2C_ReadByte(0);      /* NACK + Stop on last byte */
    I2C_Stop();

    accel_raw[0] = (int16_t)((buffer[0]  << 8) | buffer[1]);
    accel_raw[1] = (int16_t)((buffer[2]  << 8) | buffer[3]);
    accel_raw[2] = (int16_t)((buffer[4]  << 8) | buffer[5]);
    gyro_raw[0]  = (int16_t)((buffer[8]  << 8) | buffer[9]);
    gyro_raw[1]  = (int16_t)((buffer[10] << 8) | buffer[11]);
    gyro_raw[2]  = (int16_t)((buffer[12] << 8) | buffer[13]);

    for (int i = 0; i < 3; i++)
    {
        accel_ms2[i] = (float)accel_raw[i] / 16384.0f;  /* ±2g range  */
        gyro_dps[i]  = (float)gyro_raw[i]  / 131.0f;    /* ±250°/s range */
    }
}
/* USER CODE END 0 */

/* ============================================================
 * MAIN ENTRY POINT
 * ============================================================ */
int main(void)
{
    HAL_Init();


    /* SystemClock_Config() now uses HSI — will not freeze on crystal failure */
    SystemClock_Config();

    /* ---- PERIPHERAL INITIALIZATION WITH STEP DIAGNOSTICS ---- */
    debug_step = 10;
    MX_GPIO_Init();

    debug_step = 20;
    MX_DMA_Init();

    
    debug_step = 30;
    MX_USART2_UART_Init();
    
    debug_step = 40;
    MX_USART1_UART_Init();
    
    debug_step = 50;  /* If stuck here: TIM4 encoder config issue */
    MX_TIM4_Init();
    
    debug_step = 60;  /* If stuck here: TIM1 PWM config issue */
    // printf("hello");
    MX_TIM1_Init();

    /* USER CODE BEGIN 2 */
    debug_step = 1;   /* Milestone 1: All native peripherals initialized */
    HAL_Delay(100);

    /* ---- SOFTWARE I2C GPIO SETUP (PB0 = SCL, PB1 = SDA) ---- */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;   /* Open-drain for I2C */
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Drive lines high (idle state) before starting I2C traffic */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(50);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    MPU6050_Init();

    debug_step = 2;   /* Milestone 2: MPU6050 woken over software I2C */
    HAL_Delay(100);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_ALL);
    /* USER CODE END 2 */

    /* ============================================================
     * MAIN LOOP
     * ============================================================ */
    char buffer[512];
    motor_duty = 0;  /* Start motor at zero duty — safe default */

    while (1)
    {
        /* USER CODE BEGIN WHILE */
        debug_step = 3;  /* Milestone 3: Actively executing main loop */

        encoder = __HAL_TIM_GET_COUNTER(&htim4);
        MPU6050_ReadData(accel, gyro);

        sandeep += 10;

        /* Verify MPU6050 communication — WHO_AM_I should return 0x68 */
        uint8_t id = MPU6050_ReadByte(0x75);
        sprintf(buffer, "WHO_AM_I = 0x%02X\r\n", id);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

        /* ---- STATE ESTIMATION ---- */
        thetadot = gyro[0]  * (M_PI / 180.0f);          /* Gyro X → rad/s          */
        theta    = atan2f(accel[1], accel[2]) * 180.0f / (float)M_PI; /* Tilt angle °*/
        phi      = encoder  * 2.0f * M_PI / ENCODER_CPR; /* Wheel angle rad         */
        phi_dot  = get_motor_speed();                     /* Wheel speed rad/s       */

        /* ---- TELEMETRY OUTPUT ---- */
        snprintf(buffer, sizeof(buffer),
                 "AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d "
                 "theta:%f thetadot:%f phi:%f phi_dot:%f\r\n",
                 (int)(accel[0] * 1000), (int)(accel[1] * 1000), (int)(accel[2] * 1000),
                 (int)(gyro[0]  * 100),  (int)(gyro[1]  * 100),  (int)(gyro[2]  * 100),
                 theta, thetadot, phi, phi_dot);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

        /* ---- PWM OUTPUT ---- */
        htim1.Instance->CCR1 = (uint32_t)(motor_duty * htim1.Instance->ARR);

        HAL_Delay(10);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* ============================================================
 * SYSTEM CLOCK CONFIGURATION — HSI INTERNAL OSCILLATOR
 * ============================================================
 * SUMMARY:
 * Uses the internal HSI RC oscillator (8 MHz, always available).
 * HSI is divided by 2 inside the RCC hardware (gives 4 MHz input),
 * then multiplied by PLL ×16 → 64 MHz system clock.
 * Immune to PCB crystal startup failures.
 *
 * CLOCK TREE SUMMARY:
 * HSI (8 MHz) → /2 → 4 MHz → PLL ×16 → 64 MHz SYSCLK
 * HCLK  = SYSCLK / 1 = 64 MHz
 * APB1  = HCLK   / 2 = 32 MHz  (max allowed: 36 MHz)
 * APB2  = HCLK   / 1 = 64 MHz  (max allowed: 72 MHz)
 * Flash latency = 2 wait states  (required above 48 MHz)
 * ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* --- Step 1: Configure oscillator --- */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2; /* 8 MHz / 2 = 4 MHz */
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;          /* 4 MHz × 16 = 64 MHz */

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* --- Step 2: Route PLL to system clock --- */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 64 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     /* APB1  = 32 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2  = 64 MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* ============================================================
 * MOTOR SPEED CALCULATION
 * Uses encoder tick delta between calls to compute rad/s.
 * ============================================================ */
float get_motor_speed(void)
{
    static int32_t  prev_count = 0;
    static uint32_t prev_time  = 0;

    int32_t  count = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    int32_t  delta = count - prev_count;
    prev_count     = count;

    uint32_t now_time = HAL_GetTick();
    float    dt       = (float)(now_time - prev_time) / 1000.0f;
    if (dt < 0.0001f) dt = 0.001f;  /* Guard against divide-by-zero */
    prev_time = now_time;

    /* Convert encoder ticks to rad/s */
    return ((float)delta / ENCODER_CPR) * 2.0f * (float)M_PI / dt;
}
/* USER CODE END 4 */

/* ============================================================
 * ERROR HANDLER
 * ============================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}