# stm32-4-4-rc-car
4x4 RC car using an STM32 microcontroller is an exciting project that combines hardware and software to control a vehicle wirelessly
-----
Components Needed
-----
STM32F4 Discovery Board: The main microcontroller board that will control the car.

HC-05 Bluetooth Module: For wireless communication between the car and a smartphone or remote control.

L298N Motor Driver: To control the speed and direction of the motors.

4 DC Motors: For driving the wheels.

Power Supply: To power the motors and the microcontroller.

Jumper Wires: For making connections between components.

Chassis: The frame of the car to mount all components.

Connections
HC-05 Bluetooth Module:

VCC to 5V

GND to GND

TX to PA10 (USART1_RX)

RX to PA9 (USART1_TX)

L298N Motor Driver:

IN1 to PB0

IN2 to PB1

IN3 to PB2

IN4 to PB3

ENA to PB4 (PWM)

ENB to PB5 (PWM)

VCC to 12V (or appropriate motor voltage)

GND to GND

Motor connections to the output terminals

Software Implementation
The software for the STM32 microcontroller will handle the following tasks:

Bluetooth Communication: Receiving commands from a smartphone or remote control via the HC-05 module.

Motor Control: Using PWM signals to control the speed and direction of the motors via the L298N motor driver.

Command Processing: Interpreting the received commands and executing the corresponding actions (e.g., moving forward, backward, turning left, right, or stopping).

Example Code
Here's an example of how the code might look:

c
#include "main.h"
#include "usb_host.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void MX_USB_HOST_Process(void);

void Motor_Control(char command);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  while (1)
  {
    MX_USB_HOST_Process();

    char command;
    if (HAL_UART_Receive(&huart1, (uint8_t *)&command, 1, HAL_MAX_DELAY) == HAL_OK) {
        Motor_Control(command);
    }
  }
}

void Motor_Control(char command) {
    switch (command) {
        case 'F': // Forward
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000); // Set PWM duty cycle
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000); // Set PWM duty cycle
            break;
        case 'B': // Backward
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000); // Set PWM duty cycle
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000); // Set PWM duty cycle
            break;
        case 'L': // Left
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000); // Set PWM duty cycle
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000); // Set PWM duty cycle
            break;
        case 'R': // Right
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000); // Set PWM duty cycle
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000); // Set PWM duty cycle
            break;
        case 'S': // Stop
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); // Stop PWM
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); // Stop PWM
            break;
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

