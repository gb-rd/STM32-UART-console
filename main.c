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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <stdio.h>
#include <string.h>

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t flag_uart_received = 0;
typedef enum {LINE_OK=1, LINE_ERR, LINE_PROGRESS} line_status_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void print_to_uart(const char* msg);
void read_uart(void);
void parse_buffer(uint8_t *text_buffer);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  read_uart();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief This function is a wrapper over HAL_UART_Transmit.
  * @param Pointer to message char array
  * @retval None
  */
void print_to_uart(const char *msg) {
    const uint8_t timeout = 100;
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), timeout);
}

/**
  * @brief This function is executed if UART data is ready to parse.
  * @param Pointer to the buffer to parse
  * @retval None
  */
void parse_buffer(uint8_t *text_buffer)
{
    const char *str = (char*)text_buffer;

    const char *msg_go_reset  = "\r\n*Going to reset*\r\n";
    const char *msg_help      = "\r\nAvailable commands:\r\n"
                                "    help            print this help\r\n"
                                "    status          get onboard led state\r\n"
                                "    led=[on|off]    switch onboard led\r\n"
                                "    reset           reset controller\r\n\r\n";
    const char *msg_len_on    = "\r\nlen=on\r\n";
    const char *msg_len_off   = "\r\nlen=off\r\n";
    const char *msg_new_line  = "\r\n";


    /* compare input with exact num of symbols to get rid of \r \n etc. */
    if (strncasecmp(str, "help", strlen("help")) == 0) {
        print_to_uart(msg_help);
    }
    else if (strncasecmp(str, "reset", strlen("reset")) == 0) {
        print_to_uart(msg_go_reset);
        NVIC_SystemReset();
	}
    else if (strncasecmp(str, "led=on", 6) == 0) {
        print_to_uart(msg_new_line);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }
    else if (strncasecmp(str, "led=off", 7) == 0) {
        print_to_uart(msg_new_line);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
    else if (strncasecmp(str, "status", 6) == 0) {
        if (HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin)) {
            print_to_uart(msg_len_on);
        }
        else {
            print_to_uart(msg_len_off);

        }
    }
    else {
        print_to_uart(msg_new_line);
    }
}

/**
  * @brief  This function is executed as main routine.
  * @retval None
  */
void read_uart(void)
{
    uint8_t line_received = 0;             // Initialize typedef enum
    uint8_t byte_buffer = 0;               // Store single byte

    const uint8_t TEXT_BUF_SIZE = 10;      // Expected maximal command length
    uint8_t text_buffer[TEXT_BUF_SIZE];    // Buffer with space for terminating \0
    size_t text_index = 0;                 // Index of buffer

    const uint32_t timer_delay = 5000;     // Timer in Ticks from last input
    uint32_t timer_start = HAL_GetTick();  // Store last input
    uint8_t flag_time_out = 0;             // Flag for input timer

    const char *msg_ready      = "\r\n*Type \"help\" to see available commands*\r\n";
    const char *msg_time_out   = "\r\n*Input timed out*\r\n";
    const char *msg_resetted   = "\r\n*System resetted*\r\n";

    /* check if controller was reset by software and clear cause flag */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        __HAL_RCC_CLEAR_RESET_FLAGS();  // (RCC->CSR |= RCC_CSR_RMVF)
        print_to_uart(msg_resetted);
    }
    else
    {
        print_to_uart(msg_ready);
    }
    /* start receiving UART via interrupt */
    HAL_UART_Receive_IT(&huart2, &byte_buffer, sizeof(byte_buffer));


    while (1)
    {
        if (flag_uart_received)
        {
            /* echoing back to terminal */
            HAL_UART_Transmit(&huart2, &byte_buffer, sizeof(byte_buffer), 100);

            /* save new byte to the buffer */
            text_buffer[text_index] = byte_buffer;
            text_index++;

            /* save time from last input */
            uint32_t timer_elapsed = HAL_GetTick() - timer_start;

            /* check for buffer overflows, time exceeded, line ending */
            if ((text_index == TEXT_BUF_SIZE-1) || (timer_elapsed > timer_delay))
            {
                if (timer_elapsed > timer_delay)
                {
                    flag_time_out = 1;
                }
                memset(text_buffer, 0, TEXT_BUF_SIZE); // clear buffer
                text_index = 0;
                line_received = LINE_ERR;
            }
            else if ((byte_buffer == '\n') || (byte_buffer == '\r'))
            {
                text_buffer[text_index] = '\0';    // put correct end of line
                text_index = 0;
                line_received = LINE_OK;
            }

            flag_uart_received = 0;
            timer_start = HAL_GetTick();

            /* Restart receiving UART to receive next byte */
            HAL_UART_Receive_IT(&huart2, &byte_buffer, sizeof(byte_buffer));
        }


        if (line_received == LINE_OK)
        {
            line_received = LINE_PROGRESS;
            if (flag_time_out)                     // print error message if time exceeded
            {
                flag_time_out = 0;
                print_to_uart(msg_time_out);
            }
            else {                                 // else parse the command
                /* parse command function */
                parse_buffer(text_buffer);
            }
        }
        else if (line_received == LINE_ERR)
	    {
            //some message here if need
            line_received = LINE_PROGRESS;
	    }
    }
}

/**
  * @brief  This function is executed if UART register ready.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        flag_uart_received=1;
    }
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
