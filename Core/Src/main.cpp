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
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lwip_udp_client.h"
#include "mbedtls_md5.h"
#include "sip_client.h"
#include "button_handler.h"


extern "C" {
	#include <stdio.h>
	#include "lwip.h"
}
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
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
UART_HandleTypeDef huart2;

void but_task(void * argument);
void lwip_task(void * argument);
void sip_task(void *pvParameters);
/* USER CODE BEGIN PFP */

osThreadId_t thread_sip, thread_button, thread_lwip;
osEventFlagsId_t event_eth;

constexpr uint8_t COMMAND_ETH_BIT = (1<<2);

osThreadAttr_t sip_attributes = {
  .name = "SipTask",
  .stack_size = 4096,
  .priority = osPriorityNormal
};

osThreadAttr_t but_attributes = {
  .name = "ButTask",
  .stack_size = 1024,
  .priority = osPriorityNormal
};

osThreadAttr_t lwip_attributes = {
  .name = "LwipTask",
  .stack_size = 1024,
  .priority = osPriorityNormal
};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SIP_SERVER_IP   "192.168.1.117"
#define SIP_SERVER_PORT "5160"
#define SIP_USER		"103"
#define LOCAL_IP		"192.168.1.125"
#define SIP_PASSWORD	"103"
#define RING_DURATION_TIMEOUT_MSEC 7000

using SipClientT = SipClient<LwipUdpClient, MbedtlsMd5>;
SipClientT __attribute__((section(".ccmram"))) client{SIP_USER, SIP_PASSWORD, SIP_SERVER_IP, SIP_SERVER_PORT, LOCAL_IP};

extern struct netif gnetif;

ButtonInputHandler<SipClientT, RING_DURATION_TIMEOUT_MSEC> button_input_handler(client);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_0) {
	  Event event = Event::BUTTON_PRESS;
	  osMessageQueuePut(button_input_handler.m_queue, &event, osPriorityISR, 0);
  }
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

extern "C" {
	PUTCHAR_PROTOTYPE
	{
	  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	  return ch;
	}
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
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  osKernelInitialize();

  /* Call PreOsInit function */
  /* USER CODE BEGIN 2 */
  event_eth = osEventFlagsNew(NULL);

  thread_sip = 		osThreadNew(sip_task, NULL, &sip_attributes);
  thread_button = 	osThreadNew(but_task, NULL, &but_attributes);
  thread_lwip = 	osThreadNew(lwip_task, NULL, &lwip_attributes);
  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void sip_task(void *pvParameters)
{
    for(;;)
    {
    	osEventFlagsWait(event_eth, COMMAND_ETH_BIT, osFlagsNoClear, osWaitForever);

    	if (!client.is_initialized())
        {
            bool result = client.init();
            printf("SIP client initialized %ssuccessfully \n", result ? "" : "un");
            if (!result)
            {
                printf("Waiting to try again...");
                osDelay(2000);
                continue;
            }

            client.set_event_handler([](const SipClientEvent& event) {

               switch (event.event)
               {
               case SipClientEvent::Event::CALL_START:
                   printf( "Call start");
                   break;
               case SipClientEvent::Event::CALL_CANCELLED:
                   printf("Call cancelled, reason %d", (int) event.cancel_reason);
                   button_input_handler.call_end();
                   break;
               case SipClientEvent::Event::CALL_END:
                   printf("Call end");
                   button_input_handler.call_end();
                   osDelay(500);
                   //i2s_pause();
                   break;
               case SipClientEvent::Event::BUTTON_PRESS:
                   printf("Got button press: %c for %d milliseconds", event.button_signal, event.button_duration);
                   break;
               }
            });
        }

        client.run();
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

void lwip_task(void * argument)
{

	MX_LWIP_Init();
	client.set_server_ip(std::string(SIP_SERVER_IP));
	client.set_my_ip(std::string(ip4addr_ntoa(netif_ip4_addr(&gnetif))));
	osEventFlagsSet(event_eth,COMMAND_ETH_BIT);

	for(;;)
	{
			  BSP_LED_Toggle(LED3);
			  BSP_LED_Toggle(LED4);
			  BSP_LED_Toggle(LED5);
			  BSP_LED_Toggle(LED6);
			  osDelay(1000);
	}
}

void but_task(void * argument)
{
	  button_input_handler.run();
}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
