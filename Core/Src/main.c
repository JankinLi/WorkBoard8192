/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tool.h"
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
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for MainRecvTask */
osThreadId_t MainRecvTaskHandle;
const osThreadAttr_t MainRecvTask_attributes = {
  .name = "MainRecvTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for WorkTask */
osThreadId_t WorkTaskHandle;
const osThreadAttr_t WorkTask_attributes = {
  .name = "WorkTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for OutTask */
osThreadId_t OutTaskHandle;
const osThreadAttr_t OutTask_attributes = {
  .name = "OutTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for ErrorTask */
osThreadId_t ErrorTaskHandle;
const osThreadAttr_t ErrorTask_attributes = {
  .name = "ErrorTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for DownBoardTask */
osThreadId_t DownBoardTaskHandle;
const osThreadAttr_t DownBoardTask_attributes = {
  .name = "DownBoardTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for MainRecvQueue */
osMessageQueueId_t MainRecvQueueHandle;
const osMessageQueueAttr_t MainRecvQueue_attributes = {
  .name = "MainRecvQueue"
};
/* Definitions for MainOutQueue */
osMessageQueueId_t MainOutQueueHandle;
const osMessageQueueAttr_t MainOutQueue_attributes = {
  .name = "MainOutQueue"
};
/* Definitions for ErrorQueue */
osMessageQueueId_t ErrorQueueHandle;
const osMessageQueueAttr_t ErrorQueue_attributes = {
  .name = "ErrorQueue"
};
/* Definitions for DownBoardQueue */
osMessageQueueId_t DownBoardQueueHandle;
const osMessageQueueAttr_t DownBoardQueue_attributes = {
  .name = "DownBoardQueue"
};
/* Definitions for OutMutex */
osMutexId_t OutMutexHandle;
const osMutexAttr_t OutMutex_attributes = {
  .name = "OutMutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartMainRecvTask(void *argument);
void StartWorkTask(void *argument);
void StartOutTask(void *argument);
void StartErrorTask(void *argument);
void StartDownBoardTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// main COM port
unsigned char main_head_flag = 0;
char main_head_buf[8];

unsigned char main_body_flag = 0;
char main_body_buf[512];

//main receive buffer
#define MAIN_BUFFER_COUNT 4
#define MAIN_BUFFER_MAX 512
char main_data_buffer[MAIN_BUFFER_COUNT][MAIN_BUFFER_MAX];

unsigned char main_data_write_index=0;

void memset_main_buffer(){
	memset(main_head_buf, 0, sizeof(main_head_buf));
	memset(main_body_buf, 0, sizeof(main_body_buf));
	memset(main_data_buffer, 0, sizeof(main_data_buffer));
}

int compute_len(char * len_start_ptr){
	int *len_p = (int*)len_start_ptr;
	int value = *len_p;
	return value;
}

void set_main_head_it(){
	main_head_flag = 1;
	main_body_flag = 0;
	HAL_UART_Receive_IT(&huart1,(uint8_t*)main_head_buf,8);  //main communication
}

int analyze_main_head(){
	if (main_head_buf[0] != 0x4D){
		return -1;
	}

	if (main_head_buf[1] != 0x43){
		return -1;
	}

	char *len_ptr = main_head_buf + 4;
	int * len_p = (int*)len_ptr;
	int value = *len_p;
	return value;
}

void move_next_main_write_index(){
	main_data_write_index++;
	if (main_data_write_index == MAIN_BUFFER_COUNT){
		main_data_write_index = 0;
	}
}

void receive_main_head(){
	int data_len = analyze_main_head();
	if (data_len == -1){
		PutErrorCode(ErrorQueueHandle, 0x01);
		set_main_head_it();
	}
	else if (data_len == 0){
		uint32_t ret = osMessageQueueGetSpace(MainRecvQueueHandle);
		if (ret ==0){
			PutErrorCode(ErrorQueueHandle, 0x06);
			set_main_head_it();
			return;
		}
		char *p = main_data_buffer[main_data_write_index];
		char *source = main_head_buf + 2;
		memcpy(p, source, 2);
		uint8_t pos = main_data_write_index;
		osMessageQueuePut(MainRecvQueueHandle, &pos, 0U, 0U);
		move_next_main_write_index();

		set_main_head_it();
	}
	else{
		if (data_len > (MAIN_BUFFER_MAX - 8)){
			PutErrorCode(ErrorQueueHandle,0x05);
			set_main_head_it();
			return;
		}
		main_head_flag = 0;
		main_body_flag = 1;
		HAL_UART_Receive_IT(&huart1,(uint8_t*)main_body_buf, data_len);
	}
}

void receive_main_data_part(){
	uint32_t ret = osMessageQueueGetSpace(MainRecvQueueHandle);
	if (ret ==0){
		PutErrorCode(ErrorQueueHandle,0x06);
		set_main_head_it();
		return;
	}
	char *p = main_data_buffer[main_data_write_index];
	char *source = main_head_buf+2;
	memcpy(p, source, 6);

	char *p_data = p + 6;
	char *p_len = p + 2;
	int count = compute_len(p_len);
	memcpy(p_data, main_body_buf, count);

	uint8_t pos = main_data_write_index;
	osMessageQueuePut(MainRecvQueueHandle, &pos, 0U, 0U);
	move_next_main_write_index();

	set_main_head_it();
}


//COM Down Device
unsigned char down_head_flag = 0;
char down_head_buf[2];

unsigned char down_body_flag = 0;

#define DOWN_BUFFER_SIZE 128
char down_body_buf[DOWN_BUFFER_SIZE];
char *p_down_current = down_body_buf;

//main receive buffer
#define DOWN_BUFFER_COUNT 4
char down_data_buffer[DOWN_BUFFER_COUNT][DOWN_BUFFER_SIZE];

unsigned char down_data_write_index=0;

void memset_down_buffer(){
	memset(down_head_buf, 0, sizeof(down_head_buf));
	memset(down_body_buf, 0, sizeof(down_body_buf));
	memset(down_data_buffer, 0, sizeof(down_data_buffer));
}

int analyze_down_head(){
	if (down_head_buf[0] != 0xF7){
		return -1;
	}

	if (down_head_buf[1] != 0xF8){
		return -1;
	}
	return 0;
}

void set_down_head_it(){
	down_head_flag = 1;
	down_body_flag = 0;
	p_down_current = down_body_buf;
	HAL_UART_Receive_IT(&huart4,(uint8_t*)down_head_buf,2);
}

void move_next_down_write_index(){
	down_data_write_index++;
	if (down_data_write_index == DOWN_BUFFER_COUNT){
		down_data_write_index = 0;
	}
}

void receive_down_head(){
	int ret = analyze_down_head();
	if (ret == -1){
		PutErrorCode(ErrorQueueHandle, 0x01);
		set_down_head_it();
		return;
	}

	down_head_flag = 0;
	down_body_flag = 1;
	p_down_current = down_body_buf;
	HAL_UART_Receive_IT(&huart4,(uint8_t*)p_down_current, 1);
}

void receive_down_data_part(){
	uint8_t value = *p_down_current;
	if (value == 0xFD){
		uint32_t ret = osMessageQueueGetSpace(DownBoardQueueHandle);
		if (ret ==0){
			PutErrorCode(ErrorQueueHandle,0x06);
			set_down_head_it();
			return;
		}

		char *p = down_data_buffer[down_data_write_index];
		memcpy(p, down_head_buf, 2);
		char *down_data_p = p + 2;
		int size_len = p_down_current - down_body_buf + 1;
		memcpy(down_data_p, down_body_buf, size_len);

		uint8_t pos = down_data_write_index;
		osMessageQueuePut(DownBoardQueueHandle, &pos, 0U, 0U);
		move_next_down_write_index();

		set_down_head_it();
		return;
	}

	p_down_current = p_down_current + 1;
	HAL_UART_Receive_IT(&huart4,(uint8_t*)p_down_current, 1);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1){
		if(1 == main_body_flag){
			receive_main_data_part();
			return;
		}

		if (1 == main_head_flag){
			receive_main_head();
			return;
		}
		return;
	}

	if (huart->Instance == UART4 ){
		if(1 == down_body_flag){
			receive_down_data_part();
			return;
		}

		if (1 == down_head_flag){
			receive_down_head();
			return;
		}
		return;
	}
}

//output buffer
#define MAIN_OUT_BUFFER_COUNT 4
char main_out_buffer[MAIN_OUT_BUFFER_COUNT][512];

unsigned char main_out_write_index = 0;

void move_main_out_next_write_index(){
	main_out_write_index++;
	if (main_out_write_index == MAIN_OUT_BUFFER_COUNT){
		main_out_write_index = 0;
	}
}

void put_int_into_out_buffer(uint8_t type1_value, uint8_t type2_value, int value){
	osMutexAcquire(OutMutexHandle, osWaitForever);

	char buffer[8] = {0x4D, 0x43};
	buffer[2] = type1_value;
	buffer[3] = type2_value;

	char *p = main_out_buffer[main_out_write_index];
	if (value > 0){
		char *p_len = buffer + 4;
		int len = 4;
		memcpy(p_len, &len, 4);
	}
	memcpy(p, buffer, 8);

	if (value > 0){
		char *p_data = p + 8;
		memcpy(p_data, &value, 4);
	}

	uint8_t pos = main_out_write_index;
	osMessageQueuePut(MainOutQueueHandle, &pos, 0U, 0U);

	move_main_out_next_write_index();

	osMutexRelease(OutMutexHandle);
}

void put_byte_into_out_buffer(uint8_t type1_value, uint8_t type2_value, uint8_t value){
	osMutexAcquire(OutMutexHandle, osWaitForever);

		char buffer[8] = {0x4D, 0x43};
		buffer[2] = type1_value;
		buffer[3] = type2_value;

		char *p = main_out_buffer[main_out_write_index];
		if (value > 0){
			char *p_len = buffer + 4;
			int len = 1;
			memcpy(p_len, &len, 1);
		}
		memcpy(p, buffer, 8);

		if (value > 0){
			char *p_data = p + 8;
			memcpy(p_data, &value, 1);
		}

		uint8_t pos = main_out_write_index;
		osMessageQueuePut(MainOutQueueHandle, &pos, 0U, 0U);

		move_main_out_next_write_index();

		osMutexRelease(OutMutexHandle);
}


//vibration motor
uint8_t g_motor_1 = 0;
uint32_t g_motor_1_start = 0;
uint8_t g_motor_2 = 0;
uint32_t g_motor_2_start = 0;

//Fan
//uint8_t g_fan_1 = 0;
//uint32_t g_fan_1_start = 0;

uint8_t g_fan_1_value = 0x00;
uint8_t g_fan_2_value = 0x00;
uint8_t g_fan_3_value = 0x00;
uint8_t g_fan_4_value = 0x00;


// ADC
uint16_t ADC_VALUE = 0;  //value of strength
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    // Read & Update The ADC Result
//	ADC_VALUE = HAL_ADC_GetValue(&hadc1);
//
//	put_int_into_out_buffer(0x01, 0x04, ADC_VALUE);
//}


//Human radar
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13){
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
			put_byte_into_out_buffer(0x03, 0x01, 0x01);
			//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);            //清除引脚中断
		}
		else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
			put_byte_into_out_buffer(0x03, 0x01, 0x02);
		}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start_IT(&htim1);
  //HAL_TIM_Base_Start_IT(&htim3);
  //HAL_TIM_Base_Start_IT(&htim8);

  // Calibrate The ADC On Power-Up For Better Accuracy
  //uint32_t SingleDiff = 0;
  //HAL_ADCEx_Calibration_Start(&hadc1, SingleDiff);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of OutMutex */
  OutMutexHandle = osMutexNew(&OutMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MainRecvQueue */
  MainRecvQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &MainRecvQueue_attributes);

  /* creation of MainOutQueue */
  MainOutQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &MainOutQueue_attributes);

  /* creation of ErrorQueue */
  ErrorQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &ErrorQueue_attributes);

  /* creation of DownBoardQueue */
  DownBoardQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &DownBoardQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MainRecvTask */
  MainRecvTaskHandle = osThreadNew(StartMainRecvTask, NULL, &MainRecvTask_attributes);

  /* creation of WorkTask */
  WorkTaskHandle = osThreadNew(StartWorkTask, NULL, &WorkTask_attributes);

  /* creation of OutTask */
  OutTaskHandle = osThreadNew(StartOutTask, NULL, &OutTask_attributes);

  /* creation of ErrorTask */
  ErrorTaskHandle = osThreadNew(StartErrorTask, NULL, &ErrorTask_attributes);

  /* creation of DownBoardTask */
  DownBoardTaskHandle = osThreadNew(StartDownBoardTask, NULL, &DownBoardTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB4 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainRecvTask */
/**
  * @brief  Function implementing the MainRecvTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainRecvTask */
void StartMainRecvTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	memset_main_buffer();

	if (0 == main_head_flag){
		main_head_flag = 1;
		HAL_UART_Receive_IT(&huart1,(uint8_t*)main_head_buf,8);
	}

	memset_down_buffer();

	if (0 == down_head_flag){
		down_head_flag = 1;
		HAL_UART_Receive_IT(&huart4,(uint8_t*)down_head_buf,2);
	}

	/* Infinite loop */
	for(;;)
	{
		if (g_motor_1 == 1){
			uint32_t tick;
			tick = osKernelGetSysTimerCount();
			uint32_t diff = tick - g_motor_1_start;
			uint32_t freq = osKernelGetSysTimerFreq();
			uint32_t timeout_value = 1 * freq;
			if (diff > timeout_value){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
				//PutErrorCode(ErrorQueueHandle,8);
				g_motor_1 = 2;
				g_motor_1_start = osKernelGetSysTimerCount();
			}
		}
		else if(g_motor_1 == 2){
			uint32_t tick;
			tick = osKernelGetSysTimerCount();
			uint32_t diff = tick - g_motor_1_start;
			uint32_t freq = osKernelGetSysTimerFreq();
			uint32_t timeout_value = 1 * freq;
			if (diff > timeout_value){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
				//PutErrorCode(ErrorQueueHandle,9);
				g_motor_1 = 1;
				g_motor_1_start = osKernelGetSysTimerCount();
			}
		}
		if (g_motor_2 == 1){
			uint32_t tick;
			tick = osKernelGetSysTimerCount();
			uint32_t diff = tick - g_motor_2_start;
			uint32_t freq = osKernelGetSysTimerFreq();
			uint32_t timeout_value = 1 * freq;
			if (diff > timeout_value){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				g_motor_2 = 2;
				g_motor_2_start = osKernelGetSysTimerCount();
			}
		}
		else if(g_motor_2 == 2){
			uint32_t tick;
			tick = osKernelGetSysTimerCount();
			uint32_t diff = tick - g_motor_2_start;
			uint32_t freq = osKernelGetSysTimerFreq();
			uint32_t timeout_value = 1 * freq;
			if (diff > timeout_value){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
				g_motor_2 = 1;
				g_motor_2_start = osKernelGetSysTimerCount();
			}
		}

//		if (g_fan_1 == 1){
//			uint32_t tick;
//			tick = osKernelGetSysTimerCount();
//			uint32_t diff = tick - g_fan_1_start;
//			uint32_t freq = osKernelGetSysTimerFreq();
//			uint32_t timeout_value = 0.05 * g_fan_1_value * freq;
//			if (diff > timeout_value){
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//				g_fan_1 = 2;
//				g_fan_1_start = osKernelGetSysTimerCount();
//				//PutErrorCode(ErrorQueueHandle,8);
//			}
//		}
//		else if(g_fan_1 == 2){
//			uint32_t tick;
//			tick = osKernelGetSysTimerCount();
//			uint32_t diff = tick - g_fan_1_start;
//			uint32_t freq = osKernelGetSysTimerFreq();
//			uint32_t timeout_value = 0.05 *(0x14 - g_fan_1_value) * freq;
//			if (diff > timeout_value){
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
//				g_fan_1 = 1;
//				g_fan_1_start = osKernelGetSysTimerCount();
//			}
//		}


		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartWorkTask */
/**
* @brief Function implementing the WorkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWorkTask */
void StartWorkTask(void *argument)
{
  /* USER CODE BEGIN StartWorkTask */
  /* Infinite loop */
	for(;;)
	{
		uint8_t pos;
		osStatus_t status = osMessageQueueGet(MainRecvQueueHandle, &pos, NULL, 0U);
		if (status == osOK) {
			if (pos < 0 || pos >= MAIN_BUFFER_COUNT){
				PutErrorCode(ErrorQueueHandle, 0x07);
			}
			else{
				char* data_ptr = main_data_buffer[pos];
				if (data_ptr[0] == 0x01){
					if(data_ptr[1] == 0x01 ){
						put_int_into_out_buffer(0x01, 0x02, 0x1024);

						// Start ADC Conversion
						//HAL_ADC_Start_IT(&hadc1);
					}
					else if(data_ptr[1] == 0x06 ){
						//HAL_ADC_Stop_IT(&hadc1);
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if(data_ptr[0] == 0x06){
					if(data_ptr[1] == 0x01 ){ //motor_1
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 1){
							uint8_t control_value = *data_ptr;
							if (control_value == 0x01){
								g_motor_1 = 0;
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
							}
							else if (control_value == 0x02){
								g_motor_1 = 1;
								g_motor_1_start = osKernelGetSysTimerCount();
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
							}
							else if (control_value == 0x03){
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
								g_motor_1 = 0;
							}
						}
						else{
							PutErrorCode(ErrorQueueHandle,0x05);
						}
					}
					else if(data_ptr[1] == 0x02 ){ //motor_2
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 1){
							uint8_t control_value = *data_ptr;
							if (control_value == 0x01){
								g_motor_2 = 0;
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
							}
							else if (control_value == 0x02){
								g_motor_2 = 1;
								g_motor_2_start = osKernelGetSysTimerCount();
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
							}
							else if (control_value == 0x03){
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
								g_motor_2 = 0;
							}
						}
						else{
							PutErrorCode(ErrorQueueHandle,0x05);
						}
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if(data_ptr[0] == 0x07){
					if(data_ptr[1] == 0x01 ){ //fan 1
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 2){
							uint8_t value_1 = data_ptr[0];
							uint8_t value_2 = data_ptr[1];
							if ( (value_1 == 0x01) && (value_2 < 0x01 || value_2 > 0x14)){
								PutErrorCode(ErrorQueueHandle,0x08);
							}
							else{
								if (value_1 == 0x01){
									g_fan_1_value = value_2;
									//TIM8->CCR3 = (htim8.Init.Period * g_fan_1_value) / 20u;  // fan 1
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
									//HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
								}
								else if(value_1 == 0x02){
									//HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
								}
							}
						}
						else{
							PutErrorCode(ErrorQueueHandle,0x05);
						}
					}
					else if(data_ptr[1] == 0x02 ){ //fan 2
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 2){
							uint8_t value_1 = data_ptr[0];
							uint8_t value_2 = data_ptr[1];
							if ( (value_1 == 0x01) && (value_2 < 0x01 || value_2 > 0x14)){
								PutErrorCode(ErrorQueueHandle,0x08);
							}
							else{
								if (value_1 == 0x01){
									g_fan_2_value = value_2;
									//TIM3->CCR3 = (htim3.Init.Period * g_fan_2_value) / 20u;  // fan 2
									HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
									//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
								}
								else if(value_1 == 0x02){
									//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
									HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
								}
							}
						}
						else{
							PutErrorCode(ErrorQueueHandle,0x05);
						}
					}
					else if(data_ptr[1] == 0x03 ){  //fan 3
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 2){
							uint8_t value_1 = data_ptr[0];
							uint8_t value_2 = data_ptr[1];
							if ( (value_1 == 0x01) && (value_2 < 0x01 || value_2 > 0x14)){
								PutErrorCode(ErrorQueueHandle,0x08);
							}
							else{
								if (value_1 == 0x01){
									g_fan_3_value = value_2;
									//TIM1->CCR3 = (htim1.Init.Period * g_fan_3_value) / 20u;  // fan 3
									HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
									//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
								}
								else if(value_1 == 0x02){
									//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
									HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
								}
							}
						}
						else{
							PutErrorCode(ErrorQueueHandle,0x05);
						}
					}
					else if(data_ptr[1] == 0x04 ){ //fan 4
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 2){
							uint8_t value_1 = data_ptr[0];
							uint8_t value_2 = data_ptr[1];
							if ( (value_1 == 0x01) && (value_2 < 0x01 || value_2 > 0x14)){
								PutErrorCode(ErrorQueueHandle,0x08);
							}
							else{
								if (value_1 == 0x01){
									g_fan_4_value = value_2;
									//TIM8->CCR1 = (htim8.Init.Period * g_fan_4_value) / 20u;  //fan 4
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
									//HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
								}
								else if(value_1 == 0x02){
									//HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
								}
							}
						}
						else{
							PutErrorCode(ErrorQueueHandle,0x05);
						}
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if(data_ptr[0] == 0x08){
					if(data_ptr[1] == 0x01 ){
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						HAL_UART_Transmit(&huart4,(uint8_t*)data_ptr,len_value,0xffff);
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else{
					PutErrorCode(ErrorQueueHandle,0x02);
				}
			}
		}
		osDelay(1);
	}
  /* USER CODE END StartWorkTask */
}

/* USER CODE BEGIN Header_StartOutTask */
/**
* @brief Function implementing the OutTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOutTask */
void StartOutTask(void *argument)
{
  /* USER CODE BEGIN StartOutTask */
  /* Infinite loop */
	for(;;)
	{
		uint8_t pos;
		osStatus_t status = osMessageQueueGet(MainOutQueueHandle, &pos, NULL, 0U);
		if (status == osOK) {
			if (pos == -1){
				PutErrorCode(ErrorQueueHandle, 0x07);
			}
			else{
				char* out_data_ptr = main_out_buffer[pos];
				char *p_len = out_data_ptr + 4;
				int dat_len = compute_len(p_len);
				unsigned int total = dat_len + 8;
				HAL_UART_Transmit(&huart1,(uint8_t*)out_data_ptr,total,0xffff);
			}
		}
		osDelay(1);
	}
  /* USER CODE END StartOutTask */
}

/* USER CODE BEGIN Header_StartErrorTask */
/**
* @brief Function implementing the ErrorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartErrorTask */
void StartErrorTask(void *argument)
{
  /* USER CODE BEGIN StartErrorTask */
  /* Infinite loop */
	for(;;)
	{
		uint8_t code_value;
		osStatus_t status = osMessageQueueGet(ErrorQueueHandle, &code_value, NULL, 0U);
		if (status == osOK) {
			unsigned char buffer[8] = {0x4D, 0x43};
			buffer[2] = 0x0E;
			buffer[3] = code_value;
			HAL_UART_Transmit(&huart1,(uint8_t*)buffer,8,0xffff);
		}
		osDelay(1);
	}
  /* USER CODE END StartErrorTask */
}

/* USER CODE BEGIN Header_StartDownBoardTask */
/**
* @brief Function implementing the DownBoardTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDownBoardTask */
void StartDownBoardTask(void *argument)
{
  /* USER CODE BEGIN StartDownBoardTask */
  /* Infinite loop */
	for(;;)
	{
		uint8_t pos;
		osStatus_t status = osMessageQueueGet(DownBoardQueueHandle, &pos, NULL, 0U);
		if (status == osOK) {
			if (pos < 0 || pos >= DOWN_BUFFER_COUNT){
				PutErrorCode(ErrorQueueHandle, 0x07);
			}
			else{
				char* down_data_ptr = down_data_buffer[pos];
				int data_len = compute_down_len(down_data_ptr);

				osMutexAcquire(OutMutexHandle, osWaitForever);

				char buffer[8] = {0x4D, 0x43};
				buffer[2] = 0x08;
				buffer[3] = 0x02;
				char *p = main_out_buffer[main_out_write_index];
				if (data_len > 0){
					char *p_len = buffer + 4;
					int len = data_len;
					memcpy(p_len, &len, 4);
				}
				memcpy(p, buffer, 8);

				if (data_len > 0){
					char *p_data = p + 8;
					char *down_ptr = down_data_ptr;
					memcpy(p_data, down_ptr, data_len);
				}

				uint8_t out_pos = main_out_write_index;
				osMessageQueuePut(MainOutQueueHandle, &out_pos, 0U, 0U);

				move_main_out_next_write_index();

				osMutexRelease(OutMutexHandle);
			}
		}
		osDelay(1);
	}
  /* USER CODE END StartDownBoardTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
