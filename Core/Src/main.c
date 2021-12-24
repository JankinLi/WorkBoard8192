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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

LPTIM_HandleTypeDef hlptim1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

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
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC2_Init(void);
void StartMainRecvTask(void *argument);
void StartWorkTask(void *argument);
void StartOutTask(void *argument);
void StartErrorTask(void *argument);
void StartDownBoardTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// idle mode
int8_t g_idle_mode = -2; //-2 is wait down board, -1 is wait 0x01-0x01 telegram, 0 is work mode , 1 is sleep mode
uint32_t g_wait_down_board_start =0;
uint8_t g_down_borad_init_data_ptr[8];

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

//down receive buffer
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
		if (g_idle_mode == -2){ //when first communication with down board is finish, change mode.
			g_idle_mode = -1;
			set_down_head_it();
			//PutErrorCode(ErrorQueueHandle,0xE8);
			return;
		}

		if (g_idle_mode == -1){ //when mode is -1, ignore data from down board.
			set_down_head_it();
			//PutErrorCode(ErrorQueueHandle,0xE8);
			return;
		}

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

//health COM Device
unsigned char bt_head_flag = 0;
char bt_head_buf[2];

void receive_bt_head(){
	//TODO
	return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1){ //Main COM
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

	if (huart->Instance == UART4 ){ //DOWN Control Board COM
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

	if (huart->Instance == USART3){ //BT COM
		if (1 == bt_head_flag){
			receive_bt_head();
			return;
		}
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
	if (value >= 0){
		char *p_len = buffer + 4;
		int len = 4;
		memcpy(p_len, &len, 4);
	}
	memcpy(p, buffer, 8);

	if (value >= 0){
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

	char buffer[8] = {0x4D, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	buffer[2] = type1_value;
	buffer[3] = type2_value;

	char *p = main_out_buffer[main_out_write_index];
	if (value >= 0){
		char *p_len = buffer + 4;
		int len = 1;
		memcpy(p_len, &len, 1);
	}
	memcpy(p, buffer, 8);

	if (value >= 0){
		char *p_data = p + 8;
		memcpy(p_data, &value, 1);
	}

	uint8_t pos = main_out_write_index;
	osMessageQueuePut(MainOutQueueHandle, &pos, 0U, 0U);

	move_main_out_next_write_index();

	osMutexRelease(OutMutexHandle);
}

void put_no_data_into_out_buffer(uint8_t type1_value, uint8_t type2_value){
	osMutexAcquire(OutMutexHandle, osWaitForever);

	char buffer[8] = {0x4D, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	buffer[2] = type1_value;
	buffer[3] = type2_value;

	char *p = main_out_buffer[main_out_write_index];
	memcpy(p, buffer, 8);

	uint8_t pos = main_out_write_index;
	osMessageQueuePut(MainOutQueueHandle, &pos, 0U, 0U);

	move_main_out_next_write_index();

	osMutexRelease(OutMutexHandle);
}

void put_four_bytes_into_out_buffer(uint8_t type1_value, uint8_t type2_value, uint8_t byte_1, uint8_t byte_2, uint8_t byte_3, uint8_t byte_4){
	osMutexAcquire(OutMutexHandle, osWaitForever);

	char buffer[8] = {0x4D, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	buffer[2] = type1_value;
	buffer[3] = type2_value;

	char *p = main_out_buffer[main_out_write_index];

	char *p_len = buffer + 4;
	int len = 4;
	memcpy(p_len, &len, 1);

	memcpy(p, buffer, 8);

	char *p_data = p + 8;
	memcpy(p_data, &byte_1, 1);
	memcpy(p_data+1, &byte_2, 1);
	memcpy(p_data+2, &byte_3, 1);
	memcpy(p_data+3, &byte_4, 1);

	uint8_t pos = main_out_write_index;
	osMessageQueuePut(MainOutQueueHandle, &pos, 0U, 0U);

	move_main_out_next_write_index();

	osMutexRelease(OutMutexHandle);
}

void put_five_bytes_into_out_buffer(uint8_t type1_value, uint8_t type2_value, uint8_t byte_1, uint8_t byte_2, uint8_t byte_3, uint8_t byte_4, uint8_t byte_5){
	osMutexAcquire(OutMutexHandle, osWaitForever);

	char buffer[8] = {0x4D, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	buffer[2] = type1_value;
	buffer[3] = type2_value;

	char *p = main_out_buffer[main_out_write_index];

	char *p_len = buffer + 4;
	int len = 5;
	memcpy(p_len, &len, 1);

	memcpy(p, buffer, 8);

	char *p_data = p + 8;
	memcpy(p_data, &byte_1, 1);
	memcpy(p_data+1, &byte_2, 1);
	memcpy(p_data+2, &byte_3, 1);
	memcpy(p_data+3, &byte_4, 1);
	memcpy(p_data+4, &byte_5, 1);

	uint8_t pos = main_out_write_index;
	osMessageQueuePut(MainOutQueueHandle, &pos, 0U, 0U);

	move_main_out_next_write_index();

	osMutexRelease(OutMutexHandle);
}

void fill_down_board_init_telegram(){
	memset(g_down_borad_init_data_ptr, 0x00, 8);
	uint8_t *p = g_down_borad_init_data_ptr;
	p[0] = 0xf7;
	p[1] = 0xf8;
	p[2] = 0x04;
	p[3] = 0x01;
	p[4] = 0x01;
	p[5] = 0x01;
	p[6] = 0x07;
	p[7] = 0xfd;
}

// power button flag
uint8_t g_power_btn = 0;
uint32_t g_power_start = 0;

//vibration motor
uint8_t g_motor_1 = 0;
uint32_t g_motor_1_start = 0;
uint8_t g_motor_2 = 0;
uint32_t g_motor_2_start = 0;

//key
uint8_t g_key_1_value = 0x02;
uint8_t g_key_2_value = 0x02;

//Fan
//uint8_t g_fan_1 = 0;
//uint32_t g_fan_1_start = 0;

uint8_t g_fan_1_value = 0x00;
uint8_t g_fan_2_value = 0x00;
uint8_t g_fan_3_value = 0x00;
uint8_t g_fan_4_value = 0x00;


// ADC
uint16_t ADC_VALUE = 0;  //value of strength
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1){  //ANGLE_DET //PB0
		// Read & Update The ADC Result
		ADC_VALUE = HAL_ADC_GetValue(&hadc1);

		put_int_into_out_buffer(0x04, 0x01, ADC_VALUE);
	}
	else if (hadc->Instance == ADC2){ //TORQE //PA0
		ADC_VALUE = HAL_ADC_GetValue(&hadc2);

		put_int_into_out_buffer(0x01, 0x04, ADC_VALUE);
	}
}


//EXTI Callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13){  //Human radar 1	//PC13
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
			put_byte_into_out_buffer(0x03, 0x01, 0x01);
			//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);            //清除引脚中断
		}
		else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
			put_byte_into_out_buffer(0x03, 0x01, 0x02);
		}
	}
	else if (GPIO_Pin == GPIO_PIN_1){ //Human radar 2  //PA1
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET){
			put_byte_into_out_buffer(0x03, 0x02, 0x01);
		}
		else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET){
			put_byte_into_out_buffer(0x03, 0x02, 0x02);
		}
	}
	else if(GPIO_Pin == GPIO_PIN_0){  //HR1 //PC0  //霍尔检测1
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET){
			put_byte_into_out_buffer(0x02, 0x01, 0x01);
		}
		else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET){
			put_byte_into_out_buffer(0x02, 0x01, 0x02);
		}
	}
	else if(GPIO_Pin == GPIO_PIN_4){  //KEY1_DET //PA4
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
			g_key_1_value = 0x01;
			put_byte_into_out_buffer(0x05, 0x01, 0x01);
		}
		else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET){
			g_key_1_value = 0x02;
			put_byte_into_out_buffer(0x05, 0x01, 0x02);
		}
	}
	else if(GPIO_Pin == GPIO_PIN_2){  //KEY2_DET //PB2
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET){
			g_key_2_value = 0x01;
			put_byte_into_out_buffer(0x05, 0x02, 0x01);
		}
		else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET){
			g_key_2_value = 0x02;
			put_byte_into_out_buffer(0x05, 0x02, 0x02);
		}
	}
	else if(GPIO_Pin == GPIO_PIN_14){  //PWR_DOWN //PB14
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET){
			if (g_idle_mode == 1){
				g_power_btn = 1;
				g_power_start = osKernelGetSysTimerCount();
			}
			else if (g_idle_mode == 0){
				g_power_btn = 1;
				g_power_start = osKernelGetSysTimerCount();
			}
		}
		else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET){
			if (g_idle_mode == 1){
				if (g_power_btn == 1){
					g_power_btn = 0;
					uint32_t now_t = osKernelGetSysTimerCount();
					uint32_t diff = now_t - g_power_start;
					uint32_t freq = osKernelGetSysTimerFreq();
					uint32_t timeout_value_1_second = 1 * g_fan_1_value * freq;
					uint32_t timeout_value_3_second = 3 * g_fan_1_value * freq;
					uint32_t timeout_value_10_second = 10 * g_fan_1_value * freq;
					if (diff < timeout_value_1_second){
						put_no_data_into_out_buffer(0x011, 0x05);
						g_idle_mode = 0;
					}
					else if (diff > timeout_value_3_second && diff < timeout_value_10_second){
						put_no_data_into_out_buffer(0x011, 0x01);
					}
					else if( diff >= timeout_value_10_second){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
					}
				}
			}
			else if(g_idle_mode == 0){
				if (g_power_btn == 1){
					g_power_btn = 0;
					uint32_t now_t = osKernelGetSysTimerCount();
					uint32_t diff = now_t - g_power_start;
					uint32_t freq = osKernelGetSysTimerFreq();
					uint32_t timeout_value_3_second = 3 * g_fan_1_value * freq;
					uint32_t timeout_value_10_second = 10 * g_fan_1_value * freq;
					if (diff > timeout_value_3_second && diff < timeout_value_10_second){
						put_no_data_into_out_buffer(0x011, 0x01);
					}
					else if( diff >= timeout_value_10_second){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
					}
				}
			}
		}
	}
}

// LOGO lamp
#define LOGO_COUNT 23
uint8_t g_logo_lamp_flag = 0;
uint8_t g_logo_lamp_value_on = 0;
uint32_t g_logo_lamp_color_value;
uint8_t g_logo_lamp_value[LOGO_COUNT][3];
uint32_t g_logo_lamp_start = 0;
uint8_t g_logo_lamp_bit_index = 0;
uint8_t g_logo_lamp_pos = 0;
uint8_t g_logo_lamp_index = 0;

// Energy lamp
#define ENERGY_COUNT 21
uint8_t g_energy_lamp_flag = 0;
uint8_t g_energy_lamp_value_on = 0;
uint8_t g_energy_lamp_effect = 0;
uint32_t g_energy_lamp_color_value;
uint8_t g_energy_lamp_value[ENERGY_COUNT][3];
uint32_t g_energy_lamp_start = 0;
uint8_t g_energy_lamp_bit_index = 0;
uint8_t g_energy_lamp_pos = 0;
uint8_t g_energy_lamp_index = 0;

// side lamp 1
uint8_t g_side_1_lamp_flag = 0;
uint8_t g_side_1_lamp_value[168][3];
uint32_t g_side_1_lamp_start = 0;

// side lamp 2
uint8_t g_side_2_lamp_flag = 0;
uint8_t g_side_2_lamp_value[168][3];
uint32_t g_side_2_lamp_start = 0;

uint8_t compute_logo_final_value(){
	uint8_t value = g_logo_lamp_value[g_logo_lamp_index][g_logo_lamp_pos];
	uint8_t bit_value = ( value & ( 0x01 << g_logo_lamp_bit_index ) ) >> g_logo_lamp_bit_index;
	uint8_t final_val = 1;
	if (bit_value == 1){
		final_val = 3;
	}
	return final_val;
}

void start_logo_lamp_pwm(){
	g_logo_lamp_flag = 1;
	g_logo_lamp_start = osKernelGetSysTimerCount();
	g_logo_lamp_pos = 0;
	g_logo_lamp_index = 0;
	g_logo_lamp_bit_index = 7;

	uint8_t final_value = compute_logo_final_value();
	TIM1->CCR2 = (htim1.Init.Period * final_value) / 4u;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  //TIM1_CH2
}

void fill_logo_lamp_color(){
	uint32_t color_value = g_logo_lamp_color_value;

	uint8_t red_value = (color_value & 0xFF0000) >> 16;
	uint8_t green_value = (color_value & 0x00FF00) >> 8;
	uint8_t blue_value = (color_value & 0x0000FF);

	uint8_t i=0;
	for(i=0; i< LOGO_COUNT; i++){
		g_logo_lamp_value[i][0] = red_value;
		g_logo_lamp_value[i][1] = green_value;
		g_logo_lamp_value[i][2] = blue_value;
	}
	start_logo_lamp_pwm();
}

void fill_logo_lamp_blank(){
	uint8_t i=0;
	for(i=0; i< LOGO_COUNT; i++){
		g_logo_lamp_value[i][0] = 0x00;
		g_logo_lamp_value[i][1] = 0x00;
		g_logo_lamp_value[i][2] = 0x00;
	}
	start_logo_lamp_pwm();
}

uint8_t compute_energy_final_value(){
	uint8_t value = g_energy_lamp_value[g_energy_lamp_index][g_energy_lamp_pos];
	uint8_t bit_value = ( value & ( 0x01 << g_energy_lamp_bit_index ) ) >> g_energy_lamp_bit_index;
	uint8_t final_val = 1;
	if (bit_value == 1){
		final_val = 3;
	}
	return final_val;
}

void start_energy_lamp_pwm(){
	g_energy_lamp_flag = 1;
	g_energy_lamp_start = osKernelGetSysTimerCount();
	g_energy_lamp_pos = 0;
	g_energy_lamp_index = 0;
	g_energy_lamp_bit_index = 7;

	uint8_t final_value = compute_energy_final_value();
	TIM1->CCR3 = (htim1.Init.Period * final_value) / 4u;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  //TIM1_CH3
}

void fill_energy_lamp_color(){
	uint32_t color_value = g_energy_lamp_color_value;

	uint8_t red_value = (color_value & 0xFF0000) >> 16;
	uint8_t green_value = (color_value & 0x00FF00) >> 8;
	uint8_t blue_value = (color_value & 0x0000FF);

	uint8_t i=0;
	for(i=0; i< ENERGY_COUNT; i++){
		g_energy_lamp_value[i][0] = red_value;
		g_energy_lamp_value[i][1] = green_value;
		g_energy_lamp_value[i][2] = blue_value;
	}
	start_energy_lamp_pwm();
}

void fill_energy_lamp_blank(){
	uint8_t i=0;
	for(i=0; i< ENERGY_COUNT; i++){
		g_energy_lamp_value[i][0] = 0x00;
		g_energy_lamp_value[i][1] = 0x00;
		g_energy_lamp_value[i][2] = 0x00;
	}
	start_energy_lamp_pwm();
}

void fill_energy_lamp_count(uint8_t count){
	uint32_t color_value = g_energy_lamp_color_value;

	uint8_t red_value = (color_value & 0xFF0000) >> 16;
	uint8_t green_value = (color_value & 0x00FF00) >> 8;
	uint8_t blue_value = (color_value & 0x0000FF);

	uint8_t i=0;
	for(i=0; i< count; i++){
		g_energy_lamp_value[i][0] = red_value;
		g_energy_lamp_value[i][1] = green_value;
		g_energy_lamp_value[i][2] = blue_value;
	}
	for(i=count; i< ENERGY_COUNT; i++){
		g_energy_lamp_value[i][0] = 0x00;
		g_energy_lamp_value[i][1] = 0x00;
		g_energy_lamp_value[i][2] = 0x00;
	}
	start_energy_lamp_pwm();
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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_LPTIM1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USB_PCD_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);

  // Calibrate The ADC On Power-Up For Better Accuracy
  uint32_t SingleDiff = ADC_SINGLE_ENDED;
  HAL_ADCEx_Calibration_Start(&hadc1, SingleDiff);
  HAL_ADCEx_Calibration_Start(&hadc2, SingleDiff);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_LPTIM1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_0;
  hlptim1.Init.Trigger.ActiveEdge = LPTIM_ACTIVEEDGE_RISING;
  hlptim1.Init.Trigger.SampleTime = LPTIM_TRIGSAMPLETIME_DIRECTTRANSITION;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 18;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 120;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 120;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 120;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 20;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 18;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 4;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 18;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 4;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB4 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
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
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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

	g_logo_lamp_color_value = 0xCF0F0F;
	fill_logo_lamp_color();

	g_energy_lamp_color_value = 0xFF00FF;
	fill_energy_lamp_color();

	fill_down_board_init_telegram();
	g_wait_down_board_start = osKernelGetSysTimerCount();
	HAL_UART_Transmit(&huart4,(uint8_t*)g_down_borad_init_data_ptr,0x08,0xffff);

	/* Infinite loop */
	for(;;)
	{
		if (g_idle_mode == -2){
			uint32_t tick;
			tick = osKernelGetSysTimerCount();
			uint32_t diff = tick - g_wait_down_board_start;
			uint32_t freq = osKernelGetSysTimerFreq();
			uint32_t timeout_value = 10 * freq;
			if (diff >= timeout_value){
				g_wait_down_board_start = osKernelGetSysTimerCount();
				HAL_UART_Transmit(&huart4,(uint8_t*)g_down_borad_init_data_ptr,0x08,0xffff);
			}
		}

		if (g_motor_1 == 1){
			uint32_t tick;
			tick = osKernelGetSysTimerCount();
			uint32_t diff = tick - g_motor_1_start;
			uint32_t freq = osKernelGetSysTimerFreq();
			uint32_t timeout_value = 1 * freq;
			if (diff >= timeout_value){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
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
			if (diff >= timeout_value){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
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
			if (diff >= timeout_value){
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
			if (diff >= timeout_value){
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

		if (g_logo_lamp_flag == 1){
			uint32_t tick;
			tick = osKernelGetSysTimerCount();
			uint32_t diff = tick - g_logo_lamp_start;
			uint32_t freq = osKernelGetSysTimerFreq();
			uint32_t timeout_value = 0.0000012 * freq;
			if (diff >= timeout_value){
				if (g_logo_lamp_bit_index == 0){
					g_logo_lamp_bit_index = 7;
					g_logo_lamp_pos++;
					if (g_logo_lamp_pos>2){
						g_logo_lamp_pos = 0;
						g_logo_lamp_index ++;
						if (g_logo_lamp_index >= LOGO_COUNT){
							g_logo_lamp_flag = 0;
							HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
						}
					}
				}
				else{
					g_logo_lamp_bit_index--;
				}

				uint8_t final_value = compute_logo_final_value();
				TIM1->CCR2 = (htim1.Init.Period * final_value) / 4u;
			}
		}
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
						g_idle_mode = 0;  // work mode
						put_int_into_out_buffer(0x01, 0x02, 0x1024);

						// Start ADC Conversion
						HAL_ADC_Start_IT(&hadc2);
					}
					else if(data_ptr[1] == 0x06 ){
						HAL_ADC_Stop_IT(&hadc2);
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if(data_ptr[0] == 0x05){
					if(data_ptr[1] == 0x03 ){ //key_1
						put_byte_into_out_buffer(0x05, 0x05, g_key_1_value);
					}
					else if(data_ptr[1] == 0x04 ){ //key_2
						put_byte_into_out_buffer(0x05, 0x06, g_key_2_value);
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
									TIM3->CCR2 = (htim3.Init.Period * g_fan_1_value) / 20u;  // fan 1
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
									HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
								}
								else if(value_1 == 0x02){
									HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
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
									TIM8->CCR3 = (htim8.Init.Period * g_fan_2_value) / 20u;  // fan 2
									HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
									HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
								}
								else if(value_1 == 0x02){
									HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
									HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
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
									TIM2->CCR4 = (htim2.Init.Period * g_fan_3_value) / 20u;  // fan 3
									HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
									HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
								}
								else if(value_1 == 0x02){
									HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
									HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
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
									TIM8->CCR1 = (htim8.Init.Period * g_fan_4_value) / 20u;  //fan 4
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
									HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
								}
								else if(value_1 == 0x02){
									HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
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
						HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart4,(uint8_t*)data_ptr,len_value,0xffff);
						if (ret != HAL_OK){
							PutErrorCode(ErrorQueueHandle,0xE8);
						}
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if(data_ptr[0] == 0x11){
					if(data_ptr[1] == 0x02 ){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
					}
					else if(data_ptr[1] == 0x02 ){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
					}
					else if(data_ptr[1] == 0x04 ){
						g_idle_mode = 1; //sleep mode
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if (data_ptr[0] == 0x0b){
					if(data_ptr[1] == 0x01 ){
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 5){
							uint8_t value_on = data_ptr[0];
							g_energy_lamp_value_on = value_on;
							uint8_t value_red = data_ptr[1];
							uint8_t value_green = data_ptr[2];
							uint8_t value_blue = data_ptr[3];
							g_energy_lamp_effect = data_ptr[4];
							if (value_on == 0x01){
								uint32_t color_value = (value_red << 16) | (value_green << 8) | value_blue;
								g_logo_lamp_color_value = color_value;
								if (g_energy_lamp_effect == 0x01){
									fill_energy_lamp_color();
								}
								else if(g_energy_lamp_effect == 0x02){
									fill_energy_lamp_count(1);
								}
							}
							else if (value_on == 0x02){
								fill_energy_lamp_blank();
							}
							else{
								PutErrorCode(ErrorQueueHandle,0x08);
							}
						}
						else{
							PutErrorCode(ErrorQueueHandle,0x05);
						}
					}
					else if (data_ptr[1] == 0x02 ){
						uint32_t color_value = g_logo_lamp_color_value;

						uint8_t red_value = (color_value & 0xFF0000) >> 16;
						uint8_t green_value = (color_value & 0x00FF00) >> 8;
						uint8_t blue_value = (color_value & 0x0000FF);
						put_five_bytes_into_out_buffer(0x0C, 0x03, g_energy_lamp_value_on, red_value, green_value, blue_value, g_energy_lamp_effect);
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if (data_ptr[0] == 0x0c){
					if(data_ptr[1] == 0x01 ){
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 4){
							uint8_t value_on = data_ptr[0];
							g_logo_lamp_value_on = value_on;
							uint8_t value_red = data_ptr[1];
							uint8_t value_green = data_ptr[2];
							uint8_t value_blue = data_ptr[3];
							if (value_on == 0x01){
								uint32_t color_value = (value_red << 16) | (value_green << 8) | value_blue;
								g_logo_lamp_color_value = color_value;
								fill_logo_lamp_color();
							}
							else if (value_on == 0x02){
								fill_logo_lamp_blank();
							}
							else{
								PutErrorCode(ErrorQueueHandle,0x08);
							}
						}
						else{
							PutErrorCode(ErrorQueueHandle,0x05);
						}
					}
					else if (data_ptr[1] == 0x02 ){
						uint32_t color_value = g_logo_lamp_color_value;

						uint8_t red_value = (color_value & 0xFF0000) >> 16;
						uint8_t green_value = (color_value & 0x00FF00) >> 8;
						uint8_t blue_value = (color_value & 0x0000FF);
						put_four_bytes_into_out_buffer(0x0C, 0x03, g_logo_lamp_value_on, red_value, green_value, blue_value);
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if(data_ptr[0] == 0x0E){
					if(data_ptr[1] == 0x01 ){
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart3,(uint8_t*)data_ptr,len_value,0xffff);
						if (ret != HAL_OK){
							PutErrorCode(ErrorQueueHandle,0xEE);
						}
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
			if ( code_value > 0xE0){
				uint8_t value_temp = code_value & 0x0F;
				buffer[2] = value_temp;
				buffer[3] = 0xEE;
			}
			else{
				buffer[2] = 0x0E;
				buffer[3] = code_value;
			}
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
