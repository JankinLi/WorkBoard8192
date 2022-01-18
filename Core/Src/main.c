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
#include "pwm_dma_led.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim1_ch3;
DMA_HandleTypeDef hdma_tim16_ch1;
DMA_HandleTypeDef hdma_tim17_ch1;

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
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for OutTask */
osThreadId_t OutTaskHandle;
const osThreadAttr_t OutTask_attributes = {
  .name = "OutTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ErrorTask */
osThreadId_t ErrorTaskHandle;
const osThreadAttr_t ErrorTask_attributes = {
  .name = "ErrorTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for DownBoardTask */
osThreadId_t DownBoardTaskHandle;
const osThreadAttr_t DownBoardTask_attributes = {
  .name = "DownBoardTask",
  .priority = (osPriority_t) osPriorityNormal,
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
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USB_PCD_Init(void);
void StartMainRecvTask(void *argument);
void StartWorkTask(void *argument);
void StartOutTask(void *argument);
void StartErrorTask(void *argument);
void StartDownBoardTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t g_hall_detect_value = 0;
uint8_t g_hall_detect_report_flag = 0x00;

uint8_t g_step_flag = 0;  //flag for 0x01

uint8_t g_capture_order = 0;
uint32_t g_step_count = 0;
uint32_t g_step_start = 0;  		//time-stamp

uint32_t g_step_value = 0;			//value of circle
uint32_t g_old_step_value = 0;		//temporary value of circle

uint32_t g_strength_value = 0;  	//value of strength
uint32_t g_old_strength_value = 0;	//temporary value of strength

uint8_t g_strength_adc_flag = 0;
uint32_t g_strength_adc_start = 0;
uint8_t g_strength_adc_calibration_flag = 0;
uint8_t g_strength_adc_calibration_count = 0;
uint32_t g_strength_adc_calibration_value = 0;
uint32_t g_strength_adc_calibration_total_value = 0;
const uint16_t g_strength_adc_calibration_default_value = 0x2C0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		if (g_capture_order == 1){
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4,TIM_INPUTCHANNELPOLARITY_FALLING);
			g_capture_order++;
		}
		else if (g_capture_order == 2){
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			g_step_count++;
			g_capture_order = 1;
		}
	}
}


// idle mode
int8_t g_idle_mode = -2; //-2 is wait down board, -1 is wait 0x01-0x01 telegram, 0 is work mode , 1 is sleep mode
uint32_t g_wait_down_board_start =0;
uint8_t g_down_borad_init_data_ptr[8];

uint8_t g_exit_idle_report_flag = 0x00;
uint8_t g_shudown_report_flag = 0x00;

void update_retry_down_board_communication(){
	uint32_t tick;
	tick = osKernelGetSysTimerCount();
	uint32_t diff = tick - g_wait_down_board_start;
	uint32_t freq = osKernelGetSysTimerFreq();
	uint32_t timeout_value = 5 * freq;	//wait 5 seconds for retry.
	if (diff >= timeout_value){	//retry to send initialize telegram into down board, wait it reply
		g_wait_down_board_start = osKernelGetSysTimerCount();
		HAL_UART_Transmit(&huart4,(uint8_t*)g_down_borad_init_data_ptr,0x08,0xffff);
	}
}

// main COM port
unsigned char main_head_flag = 0;
char main_head_buf[8];

unsigned char main_body_flag = 0;
char main_body_buf[256];

//main receive buffer
#define MAIN_BUFFER_COUNT 4
#define MAIN_BUFFER_MAX 256
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

//FTMS COM Device
uint8_t ftms_head_flag=0;
//TODO

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

	if (huart->Instance == USART3){ //FTMS COM
		if (1 == ftms_head_flag){
			return;
		}
	}
}

//output buffer
#define MAIN_OUT_BUFFER_COUNT 4
char main_out_buffer[MAIN_OUT_BUFFER_COUNT][256];

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

void put_two_int_and_one_byte_into_out_buffer(uint8_t type1_value, uint8_t type2_value, uint32_t val_1, uint32_t val_2, uint8_t val_3){
	osMutexAcquire(OutMutexHandle, osWaitForever);

	char buffer[8] = {0x4D, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	buffer[2] = type1_value;
	buffer[3] = type2_value;

	char *p = main_out_buffer[main_out_write_index];

	char *p_len = buffer + 4;
	int len = 9;
	memcpy(p_len, &len, 1);

	memcpy(p, buffer, 8);

	char *p_data = p + 8;

	memcpy(p_data, &val_1, 4);

	memcpy(p_data + 4, &val_2, 4);

	memcpy(p_data + 8, &val_3, 1);

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

void update_motor_1_action(){
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
}

void update_motor_2_action(){
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
}


//key
uint8_t g_key_1_value = 0x02;
uint8_t g_key_1_report_flag = 0x00;

uint8_t g_key_2_value = 0x02;
uint8_t g_key_2_report_flag = 0x00;

//Fan
uint8_t g_fan_1_value = 0x00;
uint8_t g_fan_2_value = 0x00;
uint8_t g_fan_3_value = 0x00;
uint8_t g_fan_4_value = 0x00;

// Angle detect.
uint8_t g_angle_report_flag = 0x00;
uint32_t g_angle_detect_value = 0x00;
uint8_t g_angle_adc_flag = 0x00;
uint32_t g_angle_adc_start = 0;
uint32_t g_angle_detect_flag = 0;

// ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1){  //ANGLE_DET //PB0
		// Read & Update The ADC Result
		HAL_ADC_Stop_IT(&hadc1);
		uint32_t ADC_VALUE = HAL_ADC_GetValue(&hadc1);

		//uint32_t Value_1=(uint32_t)((ADC_VALUE*3.3/4096)*1000);
		g_angle_detect_value = ADC_VALUE;
		g_angle_report_flag = 0x01;

		g_angle_adc_flag = 0x00;

		if (g_angle_detect_flag == 1){
			g_angle_adc_start = osKernelGetSysTimerCount();
			g_angle_adc_flag = 0x01;
		}
	}
	else if (hadc->Instance == ADC2){ //TORQE //PA0
		HAL_ADC_Stop_IT(&hadc2);
		uint32_t ADC_VALUE = HAL_ADC_GetValue(&hadc2);

		if (g_strength_adc_calibration_flag == 1){
			g_strength_adc_calibration_total_value += ADC_VALUE;
//			if (ADC_VALUE > g_strength_adc_calibration_value){
//				g_strength_adc_calibration_value = ADC_VALUE;
//			}
		}
		else{
			if (g_strength_adc_calibration_value > 0){
				if (ADC_VALUE > g_strength_adc_calibration_value){
					g_strength_value = ADC_VALUE;
				}
				else{
					g_strength_value = 0;
				}
			}
			else{
				if (ADC_VALUE > g_strength_adc_calibration_default_value){
					g_strength_value = ADC_VALUE;
				}
				else{
					g_strength_value = 0;
				}
			}
		}
		g_strength_adc_start = osKernelGetSysTimerCount();
		g_strength_adc_flag = 1;
	}
}

void update_strength_adc(){
	uint32_t tick;
	tick = osKernelGetSysTimerCount();
	uint32_t diff = tick - g_strength_adc_start;
	uint32_t freq = osKernelGetSysTimerFreq();
	uint32_t timeout_value = 0.05 * freq;
	if (diff >= timeout_value){
		g_strength_adc_flag = 0;
		HAL_ADC_Start_IT(&hadc2);
	}
}

void update_strength_adc_calibration(){
	uint32_t tick;
	tick = osKernelGetSysTimerCount();
	uint32_t diff = tick - g_strength_adc_start;
	uint32_t freq = osKernelGetSysTimerFreq();
	uint32_t timeout_value = 0.05 * freq;
	if (diff >= timeout_value){
		g_strength_adc_flag = 0;
		g_strength_adc_calibration_count++;
		if (g_strength_adc_calibration_count>=20){
			g_strength_adc_calibration_flag = 0;
			g_strength_adc_calibration_value = (int)(g_strength_adc_calibration_total_value / 20);
			if (g_strength_adc_calibration_value > 0){
				put_int_into_out_buffer(0x01, 0x02, g_strength_adc_calibration_value);
			}
			else{
				put_int_into_out_buffer(0x01, 0x02, g_strength_adc_calibration_default_value);
			}
			return;
		}
		HAL_ADC_Start_IT(&hadc2);
	}
}

void update_angle_adc(){
	uint32_t tick;
	tick = osKernelGetSysTimerCount();
	uint32_t diff = tick - g_angle_adc_start;
	uint32_t freq = osKernelGetSysTimerFreq();
	uint32_t timeout_value = 0.2 * freq;
	if (diff >= timeout_value){
		g_angle_adc_flag = 0x00;
		HAL_ADC_Start_IT(&hadc1);
	}
}

uint8_t g_human_rader_1_report_flag = 0;
uint8_t g_human_rader_1_report_value = 0;


uint8_t g_human_rader_2_report_flag = 0;
uint8_t g_human_rader_2_report_value = 0;

//EXTI Callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13){  //Human radar 1	//PC13
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
			g_human_rader_1_report_flag = 0x01;
			g_human_rader_1_report_value = 0x01;
			//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);            //清除引脚中断
		}
		else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
			g_human_rader_1_report_flag = 0x01;
			g_human_rader_1_report_value = 0x02;
			//put_byte_into_out_buffer(0x03, 0x01, 0x02);
		}
	}
	else if (GPIO_Pin == GPIO_PIN_1){ //Human radar 2  //PA1
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET){
			//put_byte_into_out_buffer(0x03, 0x02, 0x01);
			g_human_rader_2_report_flag = 0x01;
			g_human_rader_2_report_value = 0x01;
		}
		else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET){
			//put_byte_into_out_buffer(0x03, 0x02, 0x02);
			g_human_rader_2_report_flag = 0x01;
			g_human_rader_2_report_value = 0x02;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_0){  //HR1 //PC0 //Hall detector 1
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET){
			g_hall_detect_value = 0x01;
			g_hall_detect_report_flag = 0x01;
			//put_byte_into_out_buffer(0x02, 0x01, 0x01);
		}
		else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET){
			g_hall_detect_value = 0x02;
			g_hall_detect_report_flag = 0x01;
			//put_byte_into_out_buffer(0x02, 0x01, 0x02);
		}
	}
	else if(GPIO_Pin == GPIO_PIN_4){  //KEY1_DET //PA4
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
			g_key_1_value = 0x01;
			//put_byte_into_out_buffer(0x05, 0x01, 0x01);
			g_key_1_report_flag = 0x01;
		}
		else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET){
			g_key_1_value = 0x02;
			//put_byte_into_out_buffer(0x05, 0x01, 0x02);
		}
	}
	else if(GPIO_Pin == GPIO_PIN_2){  //KEY2_DET //PB2
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET){
			g_key_2_value = 0x01;
			g_key_2_report_flag = 0x01;
			//put_byte_into_out_buffer(0x05, 0x02, 0x01);
		}
		else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET){
			g_key_2_value = 0x02;
			//put_byte_into_out_buffer(0x05, 0x02, 0x02);
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
						//put_no_data_into_out_buffer(0x011, 0x05);
						g_exit_idle_report_flag = 0x01;
						g_idle_mode = 0;
					}
					else if (diff > timeout_value_3_second && diff < timeout_value_10_second){
						//put_no_data_into_out_buffer(0x011, 0x01);
						g_shudown_report_flag = 0x01;
					}
					else if( diff >= timeout_value_10_second){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
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
						//put_no_data_into_out_buffer(0x011, 0x01);
						g_shudown_report_flag = 0x01;
					}
					else if( diff >= timeout_value_10_second){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
					}
				}
			}
		}

		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
	}
}

//--------------------------------------------------------------------------------------
//All Lamp

uint8_t g_start_lamp_order = 0;

#define LAMP_NON_BLOCK_MODE 0
#define LAMP_BLOCK_MODE 1

void fill_lamp_buffer(uint8_t* p_colors, uint32_t color_value, uint8_t lamp_num, uint8_t total_lamp_num){
	uint8_t red_value = (color_value & 0xFF0000) >> 16;
	uint8_t green_value = (color_value & 0x00FF00) >> 8;
	uint8_t blue_value = (color_value & 0x0000FF);

	memset(p_colors, 0, total_lamp_num * 3);
	for(int i =0; i < lamp_num; i++){
		p_colors[0] = red_value;
		p_colors[1] = green_value;
		p_colors[2] = blue_value;
		p_colors += 3;
	}
}

// LOGO lamp
#define LOGO_COUNT 23
#define LOGO_DMA_INDEX 0
uint8_t g_logo_lamp_flag = 0;
uint8_t g_logo_lamp_value_on = 0;
uint32_t g_logo_lamp_color_value;

uint8_t g_logo_lamp_wait_effect = 0x00;

uint8_t g_logo_lamp_buffer[LOGO_COUNT*3];
//__attribute__((aligned(4))) volatile DMA_TYPE g_logo_lamp_value[BUFFER_LOGO_BUFFER_LEN];

//void stop_update_logo_lamp_pwm_value(){
//	if (g_logo_lamp_flag == 1){
//		g_logo_lamp_flag = 0;
//		HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
//	}
//}

void logo_lamp_callback();

void change_logo_lamp_color(){
	if (g_logo_lamp_flag == 1){
		g_logo_lamp_wait_effect = 0x01;
		return;
	}

	g_logo_lamp_wait_effect = 0x00;
	pwm_dma_init(0,&htim1,TIM_CHANNEL_2,g_logo_lamp_buffer,LOGO_COUNT, logo_lamp_callback);
	fill_lamp_buffer(g_logo_lamp_buffer, g_logo_lamp_color_value, LOGO_COUNT, LOGO_COUNT);
	pwm_dma_send(LOGO_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	g_logo_lamp_flag = 1;
}

void change_logo_lamp_black(){
	if (g_logo_lamp_flag == 1){
		g_logo_lamp_wait_effect = 0x02;
		return;
	}

	g_logo_lamp_wait_effect = 0x00;
	pwm_dma_init(0,&htim1,TIM_CHANNEL_2,g_logo_lamp_buffer,LOGO_COUNT, logo_lamp_callback);
	fill_lamp_buffer(g_logo_lamp_buffer, 0x00, LOGO_COUNT, LOGO_COUNT);
	pwm_dma_send(LOGO_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	g_logo_lamp_flag = 1;
}

void logo_lamp_callback(){
	g_logo_lamp_flag = 0;
	if( g_logo_lamp_wait_effect == 0x00){
		return;
	}

	if (g_logo_lamp_wait_effect == 0x01){
		change_logo_lamp_color();
		return;
	}

	if(g_logo_lamp_wait_effect == 0x02){
		change_logo_lamp_black();
		return;
	}
}

//-------------------------------
// Energy Lamp
#define ENERGY_COUNT 21
#define ENERGY_DMA_INDEX 1
uint8_t g_energy_lamp_flag = 0;
uint8_t g_energy_lamp_value_on = 0;
uint8_t g_energy_lamp_effect = 0;
uint32_t g_energy_lamp_color_value;

uint8_t g_energy_lamp_wait_effect = 0x00;
uint8_t g_energy_lamp_wait_effect_value = 0x00;

uint8_t g_energy_lamp_buffer[ENERGY_COUNT*3];

void energy_lamp_callback();

//void stop_update_energy_lamp_pwm_value(){
//	if (g_energy_lamp_flag == 1){
//		g_energy_lamp_flag = 0;
//		HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
//	}
//}

void change_energy_lamp_with_count_dma(uint8_t count){
	pwm_dma_init(ENERGY_DMA_INDEX,&htim1,TIM_CHANNEL_3,g_energy_lamp_buffer,ENERGY_COUNT, energy_lamp_callback);
	fill_lamp_buffer(g_energy_lamp_buffer, g_energy_lamp_color_value, count, ENERGY_COUNT);
	pwm_dma_send(ENERGY_DMA_INDEX, LAMP_NON_BLOCK_MODE);
	g_energy_lamp_flag = 1;
}

void change_energy_lamp_color(){
	if (g_energy_lamp_flag == 1){
		g_energy_lamp_wait_effect = 0x01;
		return;
	}
	g_energy_lamp_wait_effect = 0x00;
	pwm_dma_init(ENERGY_DMA_INDEX,&htim1,TIM_CHANNEL_3,g_energy_lamp_buffer,ENERGY_COUNT, energy_lamp_callback);
	fill_lamp_buffer(g_energy_lamp_buffer, g_energy_lamp_color_value, ENERGY_COUNT, ENERGY_COUNT);
	pwm_dma_send(ENERGY_DMA_INDEX, LAMP_NON_BLOCK_MODE);
	g_energy_lamp_flag = 1;
}

void change_energy_lamp_black(){
	if (g_energy_lamp_flag == 1){
		g_energy_lamp_wait_effect = 0x03;
		return;
	}

	g_energy_lamp_wait_effect = 0x00;
	pwm_dma_init(ENERGY_DMA_INDEX,&htim1,TIM_CHANNEL_3,g_energy_lamp_buffer,ENERGY_COUNT, energy_lamp_callback);
	fill_lamp_buffer(g_energy_lamp_buffer, 0x00, ENERGY_COUNT, ENERGY_COUNT);
	pwm_dma_send(ENERGY_DMA_INDEX, LAMP_NON_BLOCK_MODE);
	g_energy_lamp_flag = 1;
}

void change_energy_lamp_with_circle_count(uint8_t circle_count){
	if (g_energy_lamp_flag == 1){
		g_energy_lamp_wait_effect = 0x02;
		g_energy_lamp_wait_effect_value = circle_count;
		return;
	}

	g_energy_lamp_wait_effect = 0x00;

	uint8_t count = 1;
	if (circle_count>100){
		count = 21;
	}
	else if(circle_count == 0){
		count = 0;
	}
	else{
		if ((circle_count % 5) == 0){
			count = circle_count / 5;
		}
		else{
			count = (int)(circle_count / 5) + 1;
		}
	}
	change_energy_lamp_with_count_dma(count);
}

void energy_lamp_callback(){
	g_energy_lamp_flag = 0;
	if (g_energy_lamp_wait_effect == 0x00){
		return;
	}

	if (g_energy_lamp_wait_effect == 0x01){
		change_energy_lamp_color();
		return;
	}

	if(g_energy_lamp_wait_effect == 0x02){
		change_energy_lamp_with_circle_count(g_energy_lamp_wait_effect_value);
		return;
	}

	if(g_energy_lamp_wait_effect == 0x03){
		change_energy_lamp_black();
		return;
	}
}


// Side Lamp
#define SIDE_LAMP_COUNT 168

#define SIDE_LAMP_1_DMA_INDEX 2
#define SIDE_LAMP_2_DMA_INDEX 3

// side lamp 1
uint8_t g_side_lamp_flag = 0;
uint8_t g_side_lamp_value_on = 0;
uint8_t g_side_lamp_effect = 0;
uint32_t g_side_lamp_color_value;

#define SIDE_LAMP_BUFFER_LEN (SIDE_LAMP_COUNT*3)

uint8_t g_side_1_lamp_buffer[SIDE_LAMP_BUFFER_LEN];
uint8_t g_side_2_lamp_buffer[SIDE_LAMP_BUFFER_LEN];

uint8_t g_side_lamp_wait_count = 0x00;
uint8_t g_side_lamp_wait_order = 0x00;

uint8_t g_side_lamp_wait_effect = 0x00;
uint8_t g_side_lamp_wait_effect_value = 0x00;

uint8_t g_side_lamp_effect_flag = 0;

void side_lamp_callback();

void change_side_lamp_color(){
	if (g_side_lamp_flag > 0){
		g_side_lamp_wait_effect = 0x01;
		return;
	}

	g_side_lamp_wait_effect = 0x00;
	pwm_dma_init(SIDE_LAMP_1_DMA_INDEX,&htim16,TIM_CHANNEL_1,g_side_1_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	pwm_dma_init(SIDE_LAMP_2_DMA_INDEX,&htim17,TIM_CHANNEL_1,g_side_2_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	fill_lamp_buffer(g_side_1_lamp_buffer, g_side_lamp_color_value, SIDE_LAMP_COUNT, SIDE_LAMP_COUNT);
	fill_lamp_buffer(g_side_2_lamp_buffer, g_side_lamp_color_value, SIDE_LAMP_COUNT, SIDE_LAMP_COUNT);
	pwm_dma_send(SIDE_LAMP_1_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	pwm_dma_send(SIDE_LAMP_2_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	g_side_lamp_flag = 2;
	g_side_lamp_effect_flag = 0;
}

void change_side_lamp_black(){
	if (g_side_lamp_flag > 0 ){
		g_side_lamp_wait_effect = 0x06;
		return;
	}

	g_side_lamp_wait_effect = 0x00;
	pwm_dma_init(SIDE_LAMP_1_DMA_INDEX,&htim16,TIM_CHANNEL_1,g_side_1_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	pwm_dma_init(SIDE_LAMP_2_DMA_INDEX,&htim17,TIM_CHANNEL_1,g_side_2_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	fill_lamp_buffer(g_side_1_lamp_buffer, 0x00, SIDE_LAMP_COUNT, SIDE_LAMP_COUNT);
	fill_lamp_buffer(g_side_2_lamp_buffer, 0x00, SIDE_LAMP_COUNT, SIDE_LAMP_COUNT);
	pwm_dma_send(SIDE_LAMP_1_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	pwm_dma_send(SIDE_LAMP_2_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	g_side_lamp_flag = 2;
	g_side_lamp_effect_flag = 0;
}

void change_side_lamp_with_count_dma(uint8_t count){
	if (g_side_lamp_flag > 0){
		g_side_lamp_wait_count = count;
		return;
	}

	g_side_lamp_wait_count = 0;
	pwm_dma_init(SIDE_LAMP_1_DMA_INDEX,&htim16,TIM_CHANNEL_1,g_side_1_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	pwm_dma_init(SIDE_LAMP_2_DMA_INDEX,&htim17,TIM_CHANNEL_1,g_side_2_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	fill_lamp_buffer(g_side_1_lamp_buffer, g_side_lamp_color_value, count, SIDE_LAMP_COUNT);
	fill_lamp_buffer(g_side_2_lamp_buffer, g_side_lamp_color_value, count, SIDE_LAMP_COUNT);
	pwm_dma_send(SIDE_LAMP_1_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	pwm_dma_send(SIDE_LAMP_2_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	g_side_lamp_flag = 2;
}

// side lamp effect
uint32_t g_side_lamp_effect_start = 0;

//effect_1
uint8_t g_side_lamp_flow_effect_1_current_count = 0;

//effect_2
uint8_t g_side_lamp_flow_effect_2_current_count = 0;

//pulse_effect
uint8_t g_side_lamp_pulse_effect_phase = 0;
uint8_t g_side_lamp_pulse_grey_color=0;
uint32_t g_side_lamp_wait_color_value = 0;
uint32_t g_side_lamp_wait_color_flag = 0;

//void stop_update_side_lamp_pwm_value(){
//	if (g_side_lamp_effect_flag > 0){
//		g_side_lamp_effect_flag = 0;
//	}
//
////	if (g_side_lamp_flag == 1){
////		g_side_lamp_flag = 0;
////		HAL_TIM_PWM_Stop_DMA(&htim16, TIM_CHANNEL_1);
////		HAL_TIM_PWM_Stop_DMA(&htim17, TIM_CHANNEL_1);
////	}
//}

void start_side_lamp_flow_effect_1(uint8_t effect_value){
	if( g_side_lamp_flag > 0 ){
		g_side_lamp_wait_effect = effect_value;
		return;
	}

	g_side_lamp_wait_effect = 0x00;
	g_side_lamp_effect_start = osKernelGetSysTimerCount();
	g_side_lamp_flow_effect_1_current_count = 1;
	change_side_lamp_with_count_dma(g_side_lamp_flow_effect_1_current_count);
	g_side_lamp_effect_flag = effect_value;
}

void update_sied_lamp_flow_effect_1(){
	uint32_t tick;
	tick = osKernelGetSysTimerCount();
	uint32_t diff = tick - g_side_lamp_effect_start;
	uint32_t freq = osKernelGetSysTimerFreq();
	uint32_t timeout_value = 0.015 * freq;
	if (diff >= timeout_value){
		g_side_lamp_flow_effect_1_current_count++;
		g_side_lamp_effect_start = osKernelGetSysTimerCount();
		change_side_lamp_with_count_dma(g_side_lamp_flow_effect_1_current_count);
		if (g_side_lamp_flow_effect_1_current_count >= SIDE_LAMP_COUNT){
			g_side_lamp_effect_flag = 0;
		}
	}
}

void start_side_lamp_flow_effect_2(uint8_t effect_value){
	if( g_side_lamp_flag > 0 ){
		g_side_lamp_wait_effect = effect_value;
		return;
	}

	g_side_lamp_wait_effect = 0x00;
	g_side_lamp_effect_start = osKernelGetSysTimerCount();
	g_side_lamp_flow_effect_2_current_count = SIDE_LAMP_COUNT;
	change_side_lamp_with_count_dma(g_side_lamp_flow_effect_2_current_count);
	g_side_lamp_effect_flag = effect_value;
}

void update_sied_lamp_flow_effect_2(){
	uint32_t tick;
	tick = osKernelGetSysTimerCount();
	uint32_t diff = tick - g_side_lamp_effect_start;
	uint32_t freq = osKernelGetSysTimerFreq();
	uint32_t timeout_value = 0.015 * freq;
	if (diff >= timeout_value){
		g_side_lamp_flow_effect_2_current_count--;
		g_side_lamp_effect_start = osKernelGetSysTimerCount();
		change_side_lamp_with_count_dma(g_side_lamp_flow_effect_2_current_count);
		if (g_side_lamp_flow_effect_2_current_count<=0){
			g_side_lamp_effect_flag = 0;
		}
	}
}

void change_side_lamp_pulse_color(uint32_t color_value){
	if (g_side_lamp_flag > 0){
		g_side_lamp_wait_color_value = color_value;
		g_side_lamp_wait_color_flag = 1;
		return;
	}

	g_side_lamp_wait_color_flag = 0;
	pwm_dma_init(SIDE_LAMP_1_DMA_INDEX,&htim16,TIM_CHANNEL_1,g_side_1_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	pwm_dma_init(SIDE_LAMP_2_DMA_INDEX,&htim17,TIM_CHANNEL_1,g_side_2_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	fill_lamp_buffer(g_side_1_lamp_buffer, color_value, SIDE_LAMP_COUNT, SIDE_LAMP_COUNT);
	fill_lamp_buffer(g_side_2_lamp_buffer, color_value, SIDE_LAMP_COUNT, SIDE_LAMP_COUNT);
	pwm_dma_send(SIDE_LAMP_1_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	pwm_dma_send(SIDE_LAMP_2_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	g_side_lamp_flag = 2;
}

void start_side_lamp_pulse_effect(uint8_t effect_value){
	if( g_side_lamp_flag > 0 ){
		g_side_lamp_wait_effect = effect_value;
		return;
	}

	g_side_lamp_wait_effect = 0x00;
	g_side_lamp_effect_start = osKernelGetSysTimerCount();
	uint8_t gray_value = 255;
	g_side_lamp_pulse_grey_color=gray_value;
	uint32_t color = (gray_value << 16) | (gray_value << 8) | gray_value;
	change_side_lamp_pulse_color(color);
	g_side_lamp_pulse_effect_phase = 0;
	g_side_lamp_effect_flag = effect_value;
}

void update_side_lamp_pulse_effect(){
	if (g_side_lamp_pulse_effect_phase == 0){
		uint32_t tick;
		tick = osKernelGetSysTimerCount();
		uint32_t diff = tick - g_side_lamp_effect_start;
		uint32_t freq = osKernelGetSysTimerFreq();
		uint32_t timeout_value = 0.015 * freq;
		if (diff >= timeout_value){
			g_side_lamp_pulse_grey_color--;
			g_side_lamp_effect_start = osKernelGetSysTimerCount();
			uint8_t gray_value = g_side_lamp_pulse_grey_color;
			uint32_t color = (gray_value << 16) | (gray_value << 8) | gray_value;
			change_side_lamp_pulse_color(color);
			if (g_side_lamp_pulse_grey_color == 0){
				g_side_lamp_pulse_effect_phase = 1;
				g_side_lamp_effect_start = osKernelGetSysTimerCount();
				return;
			}
		}
	}
	else if(g_side_lamp_pulse_effect_phase == 1){
		uint32_t tick;
		tick = osKernelGetSysTimerCount();
		uint32_t diff = tick - g_side_lamp_effect_start;
		uint32_t freq = osKernelGetSysTimerFreq();
		uint32_t timeout_value = 0.5 * freq;
		if (diff >= timeout_value){
			g_side_lamp_pulse_effect_phase = 2;
			g_side_lamp_effect_start = osKernelGetSysTimerCount();
			return;
		}
	}
	else if(g_side_lamp_pulse_effect_phase == 2){
		uint32_t tick;
		tick = osKernelGetSysTimerCount();
		uint32_t diff = tick - g_side_lamp_effect_start;
		uint32_t freq = osKernelGetSysTimerFreq();
		uint32_t timeout_value = 0.015 * freq;
		if (diff >= timeout_value){
			g_side_lamp_pulse_grey_color++;
			g_side_lamp_effect_start = osKernelGetSysTimerCount();
			uint8_t gray_value = g_side_lamp_pulse_grey_color;
			uint32_t color = (gray_value << 16) | (gray_value << 8) | gray_value;
			change_side_lamp_pulse_color(color);
			if (g_side_lamp_pulse_grey_color == 255){
				g_side_lamp_pulse_effect_phase = 3;
				g_side_lamp_effect_start = osKernelGetSysTimerCount();
				return;
			}
		}
	}
	else if(g_side_lamp_pulse_effect_phase == 3){
		uint32_t tick;
		tick = osKernelGetSysTimerCount();
		uint32_t diff = tick - g_side_lamp_effect_start;
		uint32_t freq = osKernelGetSysTimerFreq();
		uint32_t timeout_value = 0.5 * freq;
		if (diff >= timeout_value){
			g_side_lamp_pulse_effect_phase = 0;
			g_side_lamp_effect_start = osKernelGetSysTimerCount();
			return;
		}
	}
}

float g_revolving_scenic_lantern_time = 1.0;
uint8_t g_side_lamp_effect_order = 0x00;

void fill_lamp_buffer_with_order(uint8_t* p_colors, uint32_t color_value, uint8_t total_lamp_num, uint8_t order ){
	uint8_t red_value = (color_value & 0xFF0000) >> 16;
	uint8_t green_value = (color_value & 0x00FF00) >> 8;
	uint8_t blue_value = (color_value & 0x0000FF);

	memset(p_colors, 0, total_lamp_num * 3);

	int pos =0;
	int n = 0;
	int pos1 = 0;
	int pos2 = 0;
	while( n < 5){
		pos1 = ((n*28) + 1) + order;
		pos2 = ((n+1) *28) + order;

		for(pos = pos1; pos <= pos2; pos++){
			p_colors[(pos-1)*3 + 0] = red_value;
			p_colors[(pos-1)*3 + 1] = green_value;
			p_colors[(pos-1)*3 + 2] = blue_value;
		}

		n+=2;
	}
}

void change_side_lamp_with_order(uint8_t order){
	if( g_side_lamp_flag > 0 ){
		g_side_lamp_wait_order = order;
		return;
	}

	g_side_lamp_wait_order = 0;
	pwm_dma_init(SIDE_LAMP_1_DMA_INDEX,&htim16,TIM_CHANNEL_1,g_side_1_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	pwm_dma_init(SIDE_LAMP_2_DMA_INDEX,&htim17,TIM_CHANNEL_1,g_side_2_lamp_buffer,SIDE_LAMP_COUNT, side_lamp_callback);
	fill_lamp_buffer_with_order(g_side_1_lamp_buffer, g_side_lamp_color_value, SIDE_LAMP_COUNT, order);
	fill_lamp_buffer_with_order(g_side_2_lamp_buffer, g_side_lamp_color_value, SIDE_LAMP_COUNT, order);
	pwm_dma_send(SIDE_LAMP_1_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	pwm_dma_send(SIDE_LAMP_2_DMA_INDEX,LAMP_NON_BLOCK_MODE);
	g_side_lamp_flag = 2;
}


void start_side_lamp_revolving_scenic_lantern_effect(uint8_t effect_value, uint8_t circle_count){
	if(g_side_lamp_flag == 1){
		g_side_lamp_wait_effect = effect_value;
		g_side_lamp_wait_effect_value = circle_count;
		return;
	}

	g_side_lamp_wait_effect = 0;
	if (circle_count == 0){
		circle_count = 1;
	}
	if (circle_count > 150){
		circle_count = 150;
	}
	float recircle_time = 60/circle_count/2*3;
	g_revolving_scenic_lantern_time = recircle_time/28.0;

	g_side_lamp_effect_order = 0x00;

	g_side_lamp_effect_start = osKernelGetSysTimerCount();
	g_side_lamp_effect_flag = effect_value;
	change_side_lamp_with_order(g_side_lamp_effect_order);
}

void update_side_lamp_revolving_scenic_lantern_effect(){
	uint32_t tick;
	tick = osKernelGetSysTimerCount();
	uint32_t diff = tick - g_side_lamp_effect_start;
	uint32_t freq = osKernelGetSysTimerFreq();
	uint32_t timeout_value = g_revolving_scenic_lantern_time * freq;
	if (diff >= timeout_value){
		g_side_lamp_effect_start = osKernelGetSysTimerCount();
		if (g_side_lamp_effect_order < 28){
			g_side_lamp_effect_order++;
			change_side_lamp_with_order(g_side_lamp_effect_order);
			return;
		}

		g_side_lamp_effect_order = 0x00;
		change_side_lamp_with_order(0x00);
		return;
	}
}

void side_lamp_callback(){
	if (g_side_lamp_flag == 2){
		g_side_lamp_flag = 1;
		return;
	}
	if (g_side_lamp_flag == 1){
		g_side_lamp_flag = 0;
	}

	if(g_side_lamp_wait_color_flag == 1){
		change_side_lamp_pulse_color(g_side_lamp_wait_color_value);
		return;
	}

	if (g_side_lamp_wait_count > 0){
		change_side_lamp_with_count_dma(g_side_lamp_wait_count);
		return;
	}

	if (g_side_lamp_wait_order > 0){
		change_side_lamp_with_order(g_side_lamp_wait_order);
		return;
	}

	if (g_side_lamp_wait_effect == 0x00){
		return;
	}

	if (g_side_lamp_wait_effect == 0x01){
		change_side_lamp_color();
		return;
	}

	if (g_side_lamp_wait_effect == 0x02){
		start_side_lamp_pulse_effect(0x02);
		return;
	}

	if (g_side_lamp_wait_effect == 0x03){
		start_side_lamp_revolving_scenic_lantern_effect(0x03, g_side_lamp_wait_effect_value);
		return;
	}

	if (g_side_lamp_wait_effect == 0x04){
		start_side_lamp_flow_effect_1(0x04);
		return;
	}

	if (g_side_lamp_wait_effect == 0x05){
		start_side_lamp_flow_effect_2(0x05);
		return;
	}

	if (g_side_lamp_wait_effect == 0x06){
		change_side_lamp_black();
		return;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim1);

  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim2);

  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);

  __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim8);

  //__HAL_TIM_CLEAR_IT(&htim16, TIM_IT_UPDATE);
  //HAL_TIM_Base_Start_IT(&htim16);

  //__HAL_TIM_CLEAR_IT(&htim17, TIM_IT_UPDATE);
  //HAL_TIM_Base_Start_IT(&htim17);

  // Calibrate The ADC On Power-Up For Better Accuracy
  uint32_t SingleDiff = ADC_SINGLE_ENDED;
  HAL_ADCEx_Calibration_Start(&hadc1, SingleDiff);

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
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
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 14;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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
  htim2.Init.Prescaler = 119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
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
  htim3.Init.Prescaler = 119;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
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
  htim8.Init.Prescaler = 119;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 19;
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
  htim16.Init.Prescaler = 14;
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
  htim17.Init.Prescaler = 14;
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
  huart4.Init.BaudRate = 2400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
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

	uint32_t default_color = 0x00FFFF;

	//g_start_lamp_order = 2;

	g_logo_lamp_color_value = default_color;
	g_energy_lamp_color_value = default_color;
	g_side_lamp_color_value = default_color;

	//start_side_1_lamp_flow_effect_1();


	fill_down_board_init_telegram();
	g_wait_down_board_start = osKernelGetSysTimerCount();
	HAL_UART_Transmit(&huart4,(uint8_t*)g_down_borad_init_data_ptr,0x08,0xffff);

	/* Infinite loop */
	for(;;)
	{
		if (g_side_lamp_effect_flag == 0x04){
			update_sied_lamp_flow_effect_1();
		}

		if (g_side_lamp_effect_flag == 0x05){
			update_sied_lamp_flow_effect_2();
		}

		if (g_side_lamp_effect_flag == 0x02){
			update_side_lamp_pulse_effect();
		}

		if (g_side_lamp_effect_flag == 0x03){
			update_side_lamp_revolving_scenic_lantern_effect();
		}

		if (g_strength_adc_flag == 1){
			if (g_strength_adc_calibration_flag == 1){
				update_strength_adc_calibration();
			}
			else{
				update_strength_adc();
			}
		}

		if (g_step_flag == 1 && g_idle_mode >=0){
			uint32_t tick;
			tick = osKernelGetSysTimerCount();
			uint32_t diff = tick - g_step_start;
			uint32_t freq = osKernelGetSysTimerFreq();
			uint32_t timeout_value = 1 * freq;
			if (diff >= timeout_value){
				g_step_start = osKernelGetSysTimerCount();

				g_step_value = g_step_count;

				g_step_count = 0;

				if ((g_step_value != g_old_step_value) || (g_strength_value!=g_old_strength_value)){
					put_two_int_and_one_byte_into_out_buffer(0x01, 0x05, g_step_value, g_strength_value, g_hall_detect_value);
					if(g_step_value != g_old_step_value){
						g_old_step_value = g_step_value;
					}
					if (g_strength_value!=g_old_strength_value){
						g_old_strength_value = g_strength_value;
					}
				}
			}
		}

		if (g_angle_report_flag == 0x01){
			g_angle_report_flag = 0x00;
			put_int_into_out_buffer(0x04, 0x01, g_angle_detect_value);
		}

		if(g_angle_adc_flag == 0x01 && g_angle_detect_flag == 0x01){
			update_angle_adc();
		}

		if (g_human_rader_1_report_flag == 0x01){
			g_human_rader_1_report_flag = 0x00;
			put_byte_into_out_buffer(0x03, 0x01, g_human_rader_1_report_value); //human rader 1
		}

		if (g_human_rader_2_report_flag == 0x01){
			g_human_rader_2_report_flag = 0x02;
			put_byte_into_out_buffer(0x03, 0x02, g_human_rader_2_report_value); //human rader 2
		}

		if (g_hall_detect_report_flag == 0x01){
			g_hall_detect_report_flag = 0x00;
			put_byte_into_out_buffer(0x02, 0x01, g_hall_detect_value);	//Hall detect
		}

		if (g_key_1_report_flag == 0x01){
			g_key_1_report_flag = 0x00;
			put_byte_into_out_buffer(0x05, 0x01, g_key_1_value);	//Key 1
		}

		if (g_key_2_report_flag == 0x01){
			g_key_2_report_flag = 0x00;
			put_byte_into_out_buffer(0x05, 0x02, g_key_2_value);	//Key 2
		}

		if (g_exit_idle_report_flag == 0x01){
			g_exit_idle_report_flag = 0x00;
			put_no_data_into_out_buffer(0x11, 0x05);  //notify exit idle mode
		}

		if (g_shudown_report_flag == 0x01){
			g_shudown_report_flag = 0x00;
			put_no_data_into_out_buffer(0x11, 0x01);  // notify start process of shutdown
		}

		if (g_idle_mode == -2){  //when MCU startup with power, send initialize telegram into down board, wait it reply.
			update_retry_down_board_communication();
		}

		if (g_motor_1 > 0){
			update_motor_1_action();
		}

		if (g_motor_2 > 0){
			update_motor_2_action();
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
						HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
						g_strength_adc_calibration_flag = 1;
						g_strength_adc_calibration_count = 0;
						g_strength_adc_calibration_value = 0;
						HAL_ADC_Start_IT(&hadc2);
					}
					else if(data_ptr[1] == 0x03){
						g_step_count = 0;
						g_step_start = osKernelGetSysTimerCount();
						HAL_ADC_Start_IT(&hadc2);
						g_step_flag = 1;
						g_step_value = 0x0;
						g_old_step_value = 0x0;
						g_strength_value = 0;
						g_old_strength_value = 0;
						g_capture_order = 1;
						__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
						HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
					}
					else if(data_ptr[1] == 0x04 ){
						HAL_ADC_Stop_IT(&hadc2);
						HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_4);
						g_strength_adc_flag = 0;
						g_step_flag = 0;
						g_step_count = 0;
						g_capture_order = 0;
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if(data_ptr[0] == 0x04){
					if(data_ptr[1] == 0x02 ){ //angle detect begin
						HAL_ADC_Start_IT(&hadc1);
						g_angle_detect_flag = 1;
					}
					else if (data_ptr[1] == 0x03 ){ //angle detect end
						g_angle_detect_flag = 0;
						HAL_ADC_Stop_IT(&hadc1);
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
									TIM3->CCR2 = g_fan_1_value;  // fan 1
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
									TIM8->CCR3 = g_fan_2_value;  // fan 2
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
									TIM2->CCR4 = g_fan_3_value;  // fan 3
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
									TIM8->CCR1 = g_fan_4_value;  //fan 4
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
				else if(data_ptr[0] == 0x0a){  //Side Lamp
					if(data_ptr[1] == 0x01 ){
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 6){
							uint8_t value_on = data_ptr[0];
							g_side_lamp_value_on = value_on;
							uint8_t value_red = data_ptr[1];
							uint8_t value_green = data_ptr[2];
							uint8_t value_blue = data_ptr[3];
							g_side_lamp_effect = data_ptr[4];
							if (value_on == 0x01){
								uint32_t color_value = (value_red << 16) | (value_green << 8) | value_blue;
								g_side_lamp_color_value = color_value;
								if (g_side_lamp_effect == 0x01){
									change_side_lamp_color();
								}
								else if(g_side_lamp_effect == 0x02){
									//pulse effect
									start_side_lamp_pulse_effect(0x02);
								}
								else if(g_side_lamp_effect == 0x03){
									//revolving scenic lantern effect
									uint8_t circle_count = data_ptr[5];
									start_side_lamp_revolving_scenic_lantern_effect(0x03, circle_count);
								}
								else if(g_side_lamp_effect == 0x04){
									start_side_lamp_flow_effect_1(0x04);
								}
								else if(g_side_lamp_effect == 0x05){
									start_side_lamp_flow_effect_2(0x05);
								}
								else{
									PutErrorCode(ErrorQueueHandle,0x08);
								}
							}
							else if (value_on == 0x02){
								change_side_lamp_black();
							}
							else{
								PutErrorCode(ErrorQueueHandle,0x08);
							}
						}
						else{
							PutErrorCode(ErrorQueueHandle,0x05);
						}
					}
					else if(data_ptr[1] == 0x02 ){
						uint32_t color_value = g_side_lamp_color_value;

						uint8_t red_value = (color_value & 0xFF0000) >> 16;
						uint8_t green_value = (color_value & 0x00FF00) >> 8;
						uint8_t blue_value = (color_value & 0x0000FF);
						put_five_bytes_into_out_buffer(0x0A, 0x03, g_side_lamp_value_on, red_value, green_value, blue_value, g_side_lamp_effect);
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if (data_ptr[0] == 0x0b){ //Energy Lamp
					if(data_ptr[1] == 0x01 ){
						char *len_ptr = data_ptr + 2;
						int len_value = compute_len(len_ptr);
						char * data_ptr = len_ptr + 4;
						if(len_value == 6){
							uint8_t value_on = data_ptr[0];
							g_energy_lamp_value_on = value_on;
							uint8_t value_red = data_ptr[1];
							uint8_t value_green = data_ptr[2];
							uint8_t value_blue = data_ptr[3];
							g_energy_lamp_effect = data_ptr[4];
							if (value_on == 0x01){
								uint32_t color_value = (value_red << 16) | (value_green << 8) | value_blue;
								g_energy_lamp_color_value = color_value;
								if (g_energy_lamp_effect == 0x01){
									change_energy_lamp_color();
								}
								else if(g_energy_lamp_effect == 0x02){
									uint8_t energy_lamp_count = data_ptr[5];
									change_energy_lamp_with_circle_count(energy_lamp_count);
								}
							}
							else if (value_on == 0x02){
								change_energy_lamp_black();
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
						uint32_t color_value = g_energy_lamp_color_value;

						uint8_t red_value = (color_value & 0xFF0000) >> 16;
						uint8_t green_value = (color_value & 0x00FF00) >> 8;
						uint8_t blue_value = (color_value & 0x0000FF);
						put_five_bytes_into_out_buffer(0x0B, 0x03, g_energy_lamp_value_on, red_value, green_value, blue_value, g_energy_lamp_effect);
					}
					else{
						PutErrorCode(ErrorQueueHandle,0x03);
					}
				}
				else if (data_ptr[0] == 0x0c){ //logo lamp
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
								change_logo_lamp_color();
							}
							else if (value_on == 0x02){
								change_logo_lamp_black();
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
				else if(data_ptr[0] == 0x10){
					if(data_ptr[1] == 0x01 ){
						g_idle_mode = 0;  // work mode
						put_no_data_into_out_buffer(0x010, 0x02);
					}
				}
				else if(data_ptr[0] == 0x11){
					if(data_ptr[1] == 0x02 ){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
					}
					else if(data_ptr[1] == 0x03 ){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
					}
					else if(data_ptr[1] == 0x04 ){
						g_idle_mode = 1; //sleep mode
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

