#include "main.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim1;

#define DUTY_0 1 //20 // 26% of 75  WS2812 = 32%
#define DUTY_1 3 //56 // 75% of 75  WS2812 = 62%
#define DUTY_RESET 0

#define TRST_LEN 64 // 64*1.25 = 80us

#define DMA_TYPE  uint8_t

//__attribute__((aligned(4))) volatile  DMA_TYPE pwm_buffer[BUFFER_LEN];

//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
//{
//	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
//}

//void make_led_pwm(uint32_t led_color)
//{
//  int i = 0;
//  DMA_TYPE* p_buffer;
//
//  p_buffer = (DMA_TYPE*)pwm_buffer;
//    // TRST Buffer
//  while (i < TRST_LEN){
//    p_buffer[i] = DUTY_RESET;
//    i++;
//  }
//
//  p_buffer = (DMA_TYPE*)pwm_buffer + i;
//  while (i < BUFFER_LED_LEN + TRST_LEN){
//      color_convert(p_buffer,led_color);
//      i += 24;
//      p_buffer += 24;
//  }
//
//  while(i < BUFFER_LEN){
//    p_buffer[i] = DUTY_RESET;
//    i++;
//  }
//}
//
//void led_dma_send()
//{
//  int res = HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)pwm_buffer,BUFFER_LEN);
//  if ( res != HAL_OK){
//    return ;
//  }
//}


void color_convert(DMA_TYPE* buffer,uint32_t led_color)
{
    DMA_TYPE bit_value = 0;
    for(int i = 0; i < 24; i++){
      bit_value = (led_color >> (23 - i)) & 0x01;
      if (bit_value == 0){
        buffer[i] = DUTY_0;
      }else {
        buffer[i] = DUTY_1;
      }
    }
}

void make_led_pwm(uint32_t led_color, volatile DMA_TYPE *pData, uint16_t LedLength, uint16_t BufferLength){
	int i = 0;
	DMA_TYPE* p_buffer;

	p_buffer = (DMA_TYPE*)pData;
	// TRST Buffer
	while (i < TRST_LEN){
		p_buffer[i] = DUTY_RESET;
		i++;
	}

	p_buffer = (DMA_TYPE*)pData + TRST_LEN;
	while (i < LedLength + TRST_LEN){
		color_convert(p_buffer,led_color);
		i += 24;
		p_buffer += 24;
	}

	while(i < BufferLength){
		p_buffer[i] = DUTY_RESET;
		i++;
	}
}

void led_dma_send(TIM_HandleTypeDef *htim, uint32_t Channel, volatile DMA_TYPE *pData, uint16_t BufferLength,
		uint32_t led_color, uint16_t LedLength)
{
	make_led_pwm(led_color, pData, LedLength, BufferLength);
	int res = HAL_TIM_PWM_Start_DMA(htim,Channel,(uint32_t*)pData,BufferLength);
	if ( res != HAL_OK){
		return;
	}
}

void make_led_pwm_count(uint32_t led_color, volatile DMA_TYPE *pData, uint16_t LedLength, uint16_t BufferLength, uint8_t count){
	int i = 0;
	DMA_TYPE* p_buffer;

	p_buffer = (DMA_TYPE*)pData;
	// TRST Buffer
	while (i < TRST_LEN){
		p_buffer[i] = DUTY_RESET;
		i++;
	}

	int len_count = TRST_LEN + count * 24;
	p_buffer = (DMA_TYPE*)pData + TRST_LEN;
	while (i < LedLength + TRST_LEN){
		if (i < len_count){
			color_convert(p_buffer,led_color);
		}
		else{
			color_convert(p_buffer,0x00);
		}
		i += 24;
		p_buffer += 24;
	}

	while(i < BufferLength){
		p_buffer[i] = DUTY_RESET;
		i++;
	}
}
