#include "main.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim1;

#define LED_COUNT 100

#define DUTY_0 20 // 26% of 75  WS2812 = 32%
#define DUTY_1 56 // 75% of 75  WS2812 = 62%
#define DUTY_RESET 0

#define LED_NUM 21
#define TRST_LEN 64 // 64*1.25 = 80us
#define BUFFER_LED_LEN  LED_NUM*24
#define BUFFER_LEN (TRST_LEN + BUFFER_LED_LEN + TRST_LEN)

uint16_t pwm_buffer[BUFFER_LEN];

int pwm_send_flag=0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
	pwm_send_flag=1;
}

// void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
// {
// 	pwm_send_flag=0;
// }

void color_convert(uint16_t* buffer,uint32_t led_color)
{
    uint16_t bit_value = 0;
    for(int i = 0; i < 24; i++){
      bit_value = (led_color >> (23 - i)) & 0x01;
      if (bit_value == 0){
        buffer[i] = DUTY_0;
      }else {
        buffer[i] = DUTY_1;
      }

    }
}

void make_led_pwm(uint32_t led_color)
{
  int i = 0;
  uint16_t* p_buffer;
    // TRST Buffer
  while (i < TRST_LEN){
    pwm_buffer[i] = DUTY_RESET;
    i++;
  }

  p_buffer = pwm_buffer + i;
  while (i < BUFFER_LED_LEN + TRST_LEN){
      color_convert(p_buffer,led_color);
      i += 24;
      p_buffer += 24;
  }

  while(i < BUFFER_LEN){
    pwm_buffer[i] = DUTY_RESET;
    i++;
  }
}

void led_dma_send()
{
  int res = HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)pwm_buffer,BUFFER_LEN);
  if ( res != HAL_OK){
    return ;
  }
  // HAL_Delay(10);
  // HAL_TIM_PWM_Stop_DMA(&htim1,TIM_CHANNEL_3);
   while(!pwm_send_flag){
     HAL_Delay(1);
	//   //osDelay(1);
   }

  pwm_send_flag = 0;
}
