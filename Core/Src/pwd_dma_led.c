/*
 * pwd_dma.led.c
 *
 *  Created on: 2022/01/16
 *      Author: duan zhiqiang
 */

#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "pwm_dma_led.h"

PWM_DMA_DATA_STRUCT pwm_dma_data[PWM_LED_CHANNEL_MAX_COUNT];

PWM_DMA_DATA_STRUCT* get_pwm_data(TIM_HandleTypeDef *htim)
{
    // Get current working DMA
    uint32_t channel;
    switch(htim->Channel){
        case HAL_TIM_ACTIVE_CHANNEL_1:
            channel = TIM_CHANNEL_1;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            channel = TIM_CHANNEL_2;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_3:
            channel = TIM_CHANNEL_3;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_4:
            channel = TIM_CHANNEL_4;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_5:
            channel = TIM_CHANNEL_5;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_6:
            channel = TIM_CHANNEL_6;
            break;
        default:
            return NULL;
    }

    PWM_DMA_DATA_STRUCT* p_data = NULL;
    for(int i = 0; i < PWM_LED_CHANNEL_MAX_COUNT; i++){
        if (pwm_dma_data[i].htim == htim && pwm_dma_data[i].dma_channel == channel){
            p_data =  (PWM_DMA_DATA_STRUCT*)pwm_dma_data + i;
            break;
        }
    }
    return p_data;
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    PWM_DMA_DATA_STRUCT* p_data = get_pwm_data(htim);
    if (p_data != NULL){
        led_data_fill(p_data,0);
    }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    PWM_DMA_DATA_STRUCT* p_data = get_pwm_data(htim);
    if (p_data != NULL){
        led_data_fill(p_data,1);
    }
}

void led_color_convert(uint8_t* p_led_buffer,DMA_TYPE* p_dma_buffer)
{
    DMA_TYPE bit_value = 0;
    uint8_t led_value;
    DMA_TYPE* p_dest = p_dma_buffer;
    for(int i = 0; i < 3; i++){
        led_value = p_led_buffer[i];
        for(int j = 0; j <8; j++){
            bit_value = (led_value >> (8 - j)) & 0x01;
            if (bit_value == 0){
                *p_dest = DUTY_0;
            }else {
                *p_dest = DUTY_1;
            }
            p_dest++;
        }
    }
}

/**
* @brief Fill DMA Buffer data.
*   PWM_DMA_HEAD_RST: Fill all data with TRST,
*   PWM_DMA_LED_DATA: Fill half buffer with led color (one LED),
*            if b_half == 1, means half DMA data send finished,
*            if b_half == 0, means once DMA data send finished,
*   PWM_DMA_END_RST: Fill all data with TRST, and if b_half == 0 ,means the DMA send all filinshed
* @param dma_id: the PWM_DMA item (0 - PWM_LED_CHANNEL_MAX_COUNT-1)
* @param htim: pwm Timer
* @param channel: pwm DMA channel
* @param p_colors: the color buffer of LEDs, every LED color use 24bit (RGB)  = 3 Byte
* @param leds_count: the numbers of LEDs
* @retval 0: success
*/
void led_data_fill(PWM_DMA_DATA_STRUCT* p_dma_data,uint8_t b_half)
{
    DMA_TYPE *p_pwm_buffer = (DMA_TYPE*)p_dma_data->inter_dma_data.pwm_buffer;

    if ( p_dma_data->inter_dma_data.status == PWM_DMA_LED_SET){

        if (p_dma_data->inter_dma_data.cur_led < p_dma_data->total_leds){
            uint8_t *p_led_buffer = p_dma_data->p_dma_colors + 3 * p_dma_data->inter_dma_data.cur_led;
            if (b_half == 0){ //Once DMA Send Finished
                p_pwm_buffer += DMA_BUFFER_HALF_LEN;
            }
            led_color_convert(p_led_buffer,p_pwm_buffer);
            p_dma_data->inter_dma_data.cur_led++;
        }else{ //Last LED filled
            if(b_half == 1){
                memset(p_pwm_buffer,0,DMA_BUFFER_HALF_LEN*sizeof(DMA_TYPE));
                p_dma_data->inter_dma_data.status = PWM_DMA_END_RST;
            }else{
                p_pwm_buffer += DMA_BUFFER_HALF_LEN;
                memset(p_pwm_buffer,0,DMA_BUFFER_HALF_LEN*sizeof(DMA_TYPE));
                p_dma_data->inter_dma_data.status = PWM_DMA_END_RST;
            }
        }
        return ;
    }

    if ( p_dma_data->inter_dma_data.status == PWM_DMA_END_RST ){

        if (b_half == 0){
            p_pwm_buffer += DMA_BUFFER_HALF_LEN;
            memset(p_pwm_buffer,0,DMA_BUFFER_HALF_LEN*sizeof(DMA_TYPE));
            p_dma_data->inter_dma_data.status = PWM_DMA_END;
        }else{
            memset(p_pwm_buffer,0,DMA_BUFFER_HALF_LEN*sizeof(DMA_TYPE));
        }
        return;
    }

    if ( p_dma_data->inter_dma_data.status == PWM_DMA_HEAD_RST ){
        // Set First TRST
        memset(p_pwm_buffer,0,DMA_BUFFER_LEN*sizeof(DMA_TYPE));
        p_dma_data->inter_dma_data.status = PWM_DMA_LED_SET;
        return;
    }

    if ( p_dma_data->inter_dma_data.status == PWM_DMA_END && b_half == 0){
        // All DMA send Finished
	    HAL_TIM_PWM_Stop_DMA(p_dma_data->htim,
                             p_dma_data->dma_channel);
        p_dma_data->inter_dma_data.b_completed = 1;
        return;
    }

}
/**
* @brief Initilaize the pwm_dma data(Global array pwm_dma_data) according to dma_id.
*
* @param dma_id: the PWM_DMA item (0 - PWM_LED_CHANNEL_MAX_COUNT-1)
* @param htim: pwm Timer
* @param channel: pwm DMA channel
* @param p_colors: the color buffer of LEDs, every LED color use 24bit (RGB)  = 3 Byte
* @param leds_count: the numbers of LEDs
* @retval 0: success
*/
void pwm_dma_init(uint32_t dma_id, TIM_HandleTypeDef *htim, uint32_t channel,
                 uint8_t* p_colors, uint32_t leds_count )
{
    if (dma_id >= PWM_LED_CHANNEL_MAX_COUNT){
        return;
    }

    pwm_dma_data[dma_id].htim = htim;
    pwm_dma_data[dma_id].dma_channel = channel;
    pwm_dma_data[dma_id].p_dma_colors = p_colors;
    pwm_dma_data[dma_id].total_leds = leds_count;
}

/**
* @brief Send colors to LEDs by PWM + DMA + Circular mode
*
* @param dma_id: the PWM_DMA item (0 - PWM_LED_CHANNEL_MAX_COUNT-1)
* @param channel: pwm DMA channel
* @param p_colors: the color buffer of LEDs, every LED color use 24bit (RGB)  = 3 Byte
* @param leds_count: the numbers of LEDs
* @param b_block: flag whether waiting complete for DMA send. b_block = 1, waiting block mode , = 0 noblock mode
* @retval 0: success
*/
int pwm_dma_send(uint32_t dma_id,uint8_t b_block)
{
    int res = 0;
    if (dma_id >= PWM_LED_CHANNEL_MAX_COUNT){
        return -1;
    }

    if(pwm_dma_data[dma_id].htim == NULL){
        return -1;
    }

    pwm_dma_data[dma_id].inter_dma_data.status = PWM_DMA_HEAD_RST;
    pwm_dma_data[dma_id].inter_dma_data.cur_led = 0;
    pwm_dma_data[dma_id].inter_dma_data.b_completed = 0;

    led_data_fill(pwm_dma_data+dma_id,0);
    res = HAL_TIM_PWM_Start_DMA(pwm_dma_data[dma_id].htim,
                                pwm_dma_data[dma_id].dma_channel,
                                (uint32_t*)(pwm_dma_data[dma_id].inter_dma_data.pwm_buffer),
                                DMA_BUFFER_LEN);
    if ( res != HAL_OK){
        return res;
    }

    if (b_block){ // Block Mode
        while(pwm_dma_data[dma_id].inter_dma_data.b_completed == 0){
            osDelay(1);
        }
    }

    return res;
}
