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

void byte_to_duty(uint8_t byte_value, DMA_TYPE* p_dma_buffer)
{
    DMA_TYPE bit_value = 0;
    DMA_TYPE* p_dest = p_dma_buffer;
    for(int i = 0; i < 8; i++){
        bit_value = (byte_value >> (8 - i)) & 0x01;
        if (bit_value == 0){
            *p_dest = DUTY_0;
        }else {
            *p_dest = DUTY_1;
        }
        p_dest++;
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
    DMA_TYPE *p_pwm_buffer = (DMA_TYPE*)(p_dma_data->inter_dma_data.pwm_buffer);
    int led_len = p_dma_data->total_leds*3;

    if (b_half == 0){ // Fill the end half data
        if (p_dma_data->inter_dma_data.cur_pos >= led_len + TRST_BYTE_LEN){ // All Data Finished
            // All DMA send Finished
            HAL_TIM_PWM_Stop_DMA(p_dma_data->htim,
                                p_dma_data->dma_channel);
            p_dma_data->inter_dma_data.b_busy = 0;
            if (p_dma_data->send_finishedCallback != NULL){
                p_dma_data->send_finishedCallback(p_dma_data);
            }
            return;
        }

        p_pwm_buffer += DMA_BUFFER_HALF_LEN;
    }

    int pos = 0;
    uint8_t* p_colors;

    while( pos < DMA_BUFFER_HALF_LEN){
        if (p_dma_data->inter_dma_data.cur_pos < 0){ // Add Header TRST
            memset(p_pwm_buffer,DUTY_RESET,8);
        }else if (p_dma_data->inter_dma_data.cur_pos < led_len){ // LED Value; 8
            p_colors = (uint8_t*)(p_dma_data->p_dma_colors + p_dma_data->inter_dma_data.cur_pos);
            byte_to_duty(*p_colors,p_pwm_buffer);
        }else if (p_dma_data->inter_dma_data.cur_pos < led_len + TRST_BYTE_LEN){ //End TRST
            memset(p_pwm_buffer,DUTY_RESET,8);
        }else{  // LED DATA Finsh
            memset(p_pwm_buffer,DUTY_HIGHT,8);
        }
        p_dma_data->inter_dma_data.cur_pos++;
        pos += 8;
        p_pwm_buffer += 8;
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
* @param p_callback: Callback function
* @retval 0: success
*/
void pwm_dma_init(uint32_t dma_id, TIM_HandleTypeDef *htim, uint32_t channel,
                 uint8_t* p_colors, uint32_t leds_count, void *p_callback )
{
    if (dma_id >= PWM_LED_CHANNEL_MAX_COUNT){
        return;
    }

    pwm_dma_data[dma_id].htim = htim;
    pwm_dma_data[dma_id].dma_channel = channel;
    pwm_dma_data[dma_id].p_dma_colors = p_colors;
    pwm_dma_data[dma_id].total_leds = leds_count;
    pwm_dma_data[dma_id].send_finishedCallback = p_callback;
    pwm_dma_data[dma_id].inter_dma_data.b_busy = 0;
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
    int res = PWM_DMA_OK;
    if (dma_id >= PWM_LED_CHANNEL_MAX_COUNT){
        return PWM_DMA_ERROR_IDX;
    }

    if(pwm_dma_data[dma_id].htim == NULL){
        return PWM_DMA_ERROR_INIT;
    }

    pwm_dma_data[dma_id].inter_dma_data.cur_pos = -TRST_BYTE_LEN;
    pwm_dma_data[dma_id].inter_dma_data.b_busy = 1;

    led_data_fill(pwm_dma_data+dma_id,1);
    led_data_fill(pwm_dma_data+dma_id,0);
    res = HAL_TIM_PWM_Start_DMA(pwm_dma_data[dma_id].htim,
                                pwm_dma_data[dma_id].dma_channel,
                                (uint32_t*)(pwm_dma_data[dma_id].inter_dma_data.pwm_buffer),
                                DMA_BUFFER_LEN);
    if ( res != HAL_OK){
        return res;
    }

    if (b_block){ // Block Mode
        while(pwm_dma_data[dma_id].inter_dma_data.b_busy == 1){
            osDelay(1);
        }
    }

    return res;
}

