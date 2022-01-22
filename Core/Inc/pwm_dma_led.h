/*
 * pwm_dma_led.h
 *
 *  Created on: 2022/1/16
 *      Author: duan zhiqiang
 */

#ifndef INC_PWM_DMA_LED_H_
#define INC_PWM_DMA_LED_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define DUTY_0  1 // 1/5  // 26% of 75  WS2812 = 32%
#define DUTY_1  3 // 3/5  // 75% of 75  WS2812 = 62%
#define DUTY_RESET 0
#define DUTY_HIGHT 5 // 100% 5/5

#define TRST_BYTE_LEN      8               //8*8 = 64 bit * 1.25 = 80us
//#define TRST_LEN            TRST_BYTE_LEN*8 // 64*1.25 = 80us Two LED Color Bit(2x24)
#define DMA_BUFFER_LED_NUM 8  //Use 2 LED for DMA double in circular mode
#define DMA_BUFFER_HALF_LEN DMA_BUFFER_LED_NUM*12 // the length of half buffer
#define DMA_BUFFER_LEN  DMA_BUFFER_HALF_LEN*2

#define DMA_TYPE  uint8_t //Use Half mode for DMA Memory data
typedef enum {
    PWM_DMA_HEAD_RST = 0,
    PWM_DMA_LED_SET,
    PWM_DMA_END_RST,
    PWM_DMA_END
}PWM_DMA_STATUS;

#define PWM_DMA_OK  0
#define PWM_DMA_ERROR_INIT  0x01 //DMA not initialized
#define PWM_DMA_ERROR_BUSY  0x02 //DMA is busy
#define PWM_DMA_ERROR_IDX   0x03 //DMA index is error


typedef  struct _pwm_dma_data_struct PWM_DMA_DATA_STRUCT;

typedef struct{
    uint32_t    b_busy;  // 0: "DMA is not working", 1:"DMA is busy"
    uint8_t*    p_buffer;
    int32_t     cur_pos;
    __attribute__((aligned(4))) volatile  DMA_TYPE pwm_buffer[DMA_BUFFER_LEN];
}INTER_PWM_DMA_STRUCT;

struct _pwm_dma_data_struct {
    TIM_HandleTypeDef*      htim;
    uint32_t                dma_channel;
    uint8_t                 *p_dma_colors;
    uint32_t                total_leds;
    void (*send_finishedCallback)(PWM_DMA_DATA_STRUCT *p_pwm_dma_data);
    INTER_PWM_DMA_STRUCT    inter_dma_data;
};

#define PWM_LED_CHANNEL_MAX_COUNT  4 // The max count of DMA Channel which is used to control different LED IC.

void pwm_dma_init(uint32_t dma_id, TIM_HandleTypeDef *htim, uint32_t channel,
                 uint8_t* p_colors, uint32_t leds_count, void *p_callback );
int pwm_dma_send(uint32_t dma_id,uint8_t b_block);

void led_data_fill(PWM_DMA_DATA_STRUCT* p_dma_data,uint8_t b_half);

#ifdef __cplusplus
}
#endif

#endif /* INC_PWM_DMA_LED_H_ */
