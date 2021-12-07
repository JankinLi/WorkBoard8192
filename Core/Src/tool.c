/*
 * tool.c
 *
 *  Created on: 2021年12月6日
 *      Author: LiChuan
 */

#include "main.h"
#include "cmsis_os.h"

void PutErrorCode(void *pQueuq, uint8_t code_value){
	osMessageQueuePut(pQueuq, &code_value, 0U, 0U);
}


int compute_down_len(char* p){
	char * start = p;
	while(1){
		uint8_t value = *p;
		if (value == 0xFD){
			break;
		}
		p = p + 1;
	}

	return (p - start + 1);
}

