/*
 * tool.h
 *
 *  Created on: 2021年12月6日
 *      Author: guo
 */

#ifndef INC_TOOL_H_
#define INC_TOOL_H_

#ifdef __cplusplus
extern "C" {
#endif

void PutErrorCode(void *pQueuq, uint8_t code_value);

int compute_down_len(char* p);

#ifdef __cplusplus
}
#endif

#endif /* INC_TOOL_H_ */
