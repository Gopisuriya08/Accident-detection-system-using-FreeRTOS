/*
 * gsm.h
 *
 *  Created on: Sep 8, 2024
 *      Author: ggopi
 */

#ifndef INC_GSM_H_
#define INC_GSM_H_

#include "main.h"
#include "string.h"
#include "stdio.h"
extern UART_HandleTypeDef huart2;
void gsmcmd(const char *cmd);
void gsmsnd(const char *ph,const char *msg);

#endif /* INC_GSM_H_ */
