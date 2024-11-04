/*
 * gsm.c
 *
 *  Created on: Sep 8, 2024
 *      Author: ggopi
 */

#include"gsm.h"
char rexdata[100];
void gsmcmd(const char *cmd){
	HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2,HAL_MAX_DELAY);
	HAL_UART_Receive(&huart2, (uint8_t*)rexdata, 100, 1000);

}
void gsmsnd(const char *ph,const char *msg){
	char command[100];
	gsmcmd("AT+CMGF=1");
	HAL_Delay(1000);
	snprintf(command,sizeof(command),"AT+CMGS=\"%s\"",ph);
	gsmcmd(command);
	HAL_Delay(1000);
	gsmcmd(msg);
	uint8_t cntz=0x1A;
	HAL_UART_Transmit(&huart2, &cntz, 1,HAL_MAX_DELAY);
	HAL_Delay(3000);

}
