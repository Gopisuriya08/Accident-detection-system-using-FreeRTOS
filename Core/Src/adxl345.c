/*
 * adxl345.c
 *
 *  Created on: Sep 8, 2024
 *      Author: ggopi
 */

#include "adxl345.h"


void adxl_write(uint8_t reg,uint8_t value){
	uint8_t data[2];
	data[0]=reg;
	data[1]=value;
	HAL_I2C_Master_Transmit(&hi2c1,ADXL_ADD,data, 2, 10);
}
void adxl_read(uint8_t reg,uint8_t num){
	HAL_I2C_Mem_Read(&hi2c1, ADXL_ADD, reg, 1,rec_data, num, 100);
}
void adxl_init(){
	//adxl_read(0x00,1);
	adxl_write(0x2d,0);//reset all bits
	adxl_write(0x2d, 0x08);// set measure bit 1,all the bit reset
	adxl_write(0x31, 0x01);//data format +-4g
}
