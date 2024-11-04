/*
 * adxl345.h
 *
 *  Created on: Sep 8, 2024
 *      Author: ggopi
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c1;
#define ADXL_ADD 0x53<<1

extern uint8_t rec_data[6];
extern int16_t x,y,z;
extern float xg,yg,zg;


void adxl_write(uint8_t reg,uint8_t value);
void adxl_read(uint8_t reg,uint8_t num);
void adxl_init();

#endif /* INC_ADXL345_H_ */
