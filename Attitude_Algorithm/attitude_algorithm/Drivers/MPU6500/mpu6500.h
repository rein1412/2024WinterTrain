#ifndef __MPU6500_H
#define __MPU6500_H

#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOC,MPU_NSS_Pin,GPIO_PIN_RESET);
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOC,MPU_NSS_Pin,GPIO_PIN_SET);
#define g 9.8

#include "mpu6500_reg_h.h"
#include "stdint.h"
#include "gpio.h"
#include "spi.h"

typedef struct MPU_DATA
{
	float ax,ay,az,gx,gy,gz,temp;
}MPU_DATA;
extern MPU_DATA mpu_data;
extern uint8_t tx,rx,id;

uint8_t mpu6500_init(void);
void Get_MPU6500_Data(void);

#endif
