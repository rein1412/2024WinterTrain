#include "mpu6500_reg_h.h"
#include "stdint.h"
#include "gpio.h"
#include "spi.h"

#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOC,MPU_NSS_Pin,GPIO_PIN_RESET);
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOC,MPU_NSS_Pin,GPIO_PIN_SET);
#define g 9.8

struct MPU_DATA
{
	float ax,ay,az,gx,gy,gz,temp;
};
 
extern uint8_t tx,rx,id;
extern struct MPU_DATA mpu_data;

uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data);
uint8_t mpu_read_byte(uint8_t const reg);
uint8_t mpu6500_init(void);
float Get_16Bit_Data(uint8_t addr_h, uint8_t addr_l);
void Get_MPU6500_Data(void);
