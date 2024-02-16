#include "mpu6500.h"

uint8_t tx,rx,id;
struct MPU_DATA mpu_data;

//MPU6500单次写命令
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;					//开始通讯
    tx = reg & 0x7F;			//使第一位为0（写模式）
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);		//写入命令地址
    tx = data;				
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);		//写入数据
    MPU_NSS_HIGH;					//结束通讯
    return 0;
}
//MPU6500单次读取单字节数据
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;		//使地址第一位为1（读模式）
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);	//写入需要读取的地址
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);  //为读取的数据提供存储空间
    MPU_NSS_HIGH;
    return rx;
}

uint8_t mpu6500_init(void)
{
	HAL_Delay(100);
 
	id = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i = 0;
	uint8_t MPU6500_Init_Data[10][2] =
  {{ MPU6500_PWR_MGMT_1, 0x80 },     /* 重置设备*/ 
	 { MPU6500_PWR_MGMT_1, 0x03 },     /* 陀螺仪时钟源设置 */ 
 	 { MPU6500_PWR_MGMT_2, 0x00 },     /* 启动 Acc & Gyro */ 
	 { MPU6500_CONFIG, 0x04 },         /* 低通滤波 频率20Hz */ 
	 { MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
	 { MPU6500_ACCEL_CONFIG, 0x08 },   /* +-4G */ 
   { MPU6500_ACCEL_CONFIG_2, 0x02 }, /* 使能低通滤波器  设置 Acc 低通滤波 */ 
	 { MPU6500_USER_CTRL, 0x20 },};    /* 使能 AUX */ 
//量程设置，二进制3，4位与手册对应
//G：0x00 = ±250dps；0x08= ±500dps 0x10	= ±1000dps 0x18	= ±2000dps 
//A：±2g (0x00), ±4g (0x08), ±8g (0x10), ±16g (0x18) 
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		HAL_Delay(1);
	}
	return 0;
}

float Get_16Bit_Data(uint8_t addr_h, uint8_t addr_l)
{
  static uint8_t buf[2];
  static short data;
	
  buf[0] = mpu_read_byte(addr_l);
  buf[1] = mpu_read_byte(addr_h);
  data = (buf[1]<<8)|buf[0];
	
	return data;   
}

void Get_MPU6500_Data(void)
{
  //加速度数据+-4g
	mpu_data.ax = Get_16Bit_Data(MPU6500_ACCEL_XOUT_H,MPU6500_ACCEL_XOUT_L)*4*g*0.9993 / 32768 + 0.025;
	mpu_data.ay = Get_16Bit_Data(MPU6500_ACCEL_YOUT_H,MPU6500_ACCEL_YOUT_L)*4*g*0.9898 / 32768 - 0.0863;
	mpu_data.az = Get_16Bit_Data(MPU6500_ACCEL_ZOUT_H,MPU6500_ACCEL_ZOUT_L)*4*g*0.9892 / 32768 + 0.4448;
	
	//陀螺仪数据+-2000dps
	mpu_data.gx = Get_16Bit_Data(MPU6500_GYRO_XOUT_H,MPU6500_GYRO_XOUT_L)*2000 / 32768 + 1.679097;//零偏校准
	mpu_data.gy = Get_16Bit_Data(MPU6500_GYRO_YOUT_H,MPU6500_GYRO_YOUT_L)*2000 / 32768 - 2.095563;
	mpu_data.gz = Get_16Bit_Data(MPU6500_GYRO_ZOUT_H,MPU6500_GYRO_ZOUT_L)*2000 / 32768 + 1.244804;
	//温度
	mpu_data.temp = Get_16Bit_Data(MPU6500_TEMP_OUT_H,MPU6500_TEMP_OUT_L)/333.87f +21;

}
