#ifndef __MAHONY_H
#define __MAHONY_H

#include "math.h"
#include "arm_math.h"
#include "mpu6500.h"

extern float q[4];//四元数
extern float angle[3];//欧拉角yaw偏航角绕z轴，roll横滚角绕y轴，pitch俯仰角绕x轴

void quaternion_init(void);//初始化四元数
void Mahony_update(float dt);//更新四元数
#endif
