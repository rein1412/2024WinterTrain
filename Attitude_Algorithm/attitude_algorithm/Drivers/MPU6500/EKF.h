#ifndef __EKF_H
#define __EKF_H

#define pi 3.14

#include "math.h"
#include "arm_math.h"
#include "mpu6500.h"

extern float q[4];//四元数
extern float angle[3];//欧拉角yaw偏航角绕z轴，roll横滚角绕y轴，pitch俯仰角绕x轴


#endif
