#include "EKF.h"

//初始欧拉角，旋转顺序zxy
//angle[]欧拉角yaw偏航角绕z轴，roll横滚角绕y轴，pitch俯仰角绕x轴

void quaternion_init(MPU_DATA *mpu_data)//初始化四元数
{
	angle[0] = 0;//初始偏航角无法获得，定义为0
	angle[1] = (180*atan(mpu_data->ay / mpu_data->az)) / pi;
	angle[2] = (180*asin(mpu_data->ay / sqrt(mpu_data->ax*mpu_data->ax + mpu_data->ay*mpu_data->ay + mpu_data->az*mpu_data->az))) / pi;
	q[0] = cos(angle[0]/2)*cos(angle[1]/2)*cos(angle[2]/2) - sin(angle[0]/2)*sin(angle[1]/2)*sin(angle[2]/2);
	q[1] = cos(angle[1]/2)*cos(angle[0]/2)*sin(angle[2]/2) - cos(angle[2]/2)*sin(angle[1]/2)*sin(angle[0]/2);
	q[2] = cos(angle[2]/2)*cos(angle[0]/2)*sin(angle[1]/2) - cos(angle[1]/2)*sin(angle[2]/2)*sin(angle[0]/2);
	q[3] = cos(angle[2]/2)*cos(angle[1]/2)*sin(angle[0]/2) - cos(angle[0]/2)*sin(angle[2]/2)*sin(angle[1]/2);
}

void quaternion_norm()//四元数归一化
{
	static float tem;
	tem = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
	q[0] = q[0] / tem;
	q[1] = q[1] / tem;
	q[2] = q[2] / tem;
	q[3] = q[3] / tem;
}
