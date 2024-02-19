#include "mahony.h"

//初始欧拉角，旋转顺序zxy
//angle[]欧拉角,yaw偏航角绕z轴，roll横滚角绕y轴，pitch俯仰角绕x轴

volatile float twoKp = 2.0f * 10.0f;											// 2 * Kp
volatile float twoKi = 2.0f * 0.01f;											// 2 * Ki
float q[4];//四元数
float angle[3];//欧拉角yaw偏航角绕z轴，roll横滚角绕y轴，pitch俯仰角绕x轴

void quaternion_norm(void)//四元数归一化
{
	static float tem;
	tem = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] = q[0] / tem;
	q[1] = q[1] / tem;
	q[2] = q[2] / tem;
	q[3] = q[3] / tem;
}

void quaternion_init(void)//初始化四元数
{
	angle[0] = 0;//初始偏航角无法获得，定义为0
	angle[1] = 180*atan(mpu_data.ay / mpu_data.az) / pi;
	angle[2] = 180*asin(mpu_data.ay / sqrt(mpu_data.ax*mpu_data.ax + mpu_data.ay*mpu_data.ay + mpu_data.az*mpu_data.az)) / pi;
	q[0] = cos(angle[0]/2)*cos(angle[1]/2)*cos(angle[2]/2) - sin(angle[0]/2)*sin(angle[1]/2)*sin(angle[2]/2);
	q[1] = cos(angle[1]/2)*cos(angle[0]/2)*sin(angle[2]/2) - cos(angle[2]/2)*sin(angle[1]/2)*sin(angle[0]/2);
	q[2] = cos(angle[2]/2)*cos(angle[0]/2)*sin(angle[1]/2) - cos(angle[1]/2)*sin(angle[2]/2)*sin(angle[0]/2);
	q[3] = cos(angle[2]/2)*cos(angle[1]/2)*sin(angle[0]/2) - cos(angle[0]/2)*sin(angle[2]/2)*sin(angle[1]/2);
	quaternion_norm();
}

void acc_norm(void)//加速度归一化
{
	static float tem;
	tem = sqrt(mpu_data.ax*mpu_data.ax + mpu_data.ay*mpu_data.ay + mpu_data.az*mpu_data.az);
	mpu_data.ax = mpu_data.ax / tem;
	mpu_data.ay = mpu_data.ay / tem;
	mpu_data.az = mpu_data.az / tem;
}

void qua_euler(void)//四元数反解欧拉角
{
	angle[0] = 180*atan((2*q[0]*q[3]-2*q[1]*q[2]) / (q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3])) / pi;//yew
	angle[1] = 180*atan((2*q[0]*q[2]-2*q[1]*q[3]) / (q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])) / pi;//roll
	angle[2] = 180*asin(2*q[2]*q[3]+2*q[0]*q[1]) / pi;                                              //pitch
}

void Mahony_update(float dt)//更新四元数，dt为采样时间，单位s
{
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// Ki积分项

  // 只在加速度计有数据时才进行运算
	if(!((mpu_data.ax == 0.0f) && (mpu_data.ay == 0.0f) && (mpu_data.az == 0.0f)))
	{
		// 将加速度计得到的实际重力加速度向量v归一化
		acc_norm();
    // 通过四元数得到理论重力加速度向量g 
    // 注意，这里实际上是矩阵第三列*1/2
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	  // 对实际重力加速度向量v与理论重力加速度向量g做外积
		halfex = (mpu_data.ay * halfvz - mpu_data.az * halfvy);
		halfey = (mpu_data.az * halfvx - mpu_data.ax * halfvz);
		halfez = (mpu_data.ax * halfvy - mpu_data.ay * halfvx);
		// 在PI补偿器中有积分项的情况下计算并应用积分项
		if(twoKi > 0.0f) 
		{
      // 积分过程
			integralFBx += twoKi * halfex * dt;	
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
            
      // 应用误差补偿中的积分项
			mpu_data.gx += integralFBx;	
			mpu_data.gy += integralFBy;
			mpu_data.gz += integralFBz;
		}
		else
		{
      // 避免为负值的Ki时积分异常饱和（积分分离）
			integralFBx = 0.0f;	
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
		// 应用误差补偿中的比例项
		mpu_data.gx += twoKp * halfex;
		mpu_data.gy += twoKp * halfey;
		mpu_data.gz += twoKp * halfez;
	}
	
	// 微分方程迭代求解
	mpu_data.gx *= (0.5f * 1.0f * dt);
	mpu_data.gy *= (0.5f * 1.0f * dt);
	mpu_data.gz *= (0.5f * 1.0f * dt);
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * mpu_data.gx - qc * mpu_data.gy - q[3] * mpu_data.gz); 
	q[1] += (qa * mpu_data.gx + qc * mpu_data.gz - q[3] * mpu_data.gy);
	q[2] += (qa * mpu_data.gy - qb * mpu_data.gz + q[3] * mpu_data.gx);
	q[3] += (qa * mpu_data.gz + qb * mpu_data.gy - qc * mpu_data.gx); 
	
  // 单位化四元数 保证四元数在迭代过程中保持单位性质
	quaternion_norm();
  //四元数反解欧拉角
	qua_euler();
}
