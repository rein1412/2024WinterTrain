/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_dwt.h"
#include "motor_simulation.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

#define PI 3.14159265
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t DWT_CNT;
float dt,T;
motorObject_t Motor;//创建电机对象
struct value{
	float current;
	float velocity;
	float last_velocity;
	float angle;
	float last_angle;
	}real;//电机参数
//速度闭环
float velocity_aim,velocity_input,voltage_input;
//角度闭环
float angle_aim,angle_input,voltage_input;
//单级串口设置
//uint8_t receive_data[30];
//char str[31];
//串级串口设置
uint8_t receive_data[60];
char str[61];
int len;
//float pid[3];//存放Kp,Ki,Kd,
float pid[6];//存放内外环Kp,Ki,Kd,
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//速度闭环
/*
float PID(float pid[],float velocity_aim,struct value *real,motorObject_t *motor)
{
//	float velocity_input_p,velocity_input_i,velocity_input_d;//储存pid三项的输出
	float dv = velocity_aim-real->velocity;
	//p项
	velocity_input_p = pid[0]*dv;
	//i项
	velocity_input_i += pid[1]*dv;
	//d项
	velocity_input_d = pid[2]*(real->velocity-real->last_velocity);
	if(velocity_input_p+velocity_input_i+velocity_input_d <= motor->maxU)
	{
		return velocity_input_p+velocity_input_i+velocity_input_d;
	}
	else
	{
		return motor->maxU;
	}
}
*/

//角度闭环
/*
float PID(float pid[],float angle_aim,struct value *real,motorObject_t *motor)
{
	float angle_input_p,angle_input_i,angle_input_d;//储存pid三项的输出 
  float da = angle_aim-real->angle;
	//P项
 	angle_input_p = pid[0]*da;
	//I项
	angle_input_i += pid[1]*da;
	//d项
 	angle_input_d = pid[2]*(real->angle-real->last_angle);
 	if(angle_input_p+angle_input_i+angle_input_d <= motor->maxU)
	{
		return angle_input_p+angle_input_i+angle_input_d;
	}
	else
	{
		return motor->maxU;
	}
}
*/

//串级角度闭环

//内环
float vPID(float pid[],float velocity_aim,struct value *real,motorObject_t *motor)
{
	float velocity_input_p,velocity_input_i,velocity_input_d;//储存pid三项的输出
	float dv = velocity_aim-real->velocity;
	//P项
	velocity_input_p = pid[3]*dv;
	//I项
	velocity_input_i += pid[4]*dv;
	//d项
	velocity_input_d = pid[5]*(real->velocity-real->last_velocity);
	if(velocity_input_p+velocity_input_i+velocity_input_d <= motor->maxU)
	{
		return velocity_input_p+velocity_input_i+velocity_input_d;
	}
	else
	{
		return motor->maxU;
	}
}
//外环
float aPID(float pid[],float angle_aim,struct value *real,motorObject_t *motor)
{
	float angle_input_p,angle_input_i,angle_input_d;//储存pid三项的输出 
  float da = angle_aim-real->angle;
	float out;
	//P项
	angle_input_p = pid[0]*da;
	//I项
//  if(real->angle>6)//积分分离
//	{
	angle_input_i += pid[1]*da;
//	}
//	else
//	{
//		angle_input_i = 0;
//	}
	//d项
	angle_input_d = pid[2]*(real->angle-real->last_angle);
	velocity_aim = angle_input_p+angle_input_i+angle_input_d;
	out = vPID(pid,velocity_aim,real,&Motor);
	return out;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	DWT_Init(72);//dwt外设主频72MHz
	Motor_Object_Init(&Motor);//初始化电机
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, receive_data, 100);//DMA设置
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		dt = DWT_GetDeltaT(&DWT_CNT);//定义dt
		T += dt;//获取时间
		real.current = Get_Motor_Current(&Motor);
		real.velocity = Get_Motor_Velocity(&Motor);
		real.last_velocity = Motor.lastVelocity;
		real.angle = Get_Motor_Angle(&Motor);//测量电机当前状态
		
		//目标曲线建立区域
		//v阶跃

		if(T<3)
		{
			velocity_aim=0;
		}
		else
		{
			velocity_aim=10;
		}

		//v正弦
//		velocity_aim=7.07*sin(T);
		//a阶跃
/*
		if(T<3)
		{
			angle_aim = 0;
		}
		else
		{
			angle_aim = 2*PI;
		}
*/
		//a频率
//		angle_aim=1.414*PI*sin(T);
		//a抗干扰

		angle_aim=0;
		if(T<3)
		{
			voltage_input = aPID(pid,angle_aim,&real,&Motor);
		}
		else
		{
			voltage_input = 10+aPID(pid,angle_aim,&real,&Motor);
		}

		
//		voltage_input = PID(pid,velocity_aim,&real,&Motor);//单级v
//		voltage_input = PID(pid,angle_aim,&real,&Motor);//单级a
//		voltage_input = aPID(pid,angle_aim,&real,&Motor);
		real.last_angle=real.angle;
		Motor_Simulation(&Motor,voltage_input,dt);
    /* USER CODE END WHILE */
                                                                                                                                                                            
    /* USER CODE BEGIN 3 */
		HAL_Delay(1);
//		printf("%f,%f\n",velocity_aim,real.velocity);//打印速度波形
		printf("%f,%f\n",angle_aim,real.angle);//打印角度波形
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
 if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE))
 {	
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  __HAL_DMA_CLEAR_FLAG(huart,DMA_FLAG_TC5);
	HAL_UART_DMAStop(huart);
  int len=100-__HAL_DMA_GET_COUNTER(huart->hdmarx);
	 HAL_UART_Receive_DMA(&huart1,receive_data,100);//不定长DMA接收
	 for(int i = 0; i < sizeof(receive_data); i++) 
	 {
        sprintf(&str[i], "%c", receive_data[i]);
   }
//	 str[30] = '\0';//单级
	 str[60] = '\0';
//	 sscanf((char *)receive_data, "Kp=%f,Ki=%f,Kd=%f", &pid[0], &pid[1], &pid[2]);//将接收的数据拼接为str再用sscanf处理
	 sscanf((char *)receive_data, "Kp=%f,Ki=%f,Kd=%f,Kp=%f,Ki=%f,Kd=%f", &pid[0], &pid[1], &pid[2],&pid[3], &pid[4], &pid[5]);//将接收的数据拼接为str再用sscanf处理
 }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
