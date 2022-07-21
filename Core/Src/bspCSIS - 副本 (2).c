/**
	*@file		bspCSIS.c
	*@data		2022.03.28
	*@author	CSIS（Car Soared Into Sky）
	*@brief		Onboard integration support package
	*@logic		
	*		--
	*			
	*@InvoStru
	*			--
	*			-	MotorDriver			电机控制
	*			-	SteMotDriver		舵机控制	
	*			-	ADCData					ADC数据
	*			-	PoorCmpAnd			差比和加权算法系数
	*			-	Key							独立按键[未使用]
	*			-	Kalmam					卡尔曼滤波算法参数
	*			-	TyPID						PID算法参数
	*			--
	*@InvoFunc
	*			--
	*			-	bsp_GloableInit()								板载支持包函数全局初始化函数
	*			-	bsp_iducKalman_Init()						iducKalman初始化
	*			-	bsp_PCA_Init()									PCA(差比和加权算法系数)初始化
	*			-	bsp_StartAndStop_Detection()		干簧管起始与停止检测
	*			-	bsp_LED_FeedBack()							LED程序测试闪烁反馈函数
	*			-	bsp_Kalman_Filter()							卡尔曼滤波（一维）
	*			-	bsp_ADCvalue_Unifor()						ADC数据归一化处理[未使用]
	*			-	bsp_ADCValue_PoorCmpAnd()				差比和加权算法
	*			-	bsp_ADC_Operate()								ADC数据处理函数
	*			-	bsp_Motor_PwmSet()							电机PWM设置函数(调速)[未使用]
	*			-	bsp_SteMot_PwmSet()							舵机PWM设置函数(转向)
	*			-	bsp_PID_Init()									PID初始化
	*			-	bsp_PID_Core()									PID核心算法(位置式)
	*			-	bsp_PID_Control()								PID控制函数
	*			-	fputc()													串口输出重定向
	*			-	bsp_Esp8266_ValueCallBack()			WiFi数据发送函数
	*			-	bsp_Usart_Operate()							串口数据处理函数
	*			-	bsp_Usart_ADCValueCallBack()		ADC采样值串口回返函数
	*			-	bsp_Usart_Receive()							串口接收数据函数
	*			-	bsp_SteMot_MakeZero()						舵机初始化归零
	*			-	bsp_Emac_Operate()							电机IN控制
	*			-	bsp_OLED_Display()							OLED屏显函数
	*			-	HAL_TIM_PeriodElapsedCallback()	定时器中断回调函数
	*			-	HAL_GPIO_EXTI_Callback()				外部中断回调函数
	*			--
	*/
	

/*--------------------- Include头文件声明 ----------------------- */
#include "bspCSIS.h"


/*---------------------- 相关结构体定义 -------------------------- */
/**
	*@name		MotorDriver
	*@type		struct array
	*@about 	电机控制
	*@param
	*		- MotorDriver[0]		电机1
	*		- MotorDriver[1]		电机2
	*		--
	*		-	Onoff			电机启动与停止标志位
	*		- pwmrate		传给Tim -> CRR的值，可改变PWM占空比
	*		- IN1				L298n IN1逻辑控制引脚
	*		- IN2				L298n IN2逻辑控制引脚
	*/
typedef struct{
	
	_Bool				OnOff;
	uint16_t		pwmrate;
	_Bool				IN1;
	_Bool 			IN2;
	
}MotorDriver;


/**
	*@name		SteMotDriver
	*@type		struct
	*@about 	舵机控制
	*@param
	*		- pwmrate		传给Tim -> CRR的值，可改变PWM占空比
	*		- angle			舵机目前角度
	*/
typedef struct{
	
	uint16_t		pwmrate;	
	uint8_t			angle;
	
}SteMotDriver;


/**
	*@name	ADCData
	*@type	struct
	*@about 	ADC数据
	*@param
	*		- origanlData[]		ADC采集到的原始数据
	*		- filterData[]		滤波处理后的数据
	*		- IDUC_L					左电感
	*		-	IDUC_R					右电感
	*		- IDUC_M					中电感
	*		- IDUC_LM					左中电感
	*		- IDUC_RM					右中电感
	*		-	Error						差值
	*/
typedef struct{
	
	__IO	uint16_t		orignalData[5];
	__IO	float				filterData[5];
	__IO	float				IDUC_L;
	__IO	float				IDUC_R;
	__IO	float				IDUC_M;
	__IO	float				IDUC_LM;
	__IO	float				IDUC_RM;
	__IO	float				Error;
	
}ADCData;


/**
	*@name	PoorCmpAnd
	*@type	struct
	*@about 	差比和加权算法系数
	*@param
	*		--
	*		- paramA			控制系数A
	*		- paramB			控制系数B
	*		- paramC			控制系数C
	*		- paramP			比例系数P
 	*/
typedef struct{
	
	float paramA;
	float paramB;
	float paramC;
	float paramP;
	
}PoorCmpAnd;


/**
	*@name		Key
	*@type		struct
	*@about 	独立按键（未使用）
	*@param
	*		- key1		独立按键1
	*		- key2		独立按键2
	*/
struct{
	
	_Bool	key1;
	_Bool	key2;
	
}Key;

/**
	*@name	Kalmam
	*@type	struct
	*@about Kalman Filter 
	*@param
	*		- symStateNow					系统实时状态						X(k)
	* 	- symStatePostFore		系统上次预测状态				X(k|k-1)
	* 	- symStatePostBest		系统上次最优状态				X(k-1|k-1)
	* 	- covNow							本次系统状态协方差			P(k|k)
	* 	- covPostFore					上次预测状态协方差			P(k|k-1)
	*		- covPostBest					上次最优状态协方差			P(k-1|k-1)
	*		- symControl					系统控制量							U(k)
	*		- symParmA						系统参数A							A
	*		- symParmB						系统参数B							B
	* 	- errorMes						k时刻测量值						Z(k)
	* 	- mesParm							测量系统的参数					H
	* 	- pcesNoise						过程噪声								W(k)
	* 	- mesNoise						测量噪声								V(k)
	*		- transposeA					A的转置矩阵						A'
	*		- transposeQ					W(k)的转置矩阵					Q	
	*		- transposeR					V(k)的转置矩阵					R
	*		- transposeH					H的转置矩阵						H'
	*		- gain								卡尔曼增益							Kg
	*/
typedef struct{
	__IO	float symStateNow[5];
	__IO	float symStatePostFore[5];
	__IO	float	symStatePostBest[5];
	__IO	float covNow[5];
	__IO	float	covPostFore[5];
	__IO	float covPostBest[5];
				float symControl;
				float symParmA;
				float symParmB;
	__IO	float errorMes;
				float	mesParm;
				float pcesNoise;
				float mesNoise;	
				float transposeA;	
				float	transposeQ;
				float transposeR;
				float transposeH;
	__IO	float gain[5];
}Kalman;


/**
	*@name		TyPID
	*@type		struct
	*@about 	PID控制系数
	*@param
	*		--
	*		-	Err								自变量
	*		-	ErrLastValue[]		前几次Err值
	*		-	ErrLastSum				Err累加值
	*		- Proportion				比例
	*		- Integral					积分
	*		- Differential			微分
	*/
typedef struct{
	
	float Err;
	float	ErrLastValue[3];
	float	ErrLastSum;
	int 	Proportion;
	int 	Integral;
	int 	Differential;
	int		Integral_Max;
	
}TyPID;

/*------------------------ 相关变量定义 --------------------------- */

Kalman iducKalman;			//卡尔曼滤波
PoorCmpAnd PCA;					//差比和加权算法系数
MotorDriver Motor[2];		//电机
SteMotDriver SteMot;		//舵机
ADCData adcData;				//ADC采样
TyPID PID;							//舵机PID
TyPID MotorPID;					//电机PID（闲置）



/*-------------------------- 函数定义 ----------------------------- */

/**
	*@funcname		bsp_GloableInit()
	*@brief 			板载支持包函数全局初始化
	*/
void bsp_GloableInit(void)
{
	//OLED初始化
	
	OLED_Init();
	OLED_On();
	OLED_Clear();
	
	
	//ADC DMA 模式启动
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcData.orignalData, 5);
	
	//舵机PWM开启
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	
	
	//电机PWM开启
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	
	
	//定时器3启动
	HAL_TIM_Base_Start_IT(&htim3); 
	HAL_TIM_Base_Start(&htim3);
 
	
	//Kalman初始化
	bsp_iducKalman_Init();
	
	//PCA初始化
	bsp_PCA_Init();
	
	//PID初始化
	bsp_PID_Init();
	
	//电机控制
	bsp_Emac_Operate();
	
	//舵机初始化归零
	bsp_SteMot_MakeZero();
}



/**
	*@funcname		bsp_iducKalman_Init()
	*@brief 			iducKalman初始化
	*@param
	*			--
	*				- X(k|k)	10.0
	*				- P(k|k)	10.0
	*				- A				1.0			
	*				- A'			1.0
	*				- U(k)		0.0
	*				- H				1.0
	*				- H'			1.0
	*				- Q				0.5
	*				- R				0.5
	*				-	Kg			1.0
	*/
/*************************** 待调参 *****************************/
void bsp_iducKalman_Init(void)
{
	uint8_t _i;
	
	for(_i = 0; _i < 5; _i++)
		iducKalman.symStateNow[_i] = 10;
	for(_i = 0; _i < 5; _i++)
		iducKalman.covNow[_i] = 10;
	
	iducKalman.symParmA = 1;
	iducKalman.transposeA = 1;
	iducKalman.symControl = 0;
	iducKalman.mesParm = 1;
	iducKalman.transposeH = 1;
	iducKalman.transposeQ = 0.5;
	iducKalman.transposeR = 0.5;
	for(_i = 0; _i < 5; _i++)
		iducKalman.gain[_i] = 1.0;
} 


/**
	*@funcname		bsp_PCA_Init()
	*@brief 			PCA(差比和加权算法系数)初始化
	*@param
	*			--
	*				- A			1
	*				- B			1
	*				- C			1
	*				- P			0.5
	*/
/*************************** 待调参 *****************************/
void bsp_PCA_Init(void)
{
	PCA.paramA = 1;
	PCA.paramB = 1;
	PCA.paramC = 1;
	PCA.paramP = 0.5;
}



/**
	*@funcname		bsp_StartAndStop_Detection()
	*@brief 			干簧管起始与停止检测
	*/
void bsp_StartAndStop_Detection(void)
{
	if(HAL_GPIO_ReadPin(GPIOB, GanHuang_Pin) == GPIO_PIN_SET)
			Motor->OnOff = 0;
	else
			Motor->OnOff = 1;
}




/**
	*@funcname		bsp_LED_FeedBack()
	*@brief 			LED程序测试闪烁反馈函数
	*/
void bsp_LED_FeedBack(void)
{
	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
}






/**
	*@funcname		bsp_Kalman_Filter()
	*@brief 			卡尔曼滤波（一维）
	*/
/*************************** 待核对 *****************************/
float bsp_Kalman_Filter(uint16_t data, uint8_t _i)
{
	iducKalman.errorMes = (float)data;
	
	/*		X(k|k-1)=A X(k-1|k-1)+B U(k)	 */
	iducKalman.symStatePostFore[_i] = iducKalman.symParmA * iducKalman.symStatePostBest[_i] +
																iducKalman.symParmB * iducKalman.symControl;
	
	/* 		P(k|k-1)=A P(k-1|k-1) A’+Q		*/
	iducKalman.covPostFore[_i] = 	iducKalman.symParmA * iducKalman.covPostBest[_i] * 
														iducKalman.transposeA +iducKalman.transposeQ;
	
	/* 		X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))		*/
	iducKalman.symStateNow[_i] = 	iducKalman.symStatePostFore[_i] + iducKalman.gain[_i] * 
														(iducKalman.errorMes - iducKalman.mesParm * iducKalman.symStatePostFore[_i]);
	
	/* 		Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)		*/
	iducKalman.gain[_i] = (iducKalman.covPostFore[_i] * iducKalman.transposeH) / 
										(
											iducKalman.mesParm * iducKalman.covPostFore[_i] *
											iducKalman.transposeH + iducKalman.transposeR );
	
	/*		P(k|k)=（I-Kg(k) H）P(k|k-1)		*/
	iducKalman.covNow[_i] = (1 - iducKalman.gain[_i] * iducKalman.mesParm) * iducKalman.covPostFore[_i];
	
	/*		X(k-1|k-1) = X(k|k) 	*/
	iducKalman.symStatePostBest[_i] = iducKalman.symStateNow[_i];
	
	/*		P(k-1|k-1) = P(k|k)		*/
	iducKalman.covPostBest[_i] = iducKalman.covNow[_i];
	
	return iducKalman.symStateNow[_i];
}







/**
	*@funcname		bsp_ADCvalue_Unifor()
	*@brief 			ADC数据归一化处理(未使用)
	*/
float bsp_ADCvalue_Unifor(float value)
{
	/* 限幅 */
	if 			(value > ADCVALUEMAX) value = ADCVALUEMAX;
	else if (value < ADCVALUEMIN)	value = ADCVALUEMIN;
	/* 归一化 */
	value = 100 * ((value - ADCVALUEMIN)/(ADCVALUEMAX - ADCVALUEMIN));
	return value;
}

/**
	*@funcname		bsp_ADCValue_PoorCmpAnd()
	*@brief 			差比和加权算法
	*/
float bsp_ADCValue_PoorCmpAnd(ADCData value)
{
	float Err;
	
	Err = (
					(	PCA.paramA * (adcData.IDUC_L - adcData.IDUC_R) +
						PCA.paramB * (adcData.IDUC_LM - adcData.IDUC_RM)) /
					(
						(	PCA.paramA * (adcData.IDUC_L + adcData.IDUC_R)) +
						(	PCA.paramC * (fabs((double)(adcData.IDUC_LM - adcData.IDUC_RM))))
					)
				) * PCA.paramP;
						
	return Err;
}



/**
	*@funcname		bsp_ADC_Operate()
	*@brief 			ADC数据处理函数
	*@logic
	*		--
	*		-	通道滤波 -> 差比和加权算法 -> Err
	*/
/*************************** 待完善 *****************************/
void bsp_ADC_Operate(void)
{
	uint8_t time;
	
	//通道滤波
	for(time = 0; time < 5; time++)
	{
		adcData.filterData[time] = bsp_Kalman_Filter(adcData.orignalData[time], time);
	}
	
	//赋值
	adcData.IDUC_L 	= adcData.filterData[0];
	adcData.IDUC_LM = adcData.filterData[1];
	adcData.IDUC_M 	= adcData.filterData[2];
	adcData.IDUC_RM = adcData.filterData[3];
	adcData.IDUC_R 	= adcData.filterData[4];
	
	//计算Err
	adcData.Error = bsp_ADCValue_PoorCmpAnd(adcData);
	
}


/**
	*@funcname		bsp_Motor_PwmSet()
	*@brief 			电机PWM设置函数(调速)
	*/
void bsp_Motor_PwmSet(MotorDriver *MD)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)MD[0].pwmrate);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint16_t)MD[1].pwmrate);
}




/**
	*@funcname		bsp_SteMot_PwmSet()
	*@brief 			舵机PWM设置函数(转向)
	*/
void bsp_SteMot_PwmSet(void)
{
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, (uint16_t)SteMot.pwmrate);
}



/**************************************************************************************************
***************************************************************************************************/
/**
	*@funcname		bsp_PID_Init()
	*@brief 			PID初始化
	*@param
	*			--
	*				- P			1
	*				- I			1
	*				- D			1
	*/
/*************************** 待调参 *****************************/
void bsp_PID_Init(void)
{
	PID.Proportion = 10;
	PID.Integral = 15;
	PID.Differential = 30;
	
	PID.ErrLastSum = 0;
	PID.ErrLastValue[0] = 0.0;
	PID.ErrLastValue[1] = 0.0;
	PID.ErrLastValue[2] = 0.0;
}


/**
	*@funcname		bsp_PID_Core()
	*@brief 			PID核心算法(位置式)
	*/
uint16_t bsp_PID_Core(float error)
{
	PID.Err = error;
	float PwmRate;		//PwmRate为PID算法结束后得到的PWM控制值
	
	PID.ErrLastValue[2] = PID.ErrLastValue[1];
	PID.ErrLastValue[1] = PID.ErrLastValue[0];
	PID.ErrLastValue[0] = error;
	
	if ((PID.ErrLastSum + error) < PID.Integral_Max && (PID.ErrLastSum + error) > -PID.Integral_Max)//限幅判断，用abs()取绝对值可能会丢失精度导致意外的超限而卡死
	{
		PID.ErrLastSum += error;			//在幅度内就加上
	}
	else if (PID.ErrLastSum > 0)		//超限了但是是正的
	{
		PID.ErrLastSum = PID.Integral_Max;
	}
	else if (PID.ErrLastSum < 0)		//超限了但是是负的
	{
		PID.ErrLastSum = -PID.Integral_Max;
	}
	
	/***************************** 待调整 *****************************/
	PwmRate = PID.Proportion * (error) + PID.Integral * PID.ErrLastSum + 
						PID.Differential * ((PID.ErrLastValue[0] - PID.ErrLastValue[1]) -
					(	PID.ErrLastValue[1] - PID.ErrLastValue[2]));
	

	return PwmRate;
}


/**
	*@funcname		bsp_PID_Control()
	*@brief 			PID控制函数
	*/
void bsp_PID_Control(void)
{
	bsp_ADC_Operate();
	SteMot.pwmrate = bsp_PID_Core(adcData.Error);
	bsp_SteMot_PwmSet();
	/*
			预留代码区
			舵机角度计算 
			SteMot->angle
	*/
}

/**************************************************************************************************
***************************************************************************************************/
/**************************************标志异常映射表（可选）****************************************/

/**
	*@funcname		
	*@brief 			环岛
	*/




/**
	*@funcname		
	*@brief 			十字
	*/




/**
	*@funcname		
	*@brief 			环形
	*/



/**************************************************************************************************
***************************************************************************************************/


/**
	*@funcname		fputc()
	*@brief 			串口输出重定向
	*/
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}



/**
	*@funcname		bsp_Esp8266_ValueCallBack()
	*@brief 			WiFi数据发送函数
	*		--
	*		-					这里printf("%f",error);就可以把error值通过串口传出去，
	*		-					经过esp8266转发到vofa解析，格式选择firewater如果需要
	*		-					比较与”设定值“差距，或者展示其他数据，请一起在此处
	*		-					写到printf里，格式为printf(通道1,通道2,通道3,...);都
	*		-					是float类型
	*/
void bsp_Esp8266_ValueCallBack(void)
{
	bsp_PID_Control();
	printf("Or:%.0f %.0f %.0f %.0f %f.0\n", (float)adcData.orignalData[0], (float)adcData.orignalData[1], 
																					(float)adcData.orignalData[2], (float)adcData.orignalData[3], 
																					(float)adcData.orignalData[4]);
	printf("Fi:%.0f %.0f %.0f %.0f %f.0\n", (float)adcData.filterData[0], (float)adcData.filterData[1], 
																					(float)adcData.filterData[2], (float)adcData.filterData[3], 
																					(float)adcData.filterData[4]);
}



/**
	*@funcname		bsp_Usart_Operate()
	*@brief 			串口数据处理函数
	*/
void bsp_Usart_Operate(uint8_t *str)
{
	int value;
	uint8_t oder = 0;
	
	if			((*str == 'P') && (sscanf((const char *)str, "P:%d", &value) == 1))		oder = 1;
	else if ((*str == 'I') && (sscanf((const char *)str, "I:%d", &value) == 1))		oder = 2;
	else if ((*str == 'D') && (sscanf((const char *)str, "D:%d", &value) == 1))		oder = 3;
	else		printf("\nPID value set fail!");
	
	switch(oder)
	{
		case 1: 
						PID.Proportion = value;	
						printf("\nSuccessful set the value of P: %d", PID.Proportion);
						break;
		case 2: 
						PID.Integral = value;	
						printf("\nSuccessful set the value of I: %d", PID.Integral); 
						break;
		case 3:
						PID.Differential = value;	
						printf("\nSuccessful set the value of D: %d", PID.Differential); 
						break;
		default:	break;
	}

}







/**
	*@funcname		bsp_Usart_ADCValueCallBack()
	*@brief 			ADC采样值串口回返函数
	*/
void bsp_Usart_ADCValueCallBack(void)
{
	
	uint8_t times;
	/*
	bsp_PID_Control();
	printf("\n\nThe origanl data:");
	for(times = 0; times < 5; times++){
		printf("\nChannel %d: %d", times, adcData.orignalData[times]);
	}
	printf("\n\nThe filter data:");
	for(times = 0; times < 5; times++){
		printf("\nChannel %d: %.3f", times, adcData.filterData[times]);
	}
	printf("\n\nThe Error data:");
	printf("\nChannel %d: %.3f", times, adcData.Error);
	
	printf("\nSteMotor PWM Count: %d", SteMot.pwmrate);
	*/

	/* 赛道电磁感采样专用 */
	
	printf("\nCollection value:\n");
	for(times = 0; times < 5; times++){
		printf("%6d   ", adcData.orignalData[times]);
	}
	printf("\n");
	for(times = 0; times < 5;  times++){
		printf("%6d   ", (int)adcData.filterData[times]);
	}
	printf("\n");
	printf("%.3f", adcData.Error);
	printf("\n\n");
	
} 
 






/**
	*@funcname		bsp_Usart_Receive()
	*@brief 			串口接收数据函数
	*/
void bsp_Usart_Receive(void)
{
	if(recv_end_flag ==1)			
		{	
			//bsp_LED_FeedBack();
			bsp_Usart_Operate(rx_buffer);
			for(uint8_t i=0;i<rx_len;i++)
			{
				rx_buffer[i]=0;
			}
			rx_len=0;
			recv_end_flag=0;
		}
		HAL_UART_Receive_DMA(&huart1,rx_buffer,200);	
}


/**
	*@funcname		bsp_SteMot_MakeZero()
	*@brief 			舵机初始化归零
	*/
void bsp_SteMot_MakeZero(void)
{
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 1000);
}



/**
	*@funcname		bsp_Emac_Operate()
	*@brief 			电机IN控制
	*/
/*************************** 待完善 *****************************/
void bsp_Emac_Operate(void)
{
	//bsp_StartAndStop_Detection();
	//if(Motor->OnOff == 1)
	//{
		//电机1正转
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		//电机2正转
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	//}
	
	/*else
	{
		//电机1停止
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		//电机2停止
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	}*/
}







/**
	*@funcname		bsp_OLED_Display()
	*@brief 			OLED屏显函数
	*@count
	*		--
	*		-					PID数值
	*		-					ADC原始采集值
	*		-					Err
	*/
void bsp_OLED_Display(void)
{
	bsp_PID_Control();
	
	OLED_ShowString(0, 0, (uint8_t *)"P:", 12);
	OLED_ShowNum(15, 0, PID.Proportion, 3, 12);
	OLED_ShowString(40, 0, (uint8_t *)"I:", 12);
	OLED_ShowNum(55, 0, PID.Integral, 3, 12);
	OLED_ShowString(80, 0, (uint8_t *)"D:", 12);
	OLED_ShowNum(95, 0, PID.Differential, 3, 12);	
	
	OLED_ShowString(0, 2, (uint8_t *)"ADC OrValue:", 12);
	OLED_ShowNum(0, 3, adcData.orignalData[0], 4, 12);
	OLED_ShowNum(50, 3, adcData.orignalData[1], 4, 12);
	OLED_ShowNum(100, 3, adcData.orignalData[2], 4, 12);
	OLED_ShowNum(0, 4, adcData.orignalData[3], 4, 12);
	OLED_ShowNum(50, 4, adcData.orignalData[4], 4, 12);
	
	OLED_ShowString(0, 6, (uint8_t *)"Err:", 12);
	OLED_ShowUnFloat(30, 6, adcData.Error, 6, 3, 12);
	
	OLED_ShowString(0, 7, (uint8_t *)"PwmRate:", 12);
	OLED_ShowUnFloat(60, 7, SteMot.pwmrate, 6, 2, 12);
	
}







/**
	*@brief 	定时器中断回调函数
	*/
/********************* 定时器频率待确定 ******************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		bsp_PID_Control();
		bsp_Usart_ADCValueCallBack();
	}
}








/**
  * @brief  外部中断回调函数
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if (GPIO_Pin == Key1_Pin)
	{
		bsp_Usart_ADCValueCallBack();
		while(!HAL_GPIO_ReadPin(GPIOA, Key1_Pin));
	}
	else if (GPIO_Pin == Key2_Pin)
	{
		if(Motor->OnOff==0)
			Motor->OnOff = 1;
		else
			Motor->OnOff = 0;
		while(!HAL_GPIO_ReadPin(GPIOA, Key2_Pin));
	}
	
}

