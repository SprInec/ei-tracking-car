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
	*			-	bsp_Usart_Operate()							串口数据处理函数
	*			-	bsp_Usart_ADCValueCallBack()		ADC采样值串口回返函数
	*			-	bsp_Usart_Receive()							串口接收数据函数
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
	*		-	MotorDriver[0]		电机1
	*		-	MotorDriver[1]		电机2
	*		--
	*		-	Onoff			电机启动与停止标志位
	*		-	pwmrate		传给Tim -> CRR的值，可改变PWM占空比
	*		-	IN1				L298n IN1逻辑控制引脚
	*		-	IN2				L298n IN2逻辑控制引脚
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
	*		-	pwmrate		传给Tim -> CRR的值，可改变PWM占空比
	*		-	angle			舵机目前角度
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
	*		-	origanlData[]		ADC采集到的原始数据
	*		-	filterData[]		滤波处理后的数据
	*		-	IDUC_L					左电感
	*		-	IDUC_R					右电感
	*		-	IDUC_M					中电感
	*		-	IDUC_LM					左中电感
	*		-	IDUC_RM					右中电感
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
	*		-	symStateNow					系统实时状态						X(k)
	*		-	symStatePostFore		系统上次预测状态				X(k|k-1)
	*		-	symStatePostBest		系统上次最优状态				X(k-1|k-1)
	*		-	covNow							本次系统状态协方差			P(k|k)
	*		-	covPostFore					上次预测状态协方差			P(k|k-1)
	*		-	covPostBest					上次最优状态协方差			P(k-1|k-1)
	*		-	symControl					系统控制量							U(k)
	*		-	symParmA						系统参数A							A
	*		-	symParmB						系统参数B							B
	*		-	errorMes						k时刻测量值						Z(k)
	*		-	mesParm							测量系统的参数					H
	*		-	pcesNoise						过程噪声								W(k)
	*		-	mesNoise						测量噪声								V(k)
	*		-	transposeA					A的转置矩阵						A'
	*		-	transposeQ					W(k)的转置矩阵					Q	
	*		-	transposeR					V(k)的转置矩阵					R
	*		-	transposeH					H的转置矩阵						H'
	*		-	gain								卡尔曼增益							Kg
	*/
typedef struct{
	__IO	float symStateNow;
	__IO	float symStatePostFore;
	__IO	float	symStatePostBest;
	__IO	float covNow;
	__IO	float	covPostFore;
	__IO	float covPostBest;
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
	__IO	float gain;
}Kalman;


/**
	*@name		TyPID
	*@type		struct
	*@about 	PID控制系数
	*@param
	*		--
	*		-	Err							自变量
	*		-	Proportion			比例
	*		-	Integral				积分
	*		-	Differential		微分
	*/
typedef struct{
	
	float Err;
	int 	Proportion;
	int 	Integral;
	int 	Differential;
	
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
	
	//电机PWM开启
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	
	//舵机PWM开启
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	
	//定时器3启动
	HAL_TIM_Base_Start_IT(&htim3);
	
	//Kalman初始化
	bsp_iducKalman_Init();
	
	//PCA初始化
	bsp_PCA_Init();
	
	//PID初始化
	bsp_PID_Init();
	
	//电机控制
	bsp_Emac_Operate();
}



/**
	*@funcname		bsp_iducKalman_Init()
	*@brief 			iducKalman初始化
	*@param
	*			--
	*				- X(k|k)	10
	*				- P(k|k)	10
	*				- A				1
	*				- A'			1
	*				- U(k)		0
	*				- H				1
	*				- H'			1
	*				- Q				0.5
	*				- R				0.5
	*/
/*************************** 待调参 *****************************/
void bsp_iducKalman_Init(void)
{
	iducKalman.symStateNow = 10;
	iducKalman.covNow = 10;
	iducKalman.symParmA = 1;
	iducKalman.transposeA = 1;
	iducKalman.symControl = 0;
	iducKalman.mesParm = 1;
	iducKalman.transposeH = 1;
	iducKalman.transposeQ = 0.5;
	iducKalman.transposeR = 0.5;
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
	if(HAL_GPIO_ReadPin(GPIOB, GanHuang_Pin) == GPIO_PIN_RESET)
			Motor->OnOff = 1;
	else
			Motor->OnOff = 0;
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
float bsp_Kalman_Filter(uint16_t data)
{
	iducKalman.errorMes = data;
	
	/*		X(k|k-1)=A X(k-1|k-1)+B U(k)	 */
	iducKalman.symStatePostFore = iducKalman.symParmA * iducKalman.symStatePostBest +
																iducKalman.symParmB * iducKalman.symControl;
	
	/* 		P(k|k-1)=A P(k-1|k-1) A’+Q		*/
	iducKalman.covPostFore = 	iducKalman.symParmA * iducKalman.covPostBest * 
														iducKalman.transposeA +iducKalman.transposeQ;
	
	/* 		X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))		*/
	iducKalman.symStateNow = 	iducKalman.symStatePostFore + iducKalman.gain * 
														(iducKalman.errorMes - iducKalman.mesParm * iducKalman.symStatePostFore);
	
	/* 		Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)		*/
	iducKalman.gain = (iducKalman.covPostFore * iducKalman.transposeH) / 
										(
											iducKalman.mesParm * iducKalman.covPostFore *
											iducKalman.transposeH + iducKalman.transposeR );
	
	/*		P(k|k)=（I-Kg(k) H）P(k|k-1)		*/
	iducKalman.covNow = (1 - iducKalman.gain * iducKalman.mesParm) * iducKalman.covPostFore;
	
	/*		X(k-1|k-1) = X(k|k) 	*/
	iducKalman.symStatePostBest = iducKalman.symStateNow;
	
	/*		P(k-1|k-1) = P(k|k)		*/
	iducKalman.covPostBest = iducKalman.covNow;
	
	return iducKalman.symStateNow;
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
					(	PCA.paramA * (value.IDUC_L - value.IDUC_R) +
						PCA.paramB * (value.IDUC_LM - value.IDUC_RM)) /
					(
						(	PCA.paramA * (value.IDUC_L + value.IDUC_R)) +
						(	PCA.paramC * (fabs((double)(value.IDUC_LM - value.IDUC_RM))))
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
		adcData.filterData[time] = bsp_Kalman_Filter(adcData.orignalData[time]);
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
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)MD[1].pwmrate);
}




/**
	*@funcname		bsp_SteMot_PwmSet()
	*@brief 			舵机PWM设置函数(转向)
	*/
void bsp_SteMot_PwmSet(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)SteMot.pwmrate);
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
	PID.Proportion = 1;
	PID.Integral = 1;
	PID.Differential = 1;
}


/**
	*@funcname		bsp_PID_Core()
	*@brief 			PID核心算法(位置式)
	*/
uint16_t bsp_PID_Core(float error)
{
	PID.Err = error;
	uint16_t PwmRate;		//PwmRate为PID算法结束后得到的PWM控制值
	
	/*
	
			预留代码区
	
			for  栗雍杰  Code Area
	
	*/
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
	*@funcname		bsp_Usart_Operate()
	*@brief 			串口数据处理函数
	*/
void bsp_Usart_Operate(uint8_t *str)
{
	int value;
	uint8_t oder;
	
	if			(*str == 'P' && sscanf((const char *)str, "P:%d", &value) == 1)		oder = 1;
	else if (*str == 'I' && sscanf((const char *)str, "I:%d", &value) == 1)		oder = 2;
	else if (*str == 'D' && sscanf((const char *)str, "D:%d", &value) == 1)		oder = 3;
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
	uint8_t time;
	printf("\n\nThe origanl data:");
	for(time = 0; time < 5; time++){
		printf("\nChannel %d: %d", time, adcData.orignalData[time]);
	}
	printf("\n\nThe filter data:");
	for(time = 0; time < 5; time++){
		printf("\nChannel %d: %.1f", time, adcData.filterData[time]);
	}
	printf("\n\nThe Error data:");
	printf("\nError %d: %.1f", time, adcData.Error);

} 
 






/**
	*@funcname		bsp_Usart_Receive()
	*@brief 			串口接收数据函数
	*/
void bsp_Usart_Receive(void)
{
	if(recv_end_flag ==1)			
		{	
			bsp_LED_FeedBack();
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
	*@funcname		bsp_Emac_Operate()
	*@brief 			电机IN控制
	*/
/*************************** 待完善 *****************************/
void bsp_Emac_Operate(void)
{
	bsp_StartAndStop_Detection();
	if(Motor->OnOff == 0)
	{
		//电机1正转
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		//电机2正转
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	else
	{
		//电机1停止
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		//电机2停止
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}







/**
	*@funcname		bsp_OLED_Display()
	*@brief 			OLED屏显函数
	*/
void bsp_OLED_Display(void)
{
	OLED_ShowString(0, 0, (uint8_t *)"P:", 12);
	OLED_ShowNum(20, 0, PID.Proportion, 3, 12);
	OLED_ShowString(0, 1, (uint8_t *)"I:", 12);
	OLED_ShowNum(20, 1, PID.Integral, 3, 12);
	OLED_ShowString(0, 2, (uint8_t *)"D:", 12);
	OLED_ShowNum(20, 2, PID.Differential, 3, 12);	
}







/**
	*@brief 	定时器中断回调函数
	*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		bsp_ADC_Operate();
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

