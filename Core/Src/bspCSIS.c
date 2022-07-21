/**
	*@file		bspCSIS.c
	*@data		2022.04.30
	*@version	4.0
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
	*			-	Switch					功能开关
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
	*			-	bsp_ADCValue_PoorCmpAnd()				差比和差加权算法
	*			-	bsp_ADC_Operate()								ADC数据处理函数
	*			-	bsp_Motor_PwmSet()							电机PWM设置函数(调速)[未使用]
	*			-	bsp_SteMot_PwmSet()							舵机PWM设置函数(转向)
	*			-	bsp_PID_Init()									PID初始化
	*			-	bsp_PID_Core()									PID核心算法(位置式)
	*			-	bsp_PID_Control()								PID控制函数
	*			-	bsp_CycleIn()										环岛判断
	*			-	fputc()													串口输出重定向
	*			-	bsp_Esp8266_ValueCallBack()			WiFi数据发送函数
	*			-	bsp_Usart_Operate()							串口数据处理函数
	*			-	bsp_Usart_ADCValueCallBack()		ADC采样值串口回返函数
	*			-	bsp_Usart_Receive()							串口接收数据函数
	*			-	bsp_SteMot_MakeZero()						舵机初始化归零（数字舵机专用）[未使用]
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
	_Bool				IN1;
	_Bool 			IN2;
	float				pwmrate;
	
}MotorDriver;

/**
	*@name		MotDiff
	*@type		struct
	*@about 	电机差速控制
	*@param		
	*		--
	*		-	Param				增幅系数
	*		-	pwmSwitch		电机差速跟随转向控制开关
	*/
struct{
	
	_Bool			pwmSwitch;
	uint16_t	basepwmvalue;
	float 		Param;
	
}MotDiff;

/**
	*@name		SteMotDriver
	*@type		struct
	*@about 	舵机控制
	*@param
	*		-	pwmrateTemp		pwm控制过渡值
	*		- pwmrateFnal		pwm控制最终值
	*		-	Min						舵机中值
	*		- angle					舵机目前角度
	*/
typedef struct{
	
	uint8_t			angle;
	float				pwmrateTemp;	
	float   	  pwmrateFnal;
	float				Min;

	
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
	__IO	float				IDUC_Ex;
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
	
	uint8_t	flag;			//控制算法转换
	float paramA;
	float paramB;
	float paramC;
	float paramP;
	float paramL;
	
}PoorCmpAnd;


/**
	*@name		Switch
	*@type		struct
	*@about 	函数开关
	*@param
	*		- 		
	*		- 		
	*/
struct{
	
	_Bool	ONOF1;			//环岛
	_Bool	ONOF2;			//十字
	_Bool ONOF3;			//Y形
	_Bool ONOF4;			//标志值捕获
	uint8_t ONOF5;		//滤波
	
}Switch = {1, 0, 1, 0, 1};

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
	*		-	Integra_Max				积分限幅值
	*/
typedef struct{
	
	int 	Proportion;
	int 	Integral;
	int 	Differential;
	float Err;
	float	ErrLastValue[3];
	float	ErrLastSum;
	float	Integral_Max;
	float k;
	float b;
	
}TyPID;

/**
	*@breif	全局标志位
	*/
struct Flag{
	uint16_t 	A;		//环岛
	uint16_t 	B;		//环形
	uint16_t	C;		//Y形
	uint16_t	D;		//捕获sign
	uint16_t	G;		//干簧管
	uint16_t	S;		//出库与入库
	uint16_t	T;		//出库
	uint16_t	K;		//Y形消抖
	uint16_t	W;		//入库消抖
}Flag = {0, 0, 0, 0, 0, 0, 0, 0};

/**
	*@breif	中断读秒
	*/
struct	ITReadTimes{
	int	Tim1;		//环岛
	int	Tim2;		//入库
	int Tim3;		//Y形
	int	Tim4;
	int Tim5;
	int	Tim6;
	int	Tim7;
	int Tim8;
}ITRT = {270, 45, 100, 35, 30, 30, 30, 30};

/**
	*@breif	赛道特征判断值
	*/
struct JudgeValue{
	uint16_t	jm1;		//环岛
	uint16_t	jm2;		//环岛
	uint16_t	jm3;		//十字
	uint16_t	jm4;		//十字
	uint16_t	jm5;		//十字
	uint16_t	jm6;		//Y形
	uint16_t	jm7;		//Y形
	uint16_t	jm8;		//捕获
}JDVL = {3800, 500, 2000, 100, 100, 10, 100, 2000};

/*------------------------ 相关变量定义 --------------------------- */

Kalman iducKalman;			//卡尔曼滤波
PoorCmpAnd PCA;					//差比和差加权算法系数
MotorDriver Motor[2];		//电机
SteMotDriver SteMot;		//舵机
ADCData adcData;				//ADC采样
TyPID PID;							//舵机PID
TyPID MotorPID;					//电机PID（闲置）

uint8_t	huandaoK = 20;	//环岛中值电感增益压幅值



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
	HAL_ADC_Start(&hadc2);
	
	
	//舵机PWM开启
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	
	
	//电机PWM开启
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	
	
	//定时器3启动
	HAL_TIM_Base_Start_IT(&htim3); 
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start(&htim3);
 
	
	//PCA初始化
	bsp_PCA_Init();
	
	//PID初始化
	bsp_PID_Init();
	
	//电机控制
	bsp_Emac_Operate();
	
	//舵机中值初始化
	SteMot.Min = 950;
	
	//电机开关初始化
	Motor[0].OnOff = 1;
	
	//电机差速控制开关初始化
	MotDiff.pwmSwitch = 0;
	
	//电机差速跟随控制增幅系数
	MotDiff.Param = 100;
	
	//电机速度pwm控制基值初始化
	MotDiff.basepwmvalue = 3000;
	Motor[0].pwmrate = MotDiff.basepwmvalue;
	Motor[1].pwmrate = MotDiff.basepwmvalue;
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
void bsp_PCA_Init(void)
{
	PCA.paramA = 1.90;
	PCA.paramB = 6.75;
	PCA.paramC = 9.65;
	PCA.paramP = 1.18;
	PCA.paramL = 1;
}


/**
	*@funcname		bsp_PID_Init()
	*@brief 			PID初始化
	*@param
	*			--
	*				- P			1
	*				- I			1
	*				- D			1
	*/
void bsp_PID_Init(void)
{
	PID.Proportion = 30;
	PID.Integral = 0;
	PID.Differential = 80;
	PID.Integral_Max = 0;
	PID.b = 55;
	PID.k = 178;
	
	PID.ErrLastSum = 0;
	PID.ErrLastValue[0] = 0.0;
	PID.ErrLastValue[1] = 0.0;
	PID.ErrLastValue[2] = 0.0;
}


/**
	*@funcname		bsp_OutAndInbound()
	*@brief 			出库与入库
	*/
void bsp_OutAndInbound(void)
{
	/* 入库 */
	if (Flag.G == 2)
	{
		Flag.S = 1;
		MotDiff.basepwmvalue = 2000;
	}
	if (Flag.S == 1 && ITRT.Tim2 <= 0)
		Motor->OnOff = 0;

}

/**
	*@funcname		bsp_StartAndStop_Detection()
	*@brief 			干簧管起始与停止检测
	*/
void bsp_StartAndStop_Detection(void)
{
	/*
	if(HAL_GPIO_ReadPin(GPIOB, GanHuang_Pin) == GPIO_PIN_SET)
	{
		Flag.G++;
	}
	bsp_OutAndInbound();
	*/
}




/**
	*@funcname		bsp_LED_FeedBack()
	*@brief 			LED程序测试闪烁反馈函数
	*/
void bsp_LED_FeedBack(void)
{
	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
}



/**
	*@funcname		bsp_ArBi_Filter()
	*@brief 			一阶(αβ)滤波
	*/
float bsp_ArBi_Filter(uint16_t value, uint8_t i)
{
	static uint16_t ArBi_lastValue[5] = {0, 0, 0, 0, 0};
	float result;
	
	result = 0.80 * value + (1 - 0.80) * ArBi_lastValue[i];
	ArBi_lastValue[i] = result;
	
	return result;
}


/**
	*@funcname		bsp_ADCValue_PoorCmpAnd()
	*@brief 			差比和差加权算法
	*/
float bsp_ADCValue_PoorCmpAnd(ADCData value)
{
	float Err;
	
	/* 差比和差加权 */
	if(PCA.flag == 0)
	{
		Err = (
						(	PCA.paramA * (value.IDUC_L - value.IDUC_R) +
							PCA.paramB * (value.IDUC_LM - value.IDUC_RM)) /
						(
							(	PCA.paramA * (value.IDUC_L + value.IDUC_R)) +
							(	PCA.paramC * (fabs((double)(value.IDUC_LM - value.IDUC_RM))))
						)
					) * PCA.paramP;
						
	}
	
	return Err;
}



/**
	*@funcname		bsp_ADC_Operate()
	*@brief 			ADC数据处理函数
	*@logic
	*		--
	*		-	通道滤波 -> 差比和加权算法 -> 输出Err
	*/
void bsp_ADC_Operate(void)
{
	uint8_t time;

		//一阶(αβ)滤波
		for(time = 0; time < 5; time++)
		{
			adcData.filterData[time] = bsp_ArBi_Filter(adcData.orignalData[time], time);
		}
		
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
	*@brief 			电机PWM控制函数(调速)
	*/
void bsp_Motor_PwmSet(MotorDriver *MD)
{
	char K = 0;
	
	if(MotDiff.pwmSwitch==1)
	{
		K = adcData.Error > 0 ? 1 : -1;
		MD[0].pwmrate += K * (MotDiff.Param * (SteMot.pwmrateFnal - SteMot.Min) /
																					(STEPWMMAX - SteMot.Min));
		MD[1].pwmrate -= K * (MotDiff.Param * (SteMot.pwmrateFnal - SteMot.Min) /
																					(STEPWMMAX - SteMot.Min));
	}
	
	Motor[0].pwmrate = MotDiff.basepwmvalue;
	Motor[1].pwmrate = MotDiff.basepwmvalue;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)MD[0].pwmrate);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint16_t)MD[1].pwmrate);
}




/**
	*@funcname		bsp_SteMot_PwmSet()
	*@brief 			舵机PWM设置函数(转向)
	*/
void bsp_SteMot_PwmSet(float value)
{
	float pwmrate = value;
	
	/* 中值增减 */
	pwmrate += SteMot.Min;
	
	/* 环岛判断 */
	pwmrate = bsp_CycleIn(pwmrate);
	
	/* 十字判断 */
	pwmrate = bsp_Cross(pwmrate);
	
	/* Y形判断 */
	pwmrate = bsp_Yshape(pwmrate);
	
	/* 出库判断 */
	if (Flag.T == 0)
	{
		pwmrate += 220;
	}
	/* 入库 */
		if (Flag.S == 1 && ITRT.Tim2 != 0)
	{
		pwmrate += 230;
	}
	
	/* 标志值捕获 */
	bsp_SignJudge();
	
	if (Flag.S == 1 && ITRT.Tim2 > 0)
		pwmrate += 150;
	if (Flag.S == 1 && ITRT.Tim2 <= 0)
		Motor->OnOff = 0;
	
	/* 限幅 */
	if(pwmrate < STEPWMMIN)
		pwmrate = STEPWMMIN;
	else if (pwmrate > STEPWMMAX)
		pwmrate = STEPWMMAX;
	
	/* 终值更新 */
	SteMot.pwmrateFnal=pwmrate;
	
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, (uint16_t)SteMot.pwmrateFnal);
}



/**************************************************************************************************
***************************************************************************************************/

/**
	*@funcname		bsp_PID_Core()
	*@brief 			PID核心算法(位置式)
	*/
float bsp_PID_Core(float error)
{
	PID.Err = error;
	float PwmRate;		
	
	PID.ErrLastValue[2] = PID.ErrLastValue[1];
	PID.ErrLastValue[1] = PID.ErrLastValue[0];
	PID.ErrLastValue[0] = error;
	
	/* 积分限幅 */
	if (
			((PID.ErrLastSum + error) < PID.Integral_Max) &&
			((PID.ErrLastSum + error) > -PID.Integral_Max)
		 )
	{
		/* err值累加 */
		PID.ErrLastSum += error;			
	}
	else if (PID.ErrLastSum > 0)		
	{
		/* 正向限幅 */
		PID.ErrLastSum = PID.Integral_Max;
	}
	else if (PID.ErrLastSum < 0)		
	{
		/* 反向限幅 */
		PID.ErrLastSum = -PID.Integral_Max;
	}
	
	PID.Proportion = PID.k * fabs(adcData.Error)+ PID.b;
	/* Core */
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
	SteMot.pwmrateTemp = bsp_PID_Core(adcData.Error);
	bsp_SteMot_PwmSet(SteMot.pwmrateTemp);

}


/**
	*@funcname		bsp_CycleIn()
	*@brief 			环岛判断
	*/
float bsp_CycleIn(float value)
{
	float result = value;
	
	if (Switch.ONOF1 == 1)
	{
		if((adcData.IDUC_LM + adcData.IDUC_RM) > 5000 && adcData.IDUC_M > 3000)
		{
					Flag.A = 1;
		}
	}
	
	if (Flag.A == 1)
	{
		result -= adcData.IDUC_Ex/15;
	}
		return result;
}



/**
	*@funcname		bsp_Cross()
	*@brief 			十字，环形
	*/
float bsp_Cross(float	value)
{	
	float result = value;
	
	if (Switch.ONOF2 == 1)
	{
		if (adcData.IDUC_M > JDVL.jm3)
		{
			if (adcData.IDUC_L - adcData.IDUC_R < JDVL.jm4)
			{
				if (adcData.IDUC_LM - adcData.IDUC_RM < JDVL.jm5)
				{
					Flag.B++;
				}
			}
		}
	}
	return result;
}

/**
	*@funcname		bsp_Yshape()
	*@brief 			Y形
	*/
float bsp_Yshape(float value)
{
	float result = value;
	
	if (Switch.ONOF3 == 1)
	{
		if (adcData.IDUC_M < 10)
		{
			if (adcData.IDUC_Ex < 120 && adcData.IDUC_M < 120 && ITRT.Tim3 == 100)
			{				
					Flag.C++;
					Flag.K = 1;			
			}
		}
	}
	
	if(Flag.C == 1)
	{
		result -= 120;
	}
	if(Flag.C == 3)
	{
		result += 170;
	}
	return result;
}

/**
	*@funcname		bsp_SignJudge()
	*@brief 			标志值捕获
	*/
void bsp_SignJudge(void)
{
	if (Switch.ONOF4 == 1)
	{
		if (adcData.IDUC_R - adcData.IDUC_L > JDVL.jm8)
			Flag.D++;
	}
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
	*@brief 			串口指令
	*@oder
	*		--
	*		-		1			P:value				PID_P = value
	*		-		2			I:value				PID_I = value
	*		-		3			D:value				PID_D = value
	*		-		4			A:value				PCA_A = value
	*		-		5			B:value				PCA_B = value
	*		-		6			C:value				PCA_C = value
	*		-		7			p:value				PCA_P = value
	*		-		24		L:value				PCA_L = value
	*		-		8			M:value				SteMot.Min = value
	*		-		9			--	
	*							-	G:1					电机启动
	*							-	G:0					电机关闭
	*		-		10		--
	*							-	S:1					电机差速跟随控制启动
	*							-	S:0					电机差速跟随控制关闭
	*		-		11		M:value				电机占空比（速度）	
	*		-		15		Pb:value			PID_Pb = value
	*		-		16		Pk:value			PID_Pk = value
	*		-		19		s1:value			开断环岛判断
	*		-		20		s2:value			开断十字判断
	*		-		21		s3:value			开断Y形判断
	*		-		22		s4:value			开断标识值捕获
	*		-		23		s5:value			切换滤波算法
	*		-		17		W:value				切换归一化算法
	*		-		18		h:value				环岛中值电感增益值削减度
	*		-		25		U:Any					串口返回参数值
	*		--
	*/
void bsp_Usart_Operate(uint8_t *str)
{
	float value;
	uint8_t oder = 0;
	
	if			((*str == 'P') && (sscanf((const char *)str, "P:%f", &value) == 1))		oder = 1;
	else if ((*str == 'I') && (sscanf((const char *)str, "I:%f", &value) == 1))		oder = 2;
	else if ((*str == 'D') && (sscanf((const char *)str, "D:%f", &value) == 1))		oder = 3;
	else if ((*str == 'A') && (sscanf((const char *)str, "A:%f", &value) == 1))		oder = 4;
	else if ((*str == 'B') && (sscanf((const char *)str, "B:%f", &value) == 1))		oder = 5;
	else if ((*str == 'C') && (sscanf((const char *)str, "C:%f", &value) == 1))		oder = 6;
	else if ((*str == 'L') && (sscanf((const char *)str, "L:%f", &value) == 1))		oder = 24;
	else if ((*str == 'p') && (sscanf((const char *)str, "p:%f", &value) == 1))		oder = 7;
	else if ((*str == 'F') && (sscanf((const char *)str, "F:%f", &value) == 1))		oder = 8;
	else if ((*str == 'G') && (sscanf((const char *)str, "G:%f", &value) == 1))		oder = 9;
	else if ((*str == 'S') && (sscanf((const char *)str, "S:%f", &value) == 1))		oder = 10;
	else if ((*str == 'M') && (sscanf((const char *)str, "M:%f", &value) == 1))		oder = 11;
	
	else if ((*str == 'P') && (*(str+1) == 'k') && (sscanf((const char *)str, "Pk:%f", &value) == 1))		oder = 15;
	else if ((*str == 'P') && (*(str+1) == 'b') && (sscanf((const char *)str, "Pb:%f", &value) == 1))		oder = 16;
	else if ((*str == 's') && (*(str+1) == '1') && (sscanf((const char *)str, "s1:%f", &value) == 1))		oder = 19;
	else if ((*str == 's') && (*(str+1) == '2') && (sscanf((const char *)str, "s2:%f", &value) == 1))		oder = 20;
	else if ((*str == 's') && (*(str+1) == '3') && (sscanf((const char *)str, "s3:%f", &value) == 1))		oder = 21;
	else if ((*str == 's') && (*(str+1) == '4') && (sscanf((const char *)str, "s4:%f", &value) == 1))		oder = 22;
	else if ((*str == 's') && (*(str+1) == '5') && (sscanf((const char *)str, "s5:%f", &value) == 1))		oder = 23;
		
	else if ((*str == 'W') && (sscanf((const char *)str, "W:%f", &value) == 1))		oder = 17;
	else if ((*str == 'h') && (sscanf((const char *)str, "h:%f", &value) == 1))		oder = 18;
	else if ((*str == 'U') && (sscanf((const char *)str, "U:%f", &value) == 1))		oder = 25;
	
	else if ((*str == 'J') && (*(str+1) == '1') && (sscanf((const char *)str, "J1:%f", &value) == 1))		oder = 26;
	else if ((*str == 'J') && (*(str+1) == '2') && (sscanf((const char *)str, "J2:%f", &value) == 1))		oder = 27;
	else if ((*str == 'J') && (*(str+1) == '3') && (sscanf((const char *)str, "J3:%f", &value) == 1))		oder = 28;
	else if ((*str == 'J') && (*(str+1) == '4') && (sscanf((const char *)str, "J4:%f", &value) == 1))		oder = 29;
	else if ((*str == 'J') && (*(str+1) == '5') && (sscanf((const char *)str, "J5:%f", &value) == 1))		oder = 30;
	else if ((*str == 'J') && (*(str+1) == '6') && (sscanf((const char *)str, "J6:%f", &value) == 1))		oder = 31;
	else if ((*str == 'J') && (*(str+1) == '7') && (sscanf((const char *)str, "J7:%f", &value) == 1))		oder = 32;
	else if ((*str == 'J') && (*(str+1) == '8') && (sscanf((const char *)str, "J8:%f", &value) == 1))		oder = 33;
	
	else		printf("\nvalue set fail!");
	
	switch(oder)
	{
		case 1: 
						PID.Proportion = (int)value;	
						printf("\nSuccessful set the value of P: %d", PID.Proportion);
						break;
		case 2: 
						PID.Integral = (int)value;	
						printf("\nSuccessful set the value of I: %d", PID.Integral); 
						break;
		case 3:
						PID.Differential = (int)value;	
						printf("\nSuccessful set the value of D: %d", PID.Differential); 
						break;
		case 4:
						PCA.paramA = value;
						printf("\nSuccessful set the value of PCA-A: %f", PCA.paramA); 
						break;
		case 5:
						PCA.paramB = value;
						printf("\nSuccessful set the value of PCA-B: %f", PCA.paramB); 
						break;
		case 6:
						PCA.paramC = value;
						printf("\nSuccessful set the value of PCA-C: %f", PCA.paramC); 
						break;
		case 7:
						PCA.paramP = value;
						printf("\nSuccessful set the value of PCA-P: %f", PCA.paramP); 
						break;
		case 24:
						PCA.paramL = value;
						printf("\nSuccessful set the value of PCA-L: %f", PCA.paramL); 
						break;
		case 8:
						MotDiff.Param = value;
						printf("\nSuccessful set the value of SteMin: %f", MotDiff.Param); 
						break;
		case 9:
						if(value == 1)
						{
							Motor[0].OnOff = 1;
							printf("\nCar Motor ON."); 
						}
						else if (value == 0)
						{
							Motor[0].OnOff = 0;
							printf("\nCar Motor OFF.");
						}
						else
							printf("Err!!");
						break;
		case 10:
						if(value == 1)
						{
								MotDiff.pwmSwitch = 1;
								printf("\nCar Motor Diff Foller ON."); 
						}
						else if (value == 0)
						{
								MotDiff.pwmSwitch = 0;
								printf("\nCar Motor Diff Follor OFF.");
						}
						else
							printf("Err!!");
						break;

		case 11:
						MotDiff.basepwmvalue = (uint16_t)value;
						printf("\nMotor Speed: %.2f%%", (MotDiff.basepwmvalue/100.0)); 
						break;
		case 15:
						PID.k = value;
						printf("\nPID Pk: %.2f", PID.k); 
						break;
		case 16:
						PID.b = value;
						printf("\nPID Pb: %.2f", PID.b); 
						break;
		case 17:
						PCA.flag = value;
						printf("\nSuccessful set the value of pac flag: %f", (float)PCA.flag); 
						break;
		case 18:
						huandaoK = (uint8_t)value;
						printf("\nSuccess! %d", huandaoK);
						break;
		case 19:
						Switch.ONOF1 = (_Bool)value;
						printf("\nSuccess! ONOF1: %d", Switch.ONOF1);
						break;
		case 20:
						Switch.ONOF2 = (_Bool)value;
						printf("\nSuccess! ONOF2: %d", Switch.ONOF2);
						break;
		case 21:
						Switch.ONOF3 = (_Bool)value;
						printf("\nSuccess! ONOF3: %d", Switch.ONOF3);
						break;
		case 22:
						Switch.ONOF4 = (_Bool)value;
						printf("\nSuccess! ONOF4: %d", Switch.ONOF4);
						break;
		case 23:
						Switch.ONOF5 = (uint8_t)value;
						printf("\nSuccess! ONOF5: %d", Switch.ONOF5);
						break;
		case 25:
						bsp_Usart_CallBack();
						break;
		
		case 26:
						JDVL.jm1 = (uint16_t)value;
						printf("\nJ1: %d", JDVL.jm1);
						break;
		case 27:
						JDVL.jm2 = (uint16_t)value;
						printf("\nJ2: %d", JDVL.jm2);
						break;
		case 28:
						JDVL.jm3 = (uint16_t)value;
						printf("\nJ3: %d", JDVL.jm3);
						break;
		case 29:
						JDVL.jm4 = (uint16_t)value;
						printf("\nJ4: %d", JDVL.jm4);
						break;
		case 30:
						JDVL.jm5 = (uint16_t)value;
						printf("\nJ5: %d", JDVL.jm5);
						break;
		case 31:
						JDVL.jm6 = (uint16_t)value;
						printf("\nJ6: %d", JDVL.jm6);
						break;
		case 32:
						JDVL.jm7 = (uint16_t)value;
						printf("\nJ7: %d", JDVL.jm7);
						break;
		case 33:
						JDVL.jm8 = (uint16_t)value;
						printf("\nJ8: %d", JDVL.jm8);
						break;
		default:	break;
	}

}

/**
	*@funcname		bsp_Usart_Receive()
	*@brief 			串口接收数据函数
	*/
void bsp_Usart_Receive(void)
{
	if(recv_end_flag ==1)			
		{	
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
void bsp_Emac_Operate(void)
{
	/* 干簧管检测 */
	bsp_StartAndStop_Detection();

	//电机启动
	if(Motor[0].OnOff == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	}
	//电机停止
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	}
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
	bsp_SteMot_PwmSet(SteMot.pwmrateTemp);
	
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
	OLED_ShowNum(100, 4, adcData.IDUC_Ex, 4, 12);
	
	OLED_ShowString(0, 5, (uint8_t *)"PwmRate:", 12);
	OLED_ShowUnFloat(60, 5, SteMot.pwmrateFnal, 7, 2, 12);
	
	OLED_ShowString(0, 6, (uint8_t *)"A:", 12);
	OLED_ShowNum(30, 6, Flag.A, 4, 12);
	OLED_ShowString(60, 6, (uint8_t *)"B:", 12);
	OLED_ShowNum(100, 6, Flag.B, 4, 12);
	OLED_ShowString(0, 7, (uint8_t *)"C:", 12);
	OLED_ShowNum(30, 7, Flag.C, 4, 12);
	OLED_ShowString(60, 7, (uint8_t *)"D:", 12);
	OLED_ShowNum(100, 7, Flag.D, 4, 12);
	
}


/**
	*@brief 	定时器中断回调函数
	*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		HAL_ADC_Start(&hadc2);
		adcData.IDUC_Ex = HAL_ADC_GetValue(&hadc2);
		bsp_PID_Control();
		bsp_Motor_PwmSet(Motor);
	}
	if(htim == &htim4)
	{
		if (Flag.S == 1)
		{
			ITRT.Tim2--;
		}
		if (Flag.K == 1)
		{
			ITRT.Tim3--;
			if(ITRT.Tim3 <= 0)
			{
				Flag.K = 0;
				ITRT.Tim3 = 100;
			}
		}
		if (Flag.A == 1)
		{
			ITRT.Tim4--;
			if (ITRT.Tim4 <= 0)
			{
				Flag.A = 0;
				ITRT.Tim4 = 35;
			}
		}
		if (Flag.T == 0)
		{
			ITRT.Tim5--;
			if (ITRT.Tim5 <= 0)
			{
				Flag.T = 1;
			}
		}
		if (Flag.C == 1)
		{
			ITRT.Tim6--;
			if (ITRT.Tim6 <= 0)
			{
				Flag.C = 2;
				ITRT.Tim6= 30;
			}
		}		
		if(Flag.C == 3)
		{
			ITRT.Tim7--;
			if(ITRT.Tim7<=0)
			{
				Flag.C = 0;
				ITRT.Tim7 = 30;
			}
		}
		if (Flag.W == 1)
		{
			ITRT.Tim8--;
			if (ITRT.Tim8 <= 0)
			{
				Flag.W = 0;
				ITRT.Tim8 = 30;
			}
		}
	}
}

/**
	*@funcname		bsp_Usart_CallBack()
	*@brief 			参数返回
	*/
void bsp_Usart_CallBack(void)
{
	printf("\n");
	printf("\nPID - P: %d", PID.Proportion);
	printf("\nPID - I: %d", PID.Integral);
	printf("\nPID - D: %d", PID.Differential);
	printf("\nPID - Pb: %.2f", PID.b);
	printf("\nPID - Pk: %.2f", PID.k);
	printf("\n");
	printf("\nPCA - A: %.2f", PCA.paramA);
	printf("\nPCA - B: %.2f", PCA.paramB);
	printf("\nPCA - C: %.2f", PCA.paramC);
	printf("\nPCA - P: %.2f", PCA.paramP);
	printf("\nPCA - L: %.2f", PCA.paramL);
	printf("\n");
	printf("\nSpeed: %.2f%%", MotDiff.basepwmvalue/100.0);
	if (MotDiff.pwmSwitch == 1)
	{
		printf("\nMotDiff: ON");
		printf("\nMotDiff Param: %.2f", MotDiff.Param);
	}
	else
		printf("\nMotDiff: OFF");
	if (Switch.ONOF1 == 1)
	{
		printf("\nhuandaoK; %d", huandaoK);
		printf("\nJ1: %d", JDVL.jm1);
		printf("\nJ2: %d", JDVL.jm2);
	}
	if (Switch.ONOF2 == 1)
	{
		printf("\nJ3: %d", JDVL.jm3);
		printf("\nJ4: %d", JDVL.jm4);
		printf("\nJ5: %d", JDVL.jm5);
	}
	if (Switch.ONOF3 == 1)
	{
		printf("\nJ6: %d", JDVL.jm6);
		printf("\nJ7: %d", JDVL.jm7);
	}
	if (Switch.ONOF4 == 1)
		printf("\nJ8: %d", JDVL.jm8);
	
	printf("\nFlag G:%d", Flag.G);
	printf("\nITRT Tim2: %d", ITRT.Tim2);
	printf("\nITRT Tim1: %d", ITRT.Tim1);
	printf("\nITRT Tim3: %d", ITRT.Tim3);
	printf("\nErr: %.3f", adcData.Error);
	
	printf("\n");
}





/**
  * @brief  外部中断回调函数
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if (GPIO_Pin == GPIO_PIN_1 && Flag.W == 0)
	{
		Flag.G++;
		Flag.W = 1;
		bsp_OutAndInbound();
		while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
	}
	else if (GPIO_Pin == Key1_Pin)
	{
		Motor->OnOff = !(Motor->OnOff);
		while(!HAL_GPIO_ReadPin(GPIOA, Key1_Pin));
	}
	
}









