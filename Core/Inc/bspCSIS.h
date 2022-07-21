/**
	*@file		bspCSIS.h
	*@data		2022.03.28
	*@author	CSIS（Car Soared Into Sky）
	*@brief		Onboard support for packet header files
	*/

#ifndef __BSPCSIS_H__
#define __BSPCSIS_H__

/*------------------ Include头文件声明 ----------------------- */
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "stdio.h"
#include "OLED.h"
#include "math.h"

/*-------------------- define宏定义区 ----------------------- */
#define ADCVALUEMAX			3800	//ADC采集最大值
#define ADCVALUEMIN			0			//ADC采集最小值
#define BSP_OK					1			//程序异常信号值
#define	BSP_ERROR				0			//程序异常信号值
#define STEPWMMAX				1600	//舵机pwm限幅最大值
#define STEPWMMIN				300  		//舵机pwm限幅最小值




/*------------------- 相关变量引用声明 ---------------------- */
//Usart相关
extern volatile uint8_t rx_len;
extern volatile uint8_t recv_end_flag; 
extern 					uint8_t rx_buffer[200];

/*---------------------- bsp函数声明 ------------------------ */
void bsp_GloableInit(void);
void bsp_Emac_Operate(void);
void bsp_PCA_Init(void);
void bsp_PID_Init(void);
void bsp_OLED_Display(void);
void bsp_Usart_Receive(void);
void bsp_LED_FeedBack(void);
_Bool bsp_SteMot_MakeZero(void);
float bsp_CycleIn(float value);
float bsp_Cross(float	value);
float bsp_Yshape(float value);
void bsp_SignJudge(void);
void bsp_Usart_CallBack(void);

#endif
