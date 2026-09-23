#ifndef _FOC_H_
#define _FOC_H_

#include "main.h"
#include <stdint.h>
#include "myADC.h"
#define DRV8313_ENABLE() GPIOB->ODR|=(0x01<<11)
#define DRV8313_DISABLE() GPIOB->ODR&=~(0x01<<11)

#define FOC_PI                       3.1415927f
#define FOC_2PI                      6.2831853f

typedef enum
{
    PP_DETECT_IDLE = 0,
    /* 转子先吸到固定电角度 */
    PP_DETECT_ALIGN,
    /* 开始慢速旋转磁场 */
    PP_DETECT_RUNNING,
    /* 等待转子完全跟上 */
    PP_DETECT_END_WAIT,
    /* 测量成功 */
    PP_DETECT_DONE,
    /* 测量失败 */
    PP_DETECT_ERROR,
    /* 过流等原因人为停止 */
    PP_DETECT_ABORT
} PPDetectState_t;


typedef struct
{
    PPDetectState_t state;//记录当前检测进行到哪一步。
	
    uint32_t state_tick;//进入当前状态时的时间
    uint32_t update_tick;//控制周期性更新时间
    uint16_t encoder_last;//上一次 MT6816 的 raw 角度。

    /*
     * 机械角累计值。
     * 不直接 end-start，
     * 而是逐次累加，因此能够正确跨过 MT6816 的 0/16383。
     */
    int32_t mechanical_count_acc;//编码器增量累计
    float electrical_angle;//当前准备输出的瞬时电角度。
    float electrical_travel;//电角度累计
    float pole_pairs_float;//计算的级对数
    uint8_t pole_pairs;
    /*
     * +1 / -1
     * 可用于判断编码器方向和三相旋转方向关系
     */
    int8_t direction;
	
} PPDetect_t;


/* 开环 αβ 电压矢量 */
void FOC_SetOpenLoopVector(float electrical_angle,float amplitude);
/* 开启三相 PWM */
void FOC_PWM_Start(void);
/* 停止三相 PWM */
void FOC_PWM_Stop(void);
/* 开始自动检测 */
void FOC_PolePairDetect_Start(PPDetect_t *detect);
/* 必须在 while(1) 中反复调用 */
void FOC_PolePairDetect_Task(PPDetect_t *detect);
/* 紧急停止 */
void FOC_PolePairDetect_Abort(PPDetect_t *detect);
/*
 * 可选：
 * 在 ADC 中断中调用这个函数实现真正的电流保护。
 */
uint8_t FOC_PolePairDetect_CurrentProtect(PPDetect_t *detect,float ia,float ib,float current_limit);
void FOC_StablePointTest(void);

typedef struct MotorParameters
{
	
	unsigned char MOTOR_POLE_PAIRS; //电机极对数
	signed char MOTOR_ENCODER_DIR;	//电机旋转方向
	float ELECTRICAL_OFFSET;//电角度零点对应机械角度
	float VBUS;//母线电压，没adc所以固定12V；
	float Lq;		//q轴电感
	float	Ld;		//d轴电感
	float Rs;		//相电阻
	ADC_Type_t ADC_Parm;//ADC采样参数相电压电流
	
	float Vd;		//d轴电压
	float Vq;		//q轴电压
	float I_alpha;	//alpha轴电流 （克拉克变换）
	float I_beta;		//beta轴电流
	float I_d;			//d轴电流
	float I_q;			//q轴电流
	float theta_e;	//θe电角度
	uint16_t FOC_encoder_raw;//电机编码器值
	char open_L_check_flag;
	char open_Lq_check_flag;
	float	L_check_I_q;
	float	L_check_I_d;
	float J;
	float Kt ;
	float B;
}MotorParameters_t;


//#define MOTOR_POLE_PAIRS    7.0f
//#define MOTOR_ENCODER_DIR  (-1.0f)
//#define MotorParm.ELECTRICAL_OFFSET  5.78f//6.3f// 5.936242f
extern MotorParameters_t MotorParm;
extern float Vd;
extern float Vq;
extern float error;
extern float I_alpha;
extern float I_beta;
extern float I_d;
extern float I_q;
extern float theta_m;
extern float theta_e;
extern float Iq_ref;
float FOC_WrapAngle(float angle);
void FOC_UpdateElectricalAngle(void);
void FOC_Clarke(float Ia, float Ib);
void FOC_Park(float alpha, float beta, float angle);
void FOC_SVPWM(float alpha, float beta);
void FOC_SetOpenLoopVector( float electrical_angle,float amplitude);
void FOC_InvPark(float Vd,float Vq,float theta,float *V_alpha,float *V_beta);
typedef struct
{
    float kp;
    float ki;

    float integral;

    float out_min;
    float out_max;

} FOC_PI_t;

extern FOC_PI_t PI_Id;
extern FOC_PI_t PI_Iq;

float FOC_PI_Run(FOC_PI_t *pi,float target,float feedback,float dt);
float FOC_CurrentLoop(void);

extern volatile float motor_speed_rpm ;
extern volatile float motor_speed_rpm_filt ;
extern float error_V;
void FOC_SpeedCalculate(uint64_t encoder_raw);
void FOC_POSCalculate(uint16_t encoder_raw);
float FOC_PosLoop(int64_t pos_ref);
float FOC_SpeedLoop(float speed_ref);

typedef struct
{
    float kp;
    float ki;
		float kd;
    float integral;
    float out_limit;
		int64_t pos;
} POSPI_t;




extern POSPI_t POS_PI;


signed char NumberOfPolePairs_Check(unsigned char laps_numbles);
void InductorAndRS_Check();

/* 直接在dq坐标系给电压，vd/vq仍然使用你当前的标幺值 */
void FOC_SetDQVoltage(float vd, float vq, float theta);
#endif
