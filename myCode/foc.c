/*
链路

 STM32G431
																		│
														Position / Speed
																		│
																		▼
														iq_ref / id_ref
																		│
																		▼
 Ia ──┐              ┌─────────────┐
				├───────│			 Clarke  					  │
 Ib ──┘              └──────┬──────┘
																			│
																		Iα Iβ
																			│
																			▼
													┌──────────┐
		Encoder ──θe───	│				 Park  		  │
													└────┬─────┘
																		│
																	Id Iq
																		│
																		▼
																Id/Iq PI
																		│
																	Vd Vq
																		│
																		▼
																Inv Park
																		│
																	Vα Vβ
																		│
																		▼
																		SVPWM
												┌─────┼─────┐
												│					│ 			    │
											DutyU				 DutyV			 DutyW
												│     			│    			│
											TIM1_CH1 			CH2   			CH3
												│   			  │ 			    │
												▼     			▼    		  ▼
												IN1 			 IN2   			IN3
												└─────┬─────┘
																		│
																	DRV8313
												┌─────┼─────┐
												▼ 			    ▼ 			    ▼
											OUT1 			 OUT2  				OUT3
												U    			 V     				W
																	PMSM



*/

#include "bspUSB.h"
#include "myADC.h"
#include "foc.h"
#include "tim.h"
#include "MT6816.h"

#include <math.h>

//初始化全局变量
MotorParameters_t MotorParm=
{
	.MOTOR_POLE_PAIRS=7,
	.VBUS=12,
	.ELECTRICAL_OFFSET=-5.768919f,
	.MOTOR_ENCODER_DIR=-1,
	.open_L_check_flag=0,
	.open_Lq_check_flag=0,
	.Ld = 1.248f,
	.Lq = 1.462f
};
	

/* =========================================================
 * 内部函数,限制幅值范围
 * ========================================================= */
static float FOC_Limit(float value, float min_value,float max_value)
{
    if(value > max_value)
        return max_value;
    if(value < min_value)
        return min_value;
    return value;
}

/* =========================================================
 * 三相 PWM
 * ========================================================= */

void FOC_PWM_Start(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}


void FOC_PWM_Stop(void)
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);//a
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);//b
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);//c
}

//基础算法部分
//clark变换
void FOC_Clarke(float Ia, float Ib)
{
	I_alpha = Ia;
	I_beta	 =(Ia + 2.0f * Ib)* 0.5773502f;
}

float s=0,c=0;
//park变换
void FOC_Park(float alpha, float beta, float angle)
{
	s= sinf(angle);
	c = cosf(angle);
	I_d = alpha * c + beta  * s;
	I_q =-alpha * s + beta  * c;
}
/*park逆变换
Vd
Vq
theta 电角度
*/
void FOC_InvPark(float Vd, float Vq, float theta,float *V_alpha,float *V_beta)
{
	s = sinf(theta);
	c = cosf(theta);
	*V_alpha =Vd * c - Vq * s;
	*V_beta = Vd * s +Vq * c;
}
/*
SPWM算法
*/
void FOC_SPWM(float alpha, float beta)
{
    float va;
    float vb;
    float vc;
	
    float duty_a;
    float duty_b;
    float duty_c;
    uint32_t arr;
    /*
     * alpha/beta -> abc克拉克逆变换
     */
    va = alpha;
    vb = -0.5f * alpha  + 0.8660254038f * beta;
    vc = -0.5f * alpha- 0.8660254038f * beta;
    /*
     * SPWM:
     * 不做 vmax/vmin 零序注入
     */
    duty_a = 0.5f - va;
    duty_b = 0.5f - vb;
    duty_c = 0.5f - vc;
    /*
     * 防止超过合法 duty
     */
    duty_a = FOC_Limit(duty_a, 0.02f, 0.98f);
    duty_b = FOC_Limit(duty_b, 0.02f, 0.98f);
    duty_c = FOC_Limit(duty_c, 0.02f, 0.98f);
	//计算解析：dutya*arr:占空比*周期为高电平持续时间建议改（1-duty)
	/*（中心对齐模式）
	我期望中间直接满足					现在实际俩边加起来才是我的实际占空比
			_______								_____					____
		 |			 |									 |			 |
	___|			 |___								 |_______|	
	*/
    arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE( &htim1,TIM_CHANNEL_1,(uint32_t)(duty_a * (float)arr) );
    __HAL_TIM_SET_COMPARE( &htim1,TIM_CHANNEL_2,(uint32_t)(duty_b * (float)arr));
    __HAL_TIM_SET_COMPARE( &htim1,TIM_CHANNEL_3,(uint32_t)(duty_c * (float)arr) );
}
/*************************************
SVPWM算法：电压马鞍波
************************************/
void FOC_SVPWM(float alpha, float beta)
{
		//三相电压
    float va;
    float vb;
    float vc;
		//电压最大最小值
    float vmax;
    float vmin;
		//零序注入量
    float offset;
		//三相占空比
    float duty_a;
    float duty_b;
    float duty_c;
		//定时周期
    uint32_t arr;
    /* ===============================
     * Alpha/Beta -> 三相 克拉克逆变换
     * =============================== */
    va = alpha;
    vb = -0.5f * alpha+ 0.8660254f * beta;
    vc = -0.5f * alpha- 0.8660254f * beta;
    /* ===============================
     * 找最大、最小相电压
     * =============================== */
    vmax = va;
    if(vb > vmax)
        vmax = vb;
    if(vc > vmax)
        vmax = vc;
		
    vmin = va;
    if(vb < vmin)
        vmin = vb;
    if(vc < vmin)
        vmin = vc;
    /* ===============================
     * 零序注入 / SVPWM
     * =============================== */
    offset = 0.5f * (vmax + vmin);
    va -= offset;
    vb -= offset;
    vc -= offset;
    /* ===============================
     * 转换成 0~1 Duty占空比(1-0.5+va)
     * =============================== */
    duty_a = 0.5f - va;
    duty_b = 0.5f - vb;
    duty_c = 0.5f - vc;
    /*
     * 最终保护
     */
    duty_a = FOC_Limit(duty_a, 0.02f, 0.98f);
    duty_b = FOC_Limit(duty_b, 0.02f, 0.98f);
    duty_c = FOC_Limit(duty_c, 0.02f, 0.98f);
    /* ===============================
     * 更新TIM1 CCR
     * =============================== */
    arr = __HAL_TIM_GET_AUTORELOAD(&htim1)+1U;
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1,(uint32_t)(duty_a * (float)arr) );
    __HAL_TIM_SET_COMPARE( &htim1,TIM_CHANNEL_2, (uint32_t)(duty_b * (float)arr));
    __HAL_TIM_SET_COMPARE( &htim1,TIM_CHANNEL_3,(uint32_t)(duty_c * (float)arr));
}

/* =========================================================
 * 开环 SVPWM
 * amplitude:
 * 第一次不要给大。
 * ========================================================= */

void FOC_SetOpenLoopVector(float electrical_angle, float amplitude)
{
    float alpha;//a轴
    float beta;	//b轴

	   /* 限制整个电压矢量 */
    amplitude = FOC_Limit(amplitude, 0.0f, 0.50f);
    /*
     * αβ 静止坐标系电压矢量
     */
    alpha = amplitude * cosf(electrical_angle);
    beta  = amplitude * sinf(electrical_angle);
		FOC_SVPWM( alpha, beta );
}

/*
极对数检测 转子吸合-》记录开始POS-》转子运动-》记录结束POS-》计数极对数-》吸合-》计数电角度偏移

*/
signed char NumberOfPolePairs_Check(unsigned char laps_numble)
{
	float a=0; //旋转角
	float PolePairs=0;
	static long start_pos=0;
	char Symbol=0;//记录符号
	FOC_SetOpenLoopVector(a,0.2);//转子先吸合1s;
	HAL_Delay(1000);
	start_pos =POS_PI.pos;
	for(unsigned int i=0;i<360*laps_numble;i++)
	{
		a+=FOC_2PI/360;
		if(a>FOC_2PI)
			a-=FOC_2PI;
		FOC_SetOpenLoopVector(a,0.1);//转子旋转
		HAL_Delay(1);
		usb_print("Electrical angle:%f,%f,%f,%f,%f%%\r\n",a,ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c,(float)i*100/(360*laps_numble));
	}
	HAL_Delay(1000);
	if(POS_PI.pos-start_pos!=0)
		PolePairs=(float)laps_numble/((float)(POS_PI.pos-start_pos)/MT6816_CPR);
	else
		return (signed char)-1U;
	usb_print("Electrical angle:%f,%f,%f,%f,%f%%,%f\r\n",a,ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c,100,PolePairs);
	//处理数据并记录剔除小数约0.2-0.8的数据，因为不对,7.1是7对极,7.4?不可信
	//6.8+0.2=7>6  -6.8+0.2=-6.6<6 -7.1+0.2>-7		-6.80-0.2=-7<-6		7.1-0.2=-6<7
	if(PolePairs<0)
	{
		PolePairs=-PolePairs;
		Symbol=-1;
	}
	if(PolePairs>0)
	{
		if((char)(PolePairs+0.21f)>(char)PolePairs)
			MotorParm.MOTOR_POLE_PAIRS=(unsigned char)(PolePairs+0.2f);
		else if((char)(PolePairs-0.21f)<(char)PolePairs)
			MotorParm.MOTOR_POLE_PAIRS=(unsigned char)(PolePairs);
		else 
			return -1;
		if(Symbol==-1)
			MotorParm.MOTOR_ENCODER_DIR=(signed char)-1U;
		else
			MotorParm.MOTOR_ENCODER_DIR=1;
	}
	FOC_SetOpenLoopVector(a,0.2);//转子先吸合1s;
	HAL_Delay(1000);
	//稳定，读取电角度零值对应机械位置
	MotorParm.ELECTRICAL_OFFSET=-((float)MotorParm.FOC_encoder_raw/MT6816_CPR*FOC_2PI*MotorParm.MOTOR_ENCODER_DIR*MotorParm.MOTOR_POLE_PAIRS);
	usb_print("ELECTRICAL_OFFSET:%f\r\n",MotorParm.ELECTRICAL_OFFSET);
	return MotorParm.MOTOR_POLE_PAIRS;
}


/* 直接在dq坐标系给电压，vd/vq仍然使用你当前的标幺值 */
void FOC_SetDQVoltage(float vd, float vq, float theta)
{
    float v_alpha;
    float v_beta;

    FOC_InvPark(vd, vq, theta, &v_alpha, &v_beta);
    FOC_SVPWM(v_alpha, v_beta);
}
//电感电阻计算

void InductorAndRS_Check()
{
	uint32_t arr;
	//第1步测量相电阻，母线12V,给3V,25%占空比（实际是有50%占空比偏置，故+3V为75%占空比）
	arr = __HAL_TIM_GET_AUTORELOAD(&htim1)+1U;
	__HAL_TIM_SET_COMPARE( &htim1, 	TIM_CHANNEL_1,arr/2);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_2,arr/2);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_3,arr/2);
	HAL_Delay(100);
	usb_print("%f,%f,%f,%f\r\n",ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c,MotorParm.Rs);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_1,(uint32_t)(0.75f * (float)arr) );
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_2,arr/2);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_3,arr/2);
	HAL_Delay(1000);//等待1s
	//R=V/I，此处I单位为mA
	MotorParm.Rs=(float)((float)3*1000/ADC_parm.I_a);
	usb_print("%f,%f,%f,%f\r\n",ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c,MotorParm.Rs);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_1,arr/2);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_2,(uint32_t)(0.75f * (float)arr) );
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_3,arr/2);
	HAL_Delay(1000);//等待1s
	//R=V/I，此处I单位为mA
	MotorParm.Rs=(float)((float)3*1000/ADC_parm.I_b);
	usb_print("%f,%f,%f,%f\r\n",ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c,MotorParm.Rs);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_1,arr/2);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_2,arr/2);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_3,(uint32_t)(0.75f * (float)arr) );
	HAL_Delay(1000);//等待1s
	//R=V/I，此处I单位为mA
	MotorParm.Rs=(float)((float)3*1000/ADC_parm.I_c);
	usb_print("%f,%f,%f,%f\r\n",ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c,MotorParm.Rs);
	__HAL_TIM_SET_COMPARE( &htim1, 	TIM_CHANNEL_1,arr/2);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_2,arr/2);
	__HAL_TIM_SET_COMPARE( &htim1,	TIM_CHANNEL_3,arr/2);	
	HAL_Delay(1000);//等待1s
	
	
	//测量相电感，方法强吸合1s（让定子到位），弱吸合500ms（防止定子松动同时降低电流），再给脉冲用粗略计算 Ld=Vd*dt/dI,(时间太短置标注位给ADC中断处理)
//	
//	FOC_SetOpenLoopVector(0,0.2);
//	HAL_Delay(1000);//等待1s
//	FOC_SetOpenLoopVector(0,0.05);
//	HAL_Delay(500);//等待1s
//	//置位待ADC完成任务返回
//	MotorParm.open_L_check_flag=1;
//	float IQ_start,ID_start;
//	IQ_start=I_q;
//	ID_start=I_d;
//	while(MotorParm.open_L_check_flag)
//	{
//		usb_print("%d\r\n",MotorParm.open_L_check_flag);
//	}
//	HAL_Delay(100);
//	usb_print("%f,%f    OK\r\n",900.0f*(MotorParm.L_check_I_d-ID_start)/0.0005f,900.0f*(MotorParm.L_check_I_q-IQ_start)/0.0005f);
	//float V_alpha, V_beta;
	//FOC_InvPark(2,0,0,&V_alpha,&V_beta);//d轴给电压
	/*
 * ============================
 * 测量 d 轴电感 Ld
 * ============================
 *
 * 流程：
 * 1. 0.20 对齐转子
 * 2. 降到0.05保持，等待电流进入稳态
 * 3. ADC中断产生0.05 -> 0.30阶跃
 * 4. 一个ADC周期后测量 ΔId
 */

FOC_SetOpenLoopVector(0.0f, 0.20f);
HAL_Delay(1000);

/* 保持转子，同时降低稳态电流 */
FOC_SetOpenLoopVector(0.0f, 0.05f);
HAL_Delay(500);

/* 启动ADC中断中的电感测量状态机 */
MotorParm.open_L_check_flag = 1;

/* 等待ADC完成一个阶跃测试 */
while(MotorParm.open_L_check_flag)
{
    usb_print("OKOK");
}


/*
 * 你的SVPWM中：
 *
 * alpha/beta标幺值直接对应 Vbus。
 *
 * 0.05 -> 0.30
 *
 * ΔV = (0.30 - 0.05) * 12V
 *    = 3.0V
 */
float delta_V =(0.30f - 0.05f) * MotorParm.VBUS;


/*
 * 当前foc.c里面定义：
 *
 * FOC_DT = 0.00005f
 *
 * 即50us。
 *
 * 这里暂时按ADC中断实际也是50us处理。
 */
float delta_t = 0.00005f;


/*
 * L_check_I_d现在已经是：
 *
 * ΔId = Id_after - Id_before
 *
 * 单位还是 mA
 */
float delta_Id_mA = MotorParm.L_check_I_d;


/*
 * 电流变化必须足够大，
 * 否则噪声会让计算结果失真。
 */
if(fabsf(delta_Id_mA) > 5.0f)
{
    /*
     * L = ΔV * Δt / ΔI
     *
     * Id单位是mA，
     * 所以乘0.001转换成A。
     *
     * 最终MotorParm.Ld单位：H
     */
    MotorParm.Ld = fabsf(delta_V * delta_t /(delta_Id_mA * 0.001f) );
		HAL_Delay(100);
    usb_print( "dV=%f V, dt=%f us, dId=%f mA, Ld=%f mH\r\n", 
				delta_V,
        delta_t * 1000000.0f,
        delta_Id_mA,
        MotorParm.Ld * 1000.0f
    );
}
else
{
    MotorParm.Ld = 0.0f;
		HAL_Delay(100);
    usb_print(
        "Ld test failed: dId too small = %f mA\r\n",
        delta_Id_mA
    );
}


/* 测量结束，撤掉开环电压 */
FOC_SetOpenLoopVector(0.0f, 0.0f);

HAL_Delay(100);
	

/* =========================================================
 * 测量 Lq
 * ========================================================= */

/*
 * 重新加强一次d轴对齐，保证转子位置稳定
 */
FOC_SetOpenLoopVector(0.0f, 0.20f);
HAL_Delay(500);

/*
 * 降到小d轴保持电压
 *
 * 此时：
 * Vd = 0.05
 * Vq = 0
 */
FOC_SetOpenLoopVector(0.0f, 0.05f);
HAL_Delay(300);


/*
 * 启动q轴电感测量
 *
 * ADC中断会执行：
 *
 * Vd = 0.05
 * Vq = 0
 *
 *        ↓
 *
 * Vd = 0.05
 * Vq = 0.25
 *
 * 下一次ADC得到 ΔIq
 */
MotorParm.open_Lq_check_flag = 1;


/* 等待ADC状态机完成 */
while(MotorParm.open_Lq_check_flag)
{
  usb_print("OKOK");
}


/*
 * q轴电压变化量
 *
 * Vq:
 * 0 -> 0.25
 *
 * VBUS = 12V时：
 * ΔVq = 0.25 * 12 = 3V
 */
float delta_Vq =0.25f * MotorParm.VBUS;


/*
 * 当前测试一个ADC周期
 * 你目前按50us使用
 */
float delta_t_q = 50.0e-6f;


/*
 * ADC中断已经计算：
 *
 * ΔIq = Iq_after - Iq_before
 *
 * 单位：mA
 */
float delta_Iq_mA = MotorParm.L_check_I_q;


/*
 * 电流变化太小就认为测试失败
 */
if(fabsf(delta_Iq_mA) > 5.0f)
{
    float delta_Iq_A =  fabsf(delta_Iq_mA) * 0.001f;

    /*
     * ---------------------------------------------------
     * 推荐算法：考虑相电阻 Rs
     *
     * ΔI = ΔV/R * (1 - exp(-R*dt/L))
     *
     * 反算：
     *
     * L = -R*dt /
     *     ln(1 - R*ΔI/ΔV)
     * ---------------------------------------------------
     */
		if( MotorParm.Rs<0)
		{
			 MotorParm.Rs= -MotorParm.Rs;
		}
    float Rs = MotorParm.Rs;

    /* 如果你的每相电阻确定就是6Ω，
       也可以调试阶段直接：
       float Rs = 6.0f;
    */

    float k = 1.0f - Rs * delta_Iq_A / delta_Vq;

    if((k > 0.0f) && (k < 1.0f))
    {
        MotorParm.Lq = -Rs * delta_t_q / logf(k);
				HAL_Delay(100);
        usb_print(
            "dVq=%f V, dt=%f us, dIq=%f mA, Lq=%f mH\r\n",
            delta_Vq,
            delta_t_q * 1000000.0f,
            delta_Iq_mA,
            MotorParm.Lq * 1000.0f
        );
    }
    else
    {
        MotorParm.Lq = 0.0f;
				HAL_Delay(100);
        usb_print(
            "Lq calc failed: k=%f, dIq=%f mA\r\n",
            k,
            delta_Iq_mA
        );
    }
}
else
{
    MotorParm.Lq = 0.0f;
		HAL_Delay(100);
    usb_print( "Lq test failed: dIq too small=%f mA\r\n",   delta_Iq_mA  );
}


/*
 * Ld/Lq全部测完，最后才撤掉电压
 */
FOC_SetOpenLoopVector(0.0f, 0.0f);

HAL_Delay(100);


}
/* =========================================================
 * 紧急停止
 * ========================================================= */

void FOC_PolePairDetect_Abort(PPDetect_t *detect)
{
    FOC_PWM_Stop();

    detect->state = PP_DETECT_ABORT;
}


/* =========================================================
 * 电流保护
 * 用真实 Ia/Ib 后，在 ADC ISR 中调用。
 * ========================================================= */

uint8_t FOC_PolePairDetect_CurrentProtect(  PPDetect_t *detect,  float ia, float ib,  float current_limit)
{
    if((fabsf(ia) > current_limit) ||(fabsf(ib) > current_limit))
    {
        FOC_PolePairDetect_Abort(detect);
        return 1;
    }

    return 0;
}



#include <math.h>
#include <stdint.h>

/*****************************************************************电流环*************************************************************************************
*/
float I_alpha = 0.0f;
float I_beta  = 0.0f;

float I_d = 0.0f;
float I_q = 0.0f;

float theta_m = 0.0f;
float theta_e = 0.0f;//电角度


#define FOC_DT 0.00005f	//时间周期50us
//角度归一化0-360（0-2PI）
float FOC_WrapAngle(float angle)
{
    while(angle >= FOC_2PI)
        angle -= FOC_2PI;
    while(angle < 0.0f)
        angle += FOC_2PI;
    return angle;
}


#define MT6816_LUT_ENABLE  0	//1LUT补偿

void FOC_UpdateElectricalAngle(void)
{
    float encoder_used;

    MotorParm.FOC_encoder_raw = MT6816_ReadOneAngle();

#if MT6816_LUT_ENABLE

    encoder_used =
        Encoder_GetCorrectedRaw(MotorParm.FOC_encoder_raw);

#else

    encoder_used =
        (float)MotorParm.FOC_encoder_raw;

#endif

    theta_m =encoder_used *FOC_2PI /(float)MT6816_CPR;

    theta_e = MotorParm.MOTOR_ENCODER_DIR * MotorParm.MOTOR_POLE_PAIRS *theta_m + MotorParm.ELECTRICAL_OFFSET;

    theta_e = FOC_WrapAngle(theta_e);
}

/***************************************/
FOC_PI_t PI_Id =
{
    .kp = 0.000221f,
    .ki = 0.942478f,

    .integral = 0.0f,

    .out_min = -0.40f,
    .out_max =  0.40f
};


FOC_PI_t PI_Iq =
{
    .kp = 0.000275f,
    .ki = 0.942478f,

    .integral = 0.0f,

    .out_min = -0.40f,
    .out_max =  0.40f
};
float FOC_PI_Run( FOC_PI_t *pi, float target, float feedback, float dt)
{
    float error;
    float output;
		if(motor_speed_rpm_filt>0)
		{
			//target-=motor_speed_rpm_filt/20
			pi->kp = 0.000157f,
			pi->ki=1.7f;
		}
		else
		{
			pi->kp = 0.000157f,
			pi->ki = 0.958f;
		}
    error = target - feedback;
    pi->integral +=pi->ki *error *dt;
    pi->integral = FOC_Limit(pi->integral,pi->out_min,pi->out_max);
    output =pi->kp * error +pi->integral;
	
    output =FOC_Limit(output,pi->out_min,pi->out_max);
    return output;
}

float error;
float FOC_PI_Run1( FOC_PI_t *pi, float target, float feedback, float dt)
{   
    float output;
    error = target - feedback;
    pi->integral +=pi->ki *error *dt;
    pi->integral = FOC_Limit(pi->integral,pi->out_min,pi->out_max);
    output =pi->kp * error +pi->integral;
    output =FOC_Limit(output,pi->out_min,pi->out_max);
    return output;
}

float Vd;
float Vq;
float Id_ref = 0.0f;
float Iq_ref = 000.0f;
float FOC_CurrentLoop(void)
{
    

    

    float V_alpha;
    float V_beta;
	
    Vd = FOC_PI_Run(&PI_Id, Id_ref,I_d, FOC_DT);
		Vq = FOC_PI_Run1( &PI_Iq,Iq_ref,I_q, FOC_DT);
		
    FOC_InvPark(Vd,Vq,theta_e,&V_alpha,&V_beta);

    FOC_SVPWM(V_alpha,V_beta);
	return Vq;
}



//float FOC_CurrentLoop(void)
//{
//    float Id_ref = 0.0f;
//    float Iq_ref = 100.0f;

//    float Vd;
//    float Vq;

//    float V_alpha;
//    float V_beta;

//    /* d轴完全关闭 */
//    Vd = 0.0f;

//    /* 只测试q轴P控制 */
//    Vq = PI_Iq.kp * (Iq_ref - I_q);

//    /*
//     * 先限制得非常小
//     */
////    Vq = FOC_Limit(
////        Vq,
////        -0.05f,
////         0.05f
////    );

//    FOC_InvPark(
//        Vd,
//        Vq,
//        theta_e,
//        &V_alpha,
//        &V_beta
//    );

//    FOC_SVPWM(
//        V_alpha,
//        V_beta
//    );
//		return Vq;

//}
//速度环
#define ENCODER_CPR    16384
#define SPEED_DT       0.005f       //1ms

volatile float motor_speed_rpm = 0.0f;
volatile float motor_speed_rpm_filt = 0.0f;


void FOC_SpeedCalculate(uint64_t encoder_raw)
{
    static uint64_t last_raw = 0;
    static uint8_t first = 1;

    int64_t delta;

    if(first)
    {
        last_raw = encoder_raw;
        first = 0;
        return;
    }

    delta = (int64_t)encoder_raw -
            (int64_t)last_raw;

    last_raw = encoder_raw;

    /* 处理跨零 */
    if(delta > ENCODER_CPR / 2)
        delta -= ENCODER_CPR;

    if(delta < -(ENCODER_CPR / 2))
        delta += ENCODER_CPR;

    /*
     * rpm =
     * delta / CPR / dt * 60
     */
    motor_speed_rpm = (float)delta *60.0f /((float)ENCODER_CPR * SPEED_DT);

    /* 简单低通 */
    motor_speed_rpm_filt +=
        0.1f *
        (motor_speed_rpm -
         motor_speed_rpm_filt);
}

typedef struct
{
    float kp;
    float ki;
		float kd;
    float integral;
    float out_limit;
} SpeedPI_t;

SpeedPI_t Speed_PI = {
    .kp = 0.25f,//1.8f,
    .ki = 0.9f,
		.kd =0.0f,// 6.1f,
    .integral = 0.0f,
    .out_limit = 1500.0f
};
float error_V=0.0f;
float last_error_V=0.0f;
float last_error_V2=0.0f;
float FOC_SpeedLoop(float speed_ref)
{
		int16_t limit_i=400;
//		if(speed_ref>1000||speed_ref<-1000)
//		{
//			Speed_PI.kp = 2.42f;//1.8f,
//			Speed_PI.ki = 25.3f;//2.2f;
//			Speed_PI.kd =15.0f;// 6.1f,
//			limit_i=300;
//		}
    error_V = -(speed_ref - motor_speed_rpm_filt);			//期望-实际=error
		Speed_PI.integral +=Speed_PI.ki *error_V *SPEED_DT;
		if(Speed_PI.integral>limit_i)
		{
			Speed_PI.integral=limit_i;
		}
		else if(Speed_PI.integral<-limit_i)
		{
			Speed_PI.integral=-limit_i;
		}
    Iq_ref=Speed_PI.kp * error_V+Speed_PI.integral+
					 Speed_PI.kd	*	(error_V-last_error_V);
    if(Iq_ref > Speed_PI.out_limit)
        Iq_ref = Speed_PI.out_limit;

    if(Iq_ref < -Speed_PI.out_limit)
        Iq_ref = -Speed_PI.out_limit;
		last_error_V2=last_error_V;
		last_error_V=error_V;
		
    return Iq_ref;
}

//typedef struct
//{
//    float kp;
//    float ki;
//		float kd;
//    float integral;
//    float out_limit;
//		long pos;
//} POSPI_t;
POSPI_t POS_PI = {
    .kp = 0.15f,
    .ki = 0.00055f,
		.kd = 0.0f,
    .integral = 0.0f,
    .out_limit =80.0f,
		.pos=0
};


void FOC_POSCalculate(uint16_t encoder_raw)
{
    static uint16_t last_raw = 0;
    static uint8_t first = 1;

    int32_t delta;

    if(first)
    {
        last_raw = encoder_raw;
        first = 0;
        return;
    }

    delta =
        (int32_t)encoder_raw -
        (int32_t)last_raw;

    /*
     * 正转跨零：
     * 16380 -> 3
     * 原始 delta = -16377
     * 修正后 delta = +7
     */
    if(delta < -(ENCODER_CPR / 2))
    {
        delta += ENCODER_CPR;
    }

    /*
     * 反转跨零：
     * 3 -> 16380
     * 原始 delta = +16377
     * 修正后 delta = -7
     */
    else if(delta > (ENCODER_CPR / 2))
    {
        delta -= ENCODER_CPR;
    }

    POS_PI.pos += delta;

    last_raw = encoder_raw;
}

float error_pos=0.0f;

float FOC_PosLoop(int64_t pos_ref)
{
   
    float speed_ref;

    error_pos = (pos_ref-POS_PI.pos );			//期望-实际=error
		POS_PI.integral+=error_pos*SPEED_DT;
		if(POS_PI.integral>1500)
		{
			POS_PI.integral=1500;
		}
		else	if(POS_PI.integral<-1500)
		{
			POS_PI.integral=-1500;
		}
    speed_ref=POS_PI.kp * error_pos+POS_PI.ki * POS_PI.integral;
	
    if(speed_ref > POS_PI.out_limit)
        speed_ref = POS_PI.out_limit;

    if(speed_ref < -POS_PI.out_limit)
        speed_ref = -POS_PI.out_limit;
		
   return FOC_SpeedLoop( speed_ref);
}
