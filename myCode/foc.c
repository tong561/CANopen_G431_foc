#include "foc.h"
#include "bspUSB.h"
#include "myADC.h"
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
				├───────│ Clarke  							    │
 Ib ──┘              └──────┬──────┘
																			│
																		Iα Iβ
																			│
																			▼
													┌──────────┐
		Encoder ──θe───	│ Park  					 	  │
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



#include "foc.h"
#include "tim.h"
#include "MT6816.h"

#include <math.h>


/* =========================================================
 * 内部函数
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
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
}


/* =========================================================
 * 开环 SVPWM
 *
 * amplitude:
 *
 * 0.01 = 很弱
 * 0.02 = 弱
 * 0.05 = 已经比较明显
 *
 * 第一次不要给大。
 * ========================================================= */

void FOC_SetOpenLoopVector(float electrical_angle, float amplitude)
{
    float alpha;//a轴
    float beta;	//b轴
		//三相电压
    float va;
    float vb;
    float vc;

    float vmax;
    float vmin;
    float offset;

    float duty_a;
    float duty_b;
    float duty_c;

    uint32_t arr;
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
极对数检测

*/

#define LAPS_NUMBLE	10
void NumberOfPolePairs_Check()
{
	float a=0; //旋转角
	static long start_pos=0;
	FOC_SetOpenLoopVector(a,0.2);//转子先吸合1s;
	HAL_Delay(1000);
	start_pos =POS_PI.pos;
	for(unsigned int i=0;i<360*LAPS_NUMBLE;i++)
	{
		a+=FOC_2PI/360;
		if(a>FOC_2PI)
			a-=FOC_2PI;
		FOC_SetOpenLoopVector(a,0.1);//转子先吸合1s
		HAL_Delay(1);
		usb_print("Electrical angle:%f,%f,%f,%f,%f%%\r\n",a,ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c,(float)i*100/(360*LAPS_NUMBLE));
	}
	HAL_Delay(1000);
	usb_print("Electrical angle:%f,%f,%f,%f,%f%%,%f\r\n",a,ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c,100,(float)LAPS_NUMBLE/((float)(POS_PI.pos-start_pos)/MT6816_CPR));
	HAL_Delay(1000);
}





/* =========================================================
 * 开始检测
 * ========================================================= */

void FOC_PolePairDetect_Start(PPDetect_t *detect)
{
    uint32_t now;
    now = HAL_GetTick();
    detect->state = PP_DETECT_ALIGN;
    detect->state_tick  = now;
    detect->update_tick = now;
    detect->encoder_last = 0;
    detect->mechanical_count_acc = 0;
    detect->electrical_angle = 0.0f;
    detect->electrical_travel = 0.0f;
    detect->pole_pairs_float = 0.0f;
    detect->pole_pairs = 0;
    detect->direction = 0;


    /*
     * 先建立 θe = 0 的弱磁场。
     */
    FOC_SetOpenLoopVector( 0.0f,  PP_DETECT_PWM_AMPLITUDE);
    FOC_PWM_Start();
}


/* =========================================================
 * 自动测极对数任务
 *
 * 放在 while(1) 中不停调用。
 * ========================================================= */

void FOC_PolePairDetect_Task(PPDetect_t *detect)
{
    uint32_t now;
    uint16_t encoder;
    int32_t delta;
    int32_t mechanical_abs;
    float pole_float;
    uint8_t pole_round;
    now = HAL_GetTick();
    /* -----------------------------------------------------
     * IDLE / DONE / ERROR / ABORT 不处理
     * ----------------------------------------------------- */
    if((detect->state == PP_DETECT_IDLE)  ||
       (detect->state == PP_DETECT_DONE)  ||
       (detect->state == PP_DETECT_ERROR) ||
       (detect->state == PP_DETECT_ABORT))
    {
        return;
    }
    /* -----------------------------------------------------
     * 第一阶段：
     * 固定 θe = 0
     * 让转子慢慢吸到一个稳定位置。
     * ----------------------------------------------------- */
    if(detect->state == PP_DETECT_ALIGN)
    {
        if((now - detect->state_tick)  < PP_DETECT_ALIGN_TIME_MS)
        {
            return;
        }
        /*
         * 吸合稳定后才记录起始位置。
         * 这样初始吸合产生的机械位移不会被算进极对数。
         */
        if(!Encoder_Read(&encoder))
        {
            FOC_PWM_Stop();
            detect->state = PP_DETECT_ERROR;
            return;
        }
        detect->encoder_last = encoder;
        detect->mechanical_count_acc = 0;
        detect->electrical_angle = 0.0f;
        detect->electrical_travel = 0.0f;
        detect->update_tick = now;
        detect->state = PP_DETECT_RUNNING;
        return;
    }
    /* -----------------------------------------------------
     * 第二阶段
     * 缓慢旋转电角度。
     * ----------------------------------------------------- */
    if(detect->state == PP_DETECT_RUNNING)
    {
        if((now - detect->update_tick)  < PP_DETECT_UPDATE_TIME_MS)
        {
            return;
        }
        detect->update_tick = now;
        /*
         * 读取当前位置
         */
        if(!Encoder_Read(&encoder))
        {
            FOC_PWM_Stop();
            detect->state = PP_DETECT_ERROR;
            return;
        }
        /*
         * 累积机械位移
         */
        delta = Encoder_GetDelta( encoder,detect->encoder_last);
        detect->mechanical_count_acc += delta;
        detect->encoder_last = encoder;
        /*
         * 电角度继续向前旋转
         */
        detect->electrical_angle +=  PP_DETECT_STEP_RAD;
        detect->electrical_travel += PP_DETECT_STEP_RAD;
        /*
         * 角度限制在 0~2π。
         */
        if(detect->electrical_angle >= FOC_2PI)
        {
            detect->electrical_angle -= FOC_2PI;
        }


        /*
         * 更新定子磁场
         */
        FOC_SetOpenLoopVector(  detect->electrical_angle, PP_DETECT_PWM_AMPLITUDE);


        /*
         * 已经走够设定的电周期
         */
        if(detect->electrical_travel >=((float)PP_DETECT_ELEC_CYCLES * FOC_2PI))
        {
            detect->state = PP_DETECT_END_WAIT;
            detect->state_tick = now;
            detect->update_tick = now;
        }

        return;
    }


    /* -----------------------------------------------------
     * 第三阶段：
     *
     * 已经走完电周期，但继续固定最后的磁场。
     *
     * 给机械系统一点时间消除滞后。
     * ----------------------------------------------------- */

    if(detect->state == PP_DETECT_END_WAIT)
    {
        /*
         * 最后几百 ms 仍然继续累计编码器位移，
         * 防止转子略微落后于旋转磁场。
         */
        if((now - detect->update_tick)>= PP_DETECT_UPDATE_TIME_MS)
        {
            detect->update_tick = now;


            if(!Encoder_Read(&encoder))
            {
                FOC_PWM_Stop();

                detect->state = PP_DETECT_ERROR;

                return;
            }


            delta = Encoder_GetDelta( encoder, detect->encoder_last);


            detect->mechanical_count_acc += delta;

            detect->encoder_last = encoder;
        }


        if((now - detect->state_tick)  < PP_DETECT_END_WAIT_MS)
        {
            return;
        }


        /*
         * 测量完成，关闭输出。
         */
        FOC_PWM_Stop();


        /* 获取机械位移绝对值 */
        mechanical_abs = detect->mechanical_count_acc;


        if(mechanical_abs < 0)
        {
            mechanical_abs = -mechanical_abs;
            detect->direction = -1;
        }
        else
        {
            detect->direction = 1;
        }


        /*
         * 转子运动太少：
         *
         * 通常表示 PWM amplitude 太小，
         * 电机没有跟着定子磁场转。
         */
        if(mechanical_abs < 150)
        {
            detect->state = PP_DETECT_ERROR;

            return;
        }


        /*
         *             N_elec × Encoder_CPR
         * polePair = ------------------------
         *                mechanical_count
         */
        pole_float = ((float)PP_DETECT_ELEC_CYCLES *  (float)MT6816_CPR)/ (float)mechanical_abs;
        detect->pole_pairs_float = pole_float;


        /*
         * 四舍五入到整数。
         */
        pole_round = (uint8_t)(pole_float + 0.5f);


        /*
         * 基本合法性检查。
         */
        if((pole_round < 1) ||(pole_round > 64))
        {
            detect->state = PP_DETECT_ERROR;

            return;
        }


        /*
         * 理论计算结果应该很接近整数。
         *
         * 比如：
         *
         * 6.92 -> 7，可以接受
         * 7.08 -> 7，可以接受
         *
         * 7.48 -> 不可靠
         */
        if(fabsf(pole_float - (float)pole_round) > 0.35f)
        {
            detect->state = PP_DETECT_ERROR;

            return;
        }


        detect->pole_pairs = pole_round;

        detect->state = PP_DETECT_DONE;

        return;
    }
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
 *
 * 后面有真实 Ia/Ib 后，在 ADC ISR 中调用。
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

////#define MOTOR_POLE_PAIRS       7U
#define TEST_MECH_TURNS        5U
#define TEST_TOTAL_POINTS      (MOTOR_POLE_PAIRS * TEST_MECH_TURNS)

/* 一个电周期分成360步，每步1°电角度 */
#define TEST_ELEC_STEPS        360U

/* 每一步延时 */
#define TEST_STEP_DELAY_MS     5U

/* 到稳定点以后等待 */
#define TEST_STABLE_DELAY_MS   500U

/* 稳定以后取10次平均 */
#define TEST_SAMPLE_NUM        10U
#define TEST_SAMPLE_DELAY_MS   5U
/* ============================================================
 * 稳定后读取多次MT6816并平均
 *
 * 不能直接普通平均raw，因为可能在0/16383附近。
 * ============================================================ */
static uint16_t Encoder_ReadAverage(void)
{
    uint16_t base;
    uint16_t raw;

    int32_t delta;
    int32_t delta_sum = 0;

    int32_t average_raw;

    base = MT6816_ReadOneAngle();

    for(uint16_t i = 0; i < TEST_SAMPLE_NUM; i++)
    {
        raw = MT6816_ReadOneAngle();

        delta = Encoder_GetDelta(raw, base);

        delta_sum += delta;

        HAL_Delay(TEST_SAMPLE_DELAY_MS);
    }


    average_raw = (int32_t)base + delta_sum / TEST_SAMPLE_NUM;


    /* 处理0~16383范围 */
    while(average_raw >= MT6816_CPR)
    {
        average_raw -= MT6816_CPR;
    }

    while(average_raw < 0)
    {
        average_raw += MT6816_CPR;
    }


    return (uint16_t)average_raw;
}


/* ============================================================
 * 自动运行5机械圈并统计
 * ============================================================ */
void FOC_StablePointTest(void)
{
    float a = 0.0f;

    uint16_t raw_start;
    uint16_t raw_last;
    uint16_t raw_now;

    int32_t delta;
    int32_t mechanical_count_acc = 0;

    float delta_abs;
    float expected_delta;

    float error;
    float error_abs;

    float error_sum = 0.0f;
    float max_error = 0.0f;

    float step_sum = 0.0f;


    /*
     * 7对极：
     *
     * 相邻同电角度稳定点理论距离
     *
     * 16384 / 7
     * = 2340.571 counts
     */
    expected_delta =(float)MT6816_CPR /(float)MOTOR_POLE_PAIRS;


    /* ===========================
     * 启动
     * =========================== */

    DRV8313_ENABLE();

    FOC_PWM_Start();


    /*
     * 首先固定在0电角度
     */
    FOC_SetOpenLoopVector( 0.0f,0.2);


    /* 等第一次吸合稳定 */
    HAL_Delay(1000);


    raw_start = Encoder_ReadAverage();
    raw_last  = raw_start;


    usb_print("\r\n===== Stable Point Test =====\r\n" );

    usb_print("Start Raw=%u\r\n" "ExpectedStep=%.3f\r\n",  raw_start,expected_delta);


    /*
     * CSV标题
     */
    usb_print( "Point,Turn,Raw,Delta,AbsDelta," "Error,MechAcc,Ia,Ib,Ic\r\n" );


    /* ========================================================
     * 一共运行35个电周期
     *
     * 35 / 7 = 5机械圈
     * ======================================================== */

    for(uint16_t point = 1; point <= TEST_TOTAL_POINTS;point++)
    {

        /* ==========================================
         * 完整旋转1个电周期
         * ========================================== */

        for(uint16_t step = 0;step < TEST_ELEC_STEPS; step++)
        {
            a +=  FOC_2PI /(float)TEST_ELEC_STEPS;


            /*
             * 保持0~2PI
             */
            if(a >= FOC_2PI)
            {
                a -= FOC_2PI;
            }


            FOC_SetOpenLoopVector(a,0.2 );

            HAL_Delay(TEST_STEP_DELAY_MS);
        }


        /* ==========================================
         * 已到下一个稳定点
         *
         * 保持当前磁场500ms
         * ========================================== */

        HAL_Delay(TEST_STABLE_DELAY_MS);


        /* ==========================================
         * 读取稳定位置
         * ========================================== */

        raw_now = Encoder_ReadAverage();


        /* ==========================================
         * 计算相邻稳定点机械变化
         * ========================================== */

        delta = Encoder_GetDelta( raw_now,raw_last );
        mechanical_count_acc += delta;
        delta_abs = fabsf((float)delta);


        /*
         * 理论应该是：
         *
         * 2340.571 count
         */
        error =delta_abs -expected_delta;


        error_abs = fabsf(error);


        error_sum += error_abs;

        step_sum += delta_abs;


        if(error_abs > max_error)
        {
            max_error = error_abs;
        }


        /* ==========================================
         * 输出本次结果
         * ========================================== */

        usb_print(
            "%u,%.3f,%u,%ld,"
            "%.2f,%.2f,%ld,"
            "%.1f,%.1f,%.1f\r\n",

            point,

            (float)point /
            (float)MOTOR_POLE_PAIRS,

            raw_now,

            delta,

            delta_abs,

            error,

            mechanical_count_acc,

            ADC_parm.I_a,
            ADC_parm.I_b,
            ADC_parm.I_c
        );


        raw_last = raw_now;
    }


    /* ========================================================
     * 测试结束
     * ======================================================== */

    FOC_PWM_Stop();
    DRV8313_DISABLE();


    /* 实际机械圈数 */
    float measured_mech_turns =fabsf((float)mechanical_count_acc) /(float)MT6816_CPR;


    /*
     * 总共主动运行35个电周期
     */
    float measured_pole_pairs =(float)TEST_TOTAL_POINTS / measured_mech_turns;


    /*
     * 平均相邻稳定点距离
     */
    float average_step =step_sum /(float)TEST_TOTAL_POINTS;


    /*
     * 平均绝对误差
     */
    float average_error = error_sum /(float)TEST_TOTAL_POINTS;


    /*
     * 五圈以后应该回到原始位置附近
     */
    int32_t closure_error = Encoder_GetDelta( raw_last, raw_start );


    usb_print("\r\n===== RESULT =====\r\n" );
    usb_print(
        "Points=%u\r\n"
        "ExpectedMechTurns=%u\r\n"
        "MechAcc=%ld\r\n"
        "MeasuredMechTurns=%.6f\r\n"
        "PolePairsEstimate=%.6f\r\n"
        "ExpectedStep=%.3f\r\n"
        "AverageStep=%.3f\r\n"
        "AverageAbsError=%.3f\r\n"
        "MaxError=%.3f\r\n"
        "StartRaw=%u\r\n"
        "EndRaw=%u\r\n"
        "ClosureError=%ld\r\n",

        TEST_TOTAL_POINTS,
        TEST_MECH_TURNS,

        mechanical_count_acc,

        measured_mech_turns,
        measured_pole_pairs,

        expected_delta,
        average_step,

        average_error,
        max_error,

        raw_start,
        raw_last,
        closure_error
    );
}

void FOC_FindElectricalOffset(void)
{
    uint16_t raw;
    float corrected_raw;
    float mech_angle;
    float elec_without_offset;
    float offset;

    DRV8313_ENABLE();
    FOC_PWM_Start();

    /*
     * 定子磁场固定在电角度 0
     */
    FOC_SetOpenLoopVector(0.0f, 0.20f);

    /*
     * 等待转子吸合
     */
    HAL_Delay(1500);

    /*
     * 读取编码器
     */
    raw = MT6816_ReadOneAngle();

#if MT6816_LUT_ENABLE
    corrected_raw = Encoder_GetCorrectedRaw(raw);
#else
    corrected_raw = (float)raw;
#endif

    /*
     * 机械角
     */
    mech_angle =
        corrected_raw
        * FOC_2PI
        / (float)MT6816_CPR;

    /*
     * 不带 offset 的电角度
     */
    elec_without_offset =
        MOTOR_ENCODER_DIR
        * MOTOR_POLE_PAIRS
        * mech_angle;

    elec_without_offset =
        FOC_WrapAngle(elec_without_offset);

    /*
     * 因为现在人为规定定子电角度 = 0
     *
     * 0 = elec_without_offset + offset
     */
    offset =
        FOC_WrapAngle(-elec_without_offset);

    usb_print(
        "\r\n===== Electrical Offset =====\r\n"
        "Raw=%u\r\n"
        "CorrectedRaw=%.3f\r\n"
        "ThetaM=%.6f rad\r\n"
        "ThetaM=%.3f deg\r\n"
        "ElecNoOffset=%.6f rad\r\n"
        "ElecNoOffset=%.3f deg\r\n"
        "OFFSET=%.6f rad\r\n"
        "OFFSET=%.3f deg\r\n",
        raw,
        corrected_raw,
        mech_angle,
        mech_angle * 180.0f / 3.14159265359f,
        elec_without_offset,
        elec_without_offset * 180.0f / 3.14159265359f,
        offset,
        offset * 180.0f / 3.14159265359f
    );

    FOC_PWM_Stop();
    DRV8313_DISABLE();
}


/******************************电流环**************************************
*/
float I_alpha = 0.0f;
float I_beta  = 0.0f;

float I_d = 0.0f;
float I_q = 0.0f;

float theta_m = 0.0f;
float theta_e = 0.0f;

uint16_t FOC_encoder_raw;//电机编码器值
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

    FOC_encoder_raw = MT6816_ReadOneAngle();

#if MT6816_LUT_ENABLE

    encoder_used =
        Encoder_GetCorrectedRaw(FOC_encoder_raw);

#else

    encoder_used =
        (float)FOC_encoder_raw;

#endif

    theta_m =encoder_used *FOC_2PI /(float)MT6816_CPR;

    theta_e = MOTOR_ENCODER_DIR * MOTOR_POLE_PAIRS *theta_m + ELECTRICAL_OFFSET;

    theta_e = FOC_WrapAngle(theta_e);
}

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

void FOC_SVPWM(float alpha, float beta)
{
    float va;
    float vb;
    float vc;

    float vmax;
    float vmin;
    float offset;

    float duty_a;
    float duty_b;
    float duty_c;

    uint32_t arr;


    /* ===============================
     * Alpha/Beta -> 三相
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
     * 转换成 0~1 Duty占空比
     * =============================== */

    duty_a = 0.5f + va;
    duty_b = 0.5f + vb;
    duty_c = 0.5f + vc;


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

    __HAL_TIM_SET_COMPARE( &htim1,TIM_CHANNEL_3,(uint32_t)(duty_c * (float)arr)
    );
}

/***************************************/
FOC_PI_t PI_Id =
{
    .kp = 0.000157f,
    .ki = 0.958f,

    .integral = 0.0f,

    .out_min = -0.40f,
    .out_max =  0.40f
};


FOC_PI_t PI_Iq =
{
    .kp = 0.000157f,
    .ki = 0.958f,

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
    
    float output;
    error = target - feedback;
    pi->integral +=pi->ki *error *dt;
    pi->integral = FOC_Limit(pi->integral,pi->out_min,pi->out_max);
    output =pi->kp * error +pi->integral;
    output =FOC_Limit(output,pi->out_min,pi->out_max);
    return output;
}
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
     * alpha/beta -> abc
     */
    va = alpha;

    vb = -0.5f * alpha  + 0.8660254038f * beta;

    vc = -0.5f * alpha- 0.8660254038f * beta;


    /*
     * SPWM:
     * 不做 vmax/vmin 零序注入
     */
    duty_a = 0.5f + va;
    duty_b = 0.5f + vb;
    duty_c = 0.5f + vc;


    /*
     * 防止超过合法 duty
     */
    duty_a = FOC_Limit(duty_a, 0.02f, 0.98f);
    duty_b = FOC_Limit(duty_b, 0.02f, 0.98f);
    duty_c = FOC_Limit(duty_c, 0.02f, 0.98f);


    arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        (uint32_t)(duty_a * (float)arr)
    );

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_2,
        (uint32_t)(duty_b * (float)arr)
    );

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_3,
        (uint32_t)(duty_c * (float)arr)
    );
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
    .kp = 10.8f,//1.8f,
    .ki = 20.1f,
		.kd =10.0f,// 6.1f,
    .integral = 0.0f,
    .out_limit = 1500.0f
};
float error_V=0.0f;
float last_error_V=0.0f;
float last_error_V2=0.0f;
float FOC_SpeedLoop(float speed_ref)
{
		int16_t limit_i=1000;
    float iq_ref;
		if(speed_ref>1000||speed_ref<-1000)
		{
			Speed_PI.kp = 2.42f;//1.8f,
			Speed_PI.ki = 25.3f;//2.2f;
			Speed_PI.kd =15.0f;// 6.1f,
			limit_i=300;
		}
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
