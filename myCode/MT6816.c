#include "MT6816.h"
#include "foc.h"
#include "tim.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include <stddef.h>
#include <string.h>
#include "bspUSB.h"
unsigned short  MT6816_ReadOneAngle(void)
{
		SPI1->CR1 |= SPI_CR1_SPE;
		uint16_t	add_time=0;
		uint16_t 	angle=0;
	GPIOA->ODR&=~(0x01<<4);
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	 // 等待发送缓冲区可以写
    while ((SPI1->SR & SPI_SR_TXE) == 0)
		{
			if(add_time>20000)
				return	65534;
			add_time++;
			
		}
		add_time=0;
		SPI1->DR=0x8300;//发送读地址3
		while (!(SPI1->SR & SPI_SR_RXNE))
		{
			if(add_time>20000)
				return	65535;
			add_time++;
		}
		GPIOA->ODR|=(0x01<<4);
    angle = SPI1->DR<<6;
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		GPIOA->ODR&=~(0x01<<4);
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		 // 等待发送缓冲区可以写
    while ((SPI1->SR & SPI_SR_TXE) == 0)
		{
			if(add_time>20000)
				return	65534;
			add_time++;
		}
		add_time=0;
		SPI1->DR=0x8400;//发送读地址4
		while (!(SPI1->SR & SPI_SR_RXNE))
		{
			if(add_time>20000)
				return	65535;
			add_time++;
		}
		GPIOA->ODR|=(0x01<<4);
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		return angle|= SPI1->DR>>2;
}
/*
 * 读取编码器，同时检查 MT6816 是否返回异常值。
 * MT6816_ReadOneAngle() 返回 0~16383。
 */
uint8_t Encoder_Read(uint16_t *angle)
{
    uint16_t raw;
    raw = MT6816_ReadOneAngle();
    if(raw >= MT6816_CPR)
			return 0;
    *angle = raw;
    return 1;
}
/*
 * 计算两个 MT6816 采样点之间的增量。
 *
 * 自动处理：
 *
 * 16380 -> 3
 *
 * 或
 *
 * 3 -> 16380
 *
 * 的跨零问题。
 */
int Encoder_GetDelta(uint16_t now, uint16_t last)
{
    int32_t delta;
    delta = (int32_t)now - (int32_t)last;
    if(delta > (MT6816_CPR / 2))
    {
        delta -= MT6816_CPR;
    }
    else if(delta < -(MT6816_CPR / 2))
    {
        delta += MT6816_CPR;
    }
    return delta;
}

int16_t MT6816_ErrorLUT[MT6816_LUT_SIZE]; /* AI运行时误差表，单位为编码器raw计数。 */
static uint8_t MT6816_CalStatus=MT6816_CAL_NO_DATA; /* AI当前校准状态：0未就绪、1有效、2失败。 */
static uint8_t MT6816_LUT_Ready=0U;          /* AI标志：非零表示RAM误差表已初始化。 */
volatile uint8_t Encoder_AI_Calibrating=0U;   /* AI标志：非零表示校准接口正在独占开环控制。 */
/* AI内部结构：保存LUT版本、序号、定点误差和CRC校验值。 */
typedef struct
{
    uint32_t magic;                 /* AI数据有效标志。 */
    uint16_t version,size;          /* AI格式版本和LUT点数。 */
    uint32_t sequence;              /* AI数据序号，用于选择最新有效槽。 */
    int16_t lut[MT6816_LUT_SIZE];   /* AI整数编码器误差，单位为raw计数。 */
    uint32_t crc;                   /* AI数据CRC32校验值。 */
} Encoder_AI_FlashData_t;
/**
  * @brief AI内部函数：计算校准数据的CRC32校验值。
  * @param data:待计算数据的首地址。
  * @param length:待计算数据的字节数。
  * @retval 返回CRC32校验值。
  * 功能：用于判断Flash中的校准数据是否完整、有效。
  */
static uint32_t Encoder_AI_CRC32(const uint8_t *data,uint32_t length)
{
    uint32_t crc=0xFFFFFFFFUL;
    for(uint32_t i=0;i<length;i++)
    {
        crc^=data[i];
        for(uint8_t b=0;b<8;b++)
        {
            crc=(crc&1U)?((crc>>1)^0xEDB88320UL):(crc>>1);
        }
    }
    return crc^0xFFFFFFFFUL;
}
/**
  * @brief AI内部函数：读取并校验一个Flash存储槽。
  * @param address:Flash存储槽首地址。
  * @param data:用于接收Flash数据的数据结构指针。
  * @retval 返回1表示数据有效，返回0表示数据无效。
  * 功能：检查有效标志、版本、LUT长度和CRC，防止上电加载损坏数据。
  */
static uint8_t Encoder_AI_ReadSlot(uint32_t address,Encoder_AI_FlashData_t *data)
{
    memcpy(data,(const void *)address,sizeof(*data));
    if(data->magic!=MT6816_CAL_MAGIC||
			 data->version!=MT6816_CAL_VERSION||
			 data->size!=MT6816_LUT_SIZE)
			return 0U;
    return data->crc==Encoder_AI_CRC32((const uint8_t *)data,offsetof(Encoder_AI_FlashData_t,crc));
}
/**
  * @brief AI内部函数：加载已校验的Flash校准数据。
  * @param data:已通过校验的Flash数据结构指针。
  * @retval 无。
  * 功能：把Flash中的整数raw误差加载到RAM LUT供实时补偿使用。
  */
static void Encoder_AI_UseFlashData(const Encoder_AI_FlashData_t *data)
{
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)
			MT6816_ErrorLUT[i]=data->lut[i];
    MT6816_LUT_Ready=1U;
    MT6816_CalStatus=MT6816_CAL_OK;
}
/**
  * @brief AI内部函数：保存编码器误差LUT到Flash。
  * @param lut:待保存的128点整数误差表，单位为编码器raw计数。
  * @retval 返回1表示保存并回读校验成功，返回0表示Flash操作失败。
  * 功能：在两个Flash槽之间交替写入，保存整数raw误差表。
  */
#if MT6816_CAL_WRITE_FLASH
static uint8_t Encoder_AI_Save(const int16_t *lut)
{
    Encoder_AI_FlashData_t data,old;
    uint8_t va=Encoder_AI_ReadSlot(MT6816_CAL_BASE,&old); /* AI主槽有效标志。 */
    uint32_t address=MT6816_CAL_REV_BASE,seq=1U; /* AI本次目标槽地址和递增序号。 */
    if(va)//校验有效
    {
        seq=old.sequence+1U;
        address=MT6816_CAL_REV_BASE;
    }
    else if(Encoder_AI_ReadSlot(MT6816_CAL_REV_BASE,&old))
    {
        seq=old.sequence+1U;
        address=MT6816_CAL_BASE;
    }
    memset(&data,0xFF,sizeof(data));
    data.magic=MT6816_CAL_MAGIC;
    data.version=MT6816_CAL_VERSION;
    data.size=MT6816_LUT_SIZE;
    data.sequence=seq;
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)
    {
        data.lut[i]=lut[i];
    }
    data.crc=Encoder_AI_CRC32((const uint8_t *)&data,offsetof(Encoder_AI_FlashData_t,crc));
    FLASH_EraseInitTypeDef erase={0};
    uint32_t pageError=0U;
    erase.TypeErase=FLASH_TYPEERASE_PAGES;
    erase.Banks=FLASH_BANK_1;
    erase.Page=(address-FLASH_BASE)/MT6816_CAL_PAGE_SIZE;
    erase.NbPages=1U;
    HAL_FLASH_Unlock();
    if(HAL_FLASHEx_Erase(&erase,&pageError)!=HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0U;
    }
    const uint64_t *words=(const uint64_t *)&data;
    for(uint32_t i=0;i<sizeof(data)/8U;i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,address+i*8U,words[i])!=HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0U;
        }
    }
    HAL_FLASH_Lock();
    return Encoder_AI_ReadSlot(address,&old);
}
#endif
/**
  * @brief AI接口：查询编码器LUT校准状态。
  * @param 无。
  * @retval 返回0表示未就绪，返回1表示LUT有效，返回2表示校准或Flash操作失败。
  * 功能：供外部判断编码器补偿表当前是否可以使用。
  */
uint8_t Encoder_AI_GetCalibrationStatus(void)
{
    return MT6816_CalStatus;
}
/**
  * @brief AI接口：初始化编码器误差LUT。
  * @param 无。
  * @retval 返回0表示无有效数据，返回1表示LUT加载或校准成功，返回2表示校准或保存失败。
  * 功能：上电先读取Flash中的有效LUT；没有有效数据时，暂时独占开环控制，采集编码器误差，生成128点LUT并掉电保存。
  */
uint8_t Encoder_AI_InitCalibration(uint8_t force_refresh)
{
    Encoder_AI_FlashData_t dataA,dataB;
    uint8_t validA=Encoder_AI_ReadSlot(MT6816_CAL_BASE,&dataA); /* AI主槽校验结果。 */
    uint8_t validB=Encoder_AI_ReadSlot(MT6816_CAL_REV_BASE,&dataB); /* AI备份槽校验结果。 */
    if((validA||validB)&&!force_refresh)
    {
        if(validA&&validB)Encoder_AI_UseFlashData(dataA.sequence>=dataB.sequence?&dataA:&dataB);
        else Encoder_AI_UseFlashData(validA?&dataA:&dataB);
        return MT6816_CAL_OK;
    }
#if !MT6816_CAL_AUTO //如果未打开自动校准
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)MT6816_ErrorLUT[i]=0;
    MT6816_LUT_Ready=1U;
    MT6816_CalStatus=MT6816_CAL_NO_DATA;
    return MT6816_CAL_NO_DATA;
#else
    if(MotorParm.MOTOR_POLE_PAIRS==0U||MotorParm.open_L_check_flag||MotorParm.open_Lq_check_flag)//极对数无效或有其它校验任务
    {
        MT6816_CalStatus=MT6816_CAL_ERR;//状态设置为错误
				usb_print("ERROR=%d\r\n",MT6816_CalStatus);
        return MT6816_CAL_ERR;
    }
    uint8_t wasEnabled=(GPIOB->ODR&(1U<<11))?1U:0U; /* 保存校准前驱动使能状态。 */
    uint8_t wasRunning=(htim1.Instance->CCER&(TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E))?1U:0U; /* 保存校准前PWM状态。 */
    int64_t bin_sum_x[MT6816_LUT_SIZE]={0},bin_sum_y[MT6816_LUT_SIZE]={0};
    uint16_t count[MT6816_LUT_SIZE]={0};
    int64_t fit_sum_x=0,fit_sum_y=0,fit_sum_xx=0,fit_sum_xy=0;
    uint32_t fit_count=0U;
    uint16_t last=MT6816_ReadOneAngle(); /* 上次有效编码器角度raw值。 */
    int32_t unwrapped=0; /* 相对采样起点的机械位移。 */
    float commanded=0.0f;
    uint32_t pole_pairs=(uint32_t)MotorParm.MOTOR_POLE_PAIRS; /* 极对数仅用于电角/机械角换算。 */
    uint32_t calTurns=MT6816_CAL_TURNS; /* 采集机械圈数由宏独立控制。 */
    Encoder_AI_Calibrating=1U;
    FOC_SVPWM(0.0f,0.0f);
    FOC_PWM_Start();
    DRV8313_ENABLE();
    HAL_Delay(20);
    for(uint32_t step=0;step<MT6816_CAL_STEPS*pole_pairs*calTurns;step++)
    {
        commanded=(float)(step%MT6816_CAL_STEPS)*FOC_2PI/MT6816_CAL_STEPS;
        FOC_SetOpenLoopVector(commanded,MT6816_CAL_AMP);//开环吸合转子到位
        HAL_Delay(MT6816_CAL_SETTLE_MS);
        uint16_t raw=MotorParm.FOC_encoder_raw; /* 本次MT6816原始角度。直接取中断读取角度值 */
        if(raw>=MT6816_CPR)//编码器数值无效
					continue;
        int32_t delta=(int32_t)raw-(int32_t)last; /* 相邻样本的带符号角度增量。 */
        if(delta>(MT6816_CPR/2))//跨零处理
					delta-=MT6816_CPR;
        else if(delta<-(MT6816_CPR/2))
					delta+=MT6816_CPR;
        last=raw;
        unwrapped+=delta;
        int32_t command_turn=(int32_t)(step/((uint32_t)MT6816_CAL_STEPS*pole_pairs));
        int32_t command_phase=(int32_t)(step%((uint32_t)MT6816_CAL_STEPS*pole_pairs));
        int32_t x=command_turn*(int32_t)MT6816_CAL_STEPS+command_phase;
        int32_t y=unwrapped;
        int64_t x_centered=x;
        int64_t y_centered=(int64_t)y;
        fit_sum_x+=x_centered;
        fit_sum_y+=y_centered;
        fit_sum_xx+=x_centered*x_centered;
        fit_sum_xy+=x_centered*y_centered;
        fit_count++;
        /* LUT横轴使用运行时相同的原始编码器raw相位。 */
        uint16_t index=(uint16_t)(((uint32_t)raw*MT6816_LUT_SIZE)/MT6816_CPR);
        bin_sum_x[index]+=x_centered;
        bin_sum_y[index]+=y_centered;
        count[index]++;
    }
		//采样完成恢复现场
    FOC_SetOpenLoopVector(commanded,0.0f);
    HAL_Delay(20);
    if(!wasEnabled)DRV8313_DISABLE();
    if(!wasRunning)FOC_PWM_Stop();
    Encoder_AI_Calibrating=0U;
    uint32_t total=0U;
    uint16_t validBins=0U;
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)
    {
        total+=count[i];
        if(count[i]>0U)validBins++;
    }
    if(total<(uint32_t)MT6816_LUT_SIZE*calTurns/2U || validBins<(MT6816_LUT_SIZE*3U)/4U)
    {
        MT6816_CalStatus=MT6816_CAL_ERR;
        return MT6816_CAL_ERR;
    }
    /* 整数最小二乘拟合强托轨迹直线，允许起点偏置和实测斜率偏差。 */
    int64_t denominator=(int64_t)fit_count*fit_sum_xx-fit_sum_x*fit_sum_x;
    int64_t slope_num=(int64_t)fit_count*fit_sum_xy-fit_sum_x*fit_sum_y;
    if(fit_count<2U || denominator<=0 || slope_num<=0)
    {
        MT6816_CalStatus=MT6816_CAL_ERR;
        return MT6816_CAL_ERR;
    }
    /* 以千分之一为定点比例，斜率理论值为 CPR/(steps*pole_pairs)。 */
    int64_t slope_q=(slope_num*1000+denominator/2)/denominator;
    if(slope_q<((1000LL*MT6816_CPR)/(MT6816_CAL_STEPS*pole_pairs))*7/10 || slope_q>((1000LL*MT6816_CPR)/(MT6816_CAL_STEPS*pole_pairs))*13/10)
    {
        MT6816_CalStatus=MT6816_CAL_ERR;
        return MT6816_CAL_ERR;
    }
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)
    {
        if(count[i]>0U)
        {
            int64_t x_mean_q=(bin_sum_x[i]*1000+(bin_sum_x[i]>=0?count[i]/2U:-(int64_t)count[i]/2U))/count[i];
            int64_t y_mean_q=(bin_sum_y[i]*1000+(bin_sum_y[i]>=0?count[i]/2U:-(int64_t)count[i]/2U))/count[i];
            int64_t residual_q=y_mean_q-(slope_q*x_mean_q)/1000;
            MT6816_ErrorLUT[i]=(int16_t)((residual_q>=0)?(residual_q+500)/1000:(residual_q-500)/1000);
        }
        else MT6816_ErrorLUT[i]=0;
    }
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)
    {
        if(count[i]==0U)
        {
            uint16_t left=i,right=i,tries=0U; /* 缺样分箱左右最近的有效点。 */
            do{left=(left+MT6816_LUT_SIZE-1U)%MT6816_LUT_SIZE;tries++;}while(count[left]==0U&&tries<MT6816_LUT_SIZE);
            tries=0U;
            do{right=(right+1U)%MT6816_LUT_SIZE;tries++;}while(count[right]==0U&&tries<MT6816_LUT_SIZE);
            if(count[left]==0U||count[right]==0U)
            {
                MT6816_CalStatus=MT6816_CAL_ERR;
                return MT6816_CAL_ERR;
            }
            uint16_t span=(uint16_t)((right+MT6816_LUT_SIZE-left)%MT6816_LUT_SIZE);
            if(span==0U)
            {
                MT6816_CalStatus=MT6816_CAL_ERR;
                return MT6816_CAL_ERR;
            }
            uint16_t position=(uint16_t)((i+MT6816_LUT_SIZE-left)%MT6816_LUT_SIZE);
            int32_t numerator=(int32_t)MT6816_ErrorLUT[left]*(span-position)+(int32_t)MT6816_ErrorLUT[right]*position;
            MT6816_ErrorLUT[i]=(int16_t)((numerator>=0)?(numerator+span/2U)/span:(numerator-span/2U)/span);
        }
    }
    int32_t offset_sum=0;
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)offset_sum+=MT6816_ErrorLUT[i];
    int16_t offset=(int16_t)((offset_sum>=0)?(offset_sum+MT6816_LUT_SIZE/2U)/MT6816_LUT_SIZE:(offset_sum-MT6816_LUT_SIZE/2U)/MT6816_LUT_SIZE);
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)MT6816_ErrorLUT[i]-=offset;
#if MT6816_CAL_WRITE_FLASH
    if(!Encoder_AI_Save(MT6816_ErrorLUT))
    {
        MT6816_CalStatus=MT6816_CAL_ERR;
        return MT6816_CAL_ERR;
    }
#endif
    MT6816_LUT_Ready=1U;
    MT6816_CalStatus=MT6816_CAL_OK;
    return MT6816_CAL_OK;
#endif
}
/* AI接口：输入MT6816原始角度0~16383，输出整数补偿角度。 */
/* 功能：按128点误差LUT修正编码器周期误差；LUT尚未加载时先使用零误差表。 */
uint16_t Encoder_GetCorrectedRaw(uint16_t raw)
{
    uint16_t index=(uint16_t)(raw>>7);
    uint16_t next=(uint16_t)((index+1U)%MT6816_LUT_SIZE);
    uint16_t sub=(uint16_t)(raw&0x7FU);
    if(!MT6816_LUT_Ready)
    {
        for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)MT6816_ErrorLUT[i]=0;
        MT6816_LUT_Ready=1U;
    }
    int32_t numerator=(int32_t)MT6816_ErrorLUT[index]*(128U-sub)+(int32_t)MT6816_ErrorLUT[next]*sub;
    int32_t err=(numerator>=0)?(numerator+64)/128:(numerator-64)/128;
    int32_t corrected=(int32_t)raw-err;
    if(corrected>=(int32_t)MT6816_CPR)corrected-=(int32_t)MT6816_CPR;
    if(corrected<0)corrected+=(int32_t)MT6816_CPR;
    return (uint16_t)corrected;
}
float Encoder_RawToRad(float raw)
{
    return raw*6.28318530717958647692f/(float)MT6816_CPR;
}






