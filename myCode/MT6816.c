#include "MT6816.h"
#include "foc.h"
#include "tim.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include <stddef.h>
#include <string.h>
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
#define MT6816_LUT_SIZE 128                 /* AI误差表点数，覆盖编码器整圈。 */
#define MT6816_CAL_MAGIC 0x41494C55UL       /* AI参数有效标志，用于识别Flash中的校准数据。 */
#define MT6816_CAL_VERSION 1U               /* AI参数格式版本，结构变化时递增。 */
#define MT6816_CAL_PAGE 62U                 /* AI保存区起始页号，STM32G431每页2KB。 */
#define MT6816_CAL_PAGE_SIZE 0x800U         /* AI Flash页大小，单位字节。 */
#define MT6816_CAL_BASE (FLASH_BASE + MT6816_CAL_PAGE * MT6816_CAL_PAGE_SIZE) /* AI主存储槽首地址。 */
#define MT6816_CAL_REV_BASE (MT6816_CAL_BASE + MT6816_CAL_PAGE_SIZE) /* AI备份存储槽首地址。 */
#define MT6816_CAL_TURNS 8U                 /* AI开环采集圈数，用于多圈平均。 */
#define MT6816_CAL_STEPS 512U               /* AI每圈开环角度更新步数。 */
#define MT6816_CAL_AMP 0.10f                /* AI开环电压矢量幅值，归一化范围0~1。 */
#define MT6816_CAL_AUTO 1U                  /* AI自动校准开关，1=无有效数据时自动采集。 */
#define MT6816_CAL_OK 1U                    /* AI状态码：校准表已加载或保存成功。 */
#define MT6816_CAL_NO_DATA 0U               /* AI状态码：Flash无有效校准数据。 */
#define MT6816_CAL_ERR 2U                   /* AI状态码：采集、校验或Flash操作失败。 */
static float MT6816_ErrorLUT[MT6816_LUT_SIZE]; /* AI运行时误差表，单位为编码器raw计数。 */
static uint8_t MT6816_CalStatus=MT6816_CAL_NO_DATA; /* AI当前校准状态：0未就绪、1有效、2失败。 */
static uint8_t MT6816_LUT_Ready=0U;          /* AI标志：非零表示RAM误差表已初始化。 */
volatile uint8_t Encoder_AI_Calibrating=0U;   /* AI标志：非零表示校准接口正在独占开环控制。 */
/* AI内部结构：保存LUT版本、序号、定点误差和CRC校验值。 */
typedef struct
{
    uint32_t magic;                 /* AI数据有效标志。 */
    uint16_t version,size;          /* AI格式版本和LUT点数。 */
    uint32_t sequence;              /* AI数据序号，用于选择最新有效槽。 */
    int16_t lut[MT6816_LUT_SIZE];  /* AI定点编码器误差，扩大16倍保存。 */
    uint32_t crc;                   /* AI数据CRC32校验值。 */
} Encoder_AI_FlashData_t;
/* AI内部函数：计算数据CRC，输入为数据指针和字节数，返回CRC32校验值。 */
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
/* AI内部函数：读取并校验一个Flash槽，输入地址和数据结构指针，返回1有效、0无效。 */
static uint8_t Encoder_AI_ReadSlot(uint32_t address,Encoder_AI_FlashData_t *data)
{
    memcpy(data,(const void *)address,sizeof(*data));
    if(data->magic!=MT6816_CAL_MAGIC||data->version!=MT6816_CAL_VERSION||data->size!=MT6816_LUT_SIZE)return 0U;
    return data->crc==Encoder_AI_CRC32((const uint8_t *)data,offsetof(Encoder_AI_FlashData_t,crc));
}
/* AI内部函数：把已校验的Flash数据复制到RAM LUT，输入为有效Flash数据指针，无返回值。 */
static void Encoder_AI_UseFlashData(const Encoder_AI_FlashData_t *data)
{
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)MT6816_ErrorLUT[i]=(float)data->lut[i]/16.0f;
    MT6816_LUT_Ready=1U;
    MT6816_CalStatus=MT6816_CAL_OK;
}
/* AI内部函数：把128点误差表写入交替Flash页，输入误差表指针，返回保存成功标志。 */
static uint8_t Encoder_AI_Save(const float *lut)
{
    Encoder_AI_FlashData_t data,old;
    uint8_t va=Encoder_AI_ReadSlot(MT6816_CAL_BASE,&old); /* AI主槽有效标志。 */
    uint32_t address=MT6816_CAL_REV_BASE,seq=1U; /* AI本次目标槽地址和递增序号。 */
    if(va)
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
        float value=lut[i]*16.0f;
        if(value>32767.0f)value=32767.0f;
        if(value<-32768.0f)value=-32768.0f;
        data.lut[i]=(int16_t)value;
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
/* AI接口：无输入，返回0未就绪、1校准数据有效、2校准或Flash操作失败。 */
/* 功能：查询当前编码器LUT的加载或校准状态。 */
uint8_t Encoder_AI_GetCalibrationStatus(void)
{
    return MT6816_CalStatus;
}
/* AI接口：无输入，返回0无有效数据、1 LUT已就绪、2校准或保存失败。 */
/* 功能：优先从Flash加载LUT；无有效数据时独占开环、采集一圈误差并保存128点LUT。 */
uint8_t Encoder_AI_InitCalibration(void)
{
    Encoder_AI_FlashData_t dataA,dataB;
    uint8_t validA=Encoder_AI_ReadSlot(MT6816_CAL_BASE,&dataA); /* AI主槽校验结果。 */
    uint8_t validB=Encoder_AI_ReadSlot(MT6816_CAL_REV_BASE,&dataB); /* AI备份槽校验结果。 */
    if(validA||validB)
    {
        if(validA&&validB)Encoder_AI_UseFlashData(dataA.sequence>=dataB.sequence?&dataA:&dataB);
        else Encoder_AI_UseFlashData(validA?&dataA:&dataB);
        return MT6816_CAL_OK;
    }
#if !MT6816_CAL_AUTO
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)MT6816_ErrorLUT[i]=0.0f;
    MT6816_LUT_Ready=1U;
    MT6816_CalStatus=MT6816_CAL_NO_DATA;
    return MT6816_CAL_NO_DATA;
#else
    if(MotorParm.open_L_check_flag||MotorParm.open_Lq_check_flag)
    {
        MT6816_CalStatus=MT6816_CAL_ERR;
        return MT6816_CAL_ERR;
    }
    uint8_t wasEnabled=(GPIOB->ODR&(1U<<11))?1U:0U; /* 保存校准前驱动使能状态。 */
    uint8_t wasRunning=(htim1.Instance->CCER&(TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E))?1U:0U; /* 保存校准前PWM状态。 */
    float sum[MT6816_LUT_SIZE]={0},count[MT6816_LUT_SIZE]={0}; /* 各角度分箱的误差和、采样数。 */
    uint16_t last=MT6816_ReadOneAngle(); /* 上次有效编码器角度raw值。 */
    int32_t unwrapped=0; /* 跨过编码器零点后的连续角度计数。 */
    float commanded=0.0f,commandOrigin=0.0f; /* 当前开环角度及起始相位差，单位rad。 */
    float fitSumX=0.0f,fitSumY=0.0f,fitSumXX=0.0f,fitSumXY=0.0f,fitCount=0.0f; /* 多圈开环角度与编码器角度线性拟合累计量。 */
    Encoder_AI_Calibrating=1U;
    FOC_SVPWM(0.0f,0.0f);
    FOC_PWM_Start();
    DRV8313_ENABLE();
    HAL_Delay(20);
    for(uint32_t step=0;step<MT6816_CAL_STEPS*MT6816_CAL_TURNS;step++)
    {
        commanded=(float)(step%MT6816_CAL_STEPS)*FOC_2PI/MT6816_CAL_STEPS;
        FOC_SetOpenLoopVector(commanded,MT6816_CAL_AMP);
        HAL_Delay(2);
        uint16_t raw=MT6816_ReadOneAngle(); /* 本次MT6816原始角度。 */
        if(raw>=MT6816_CPR)continue;
        int32_t delta=(int32_t)raw-(int32_t)last; /* 相邻样本的带符号角度增量。 */
        if(delta>(MT6816_CPR/2))delta-=MT6816_CPR;
        else if(delta<-(MT6816_CPR/2))delta+=MT6816_CPR;
        last=raw;
        unwrapped+=delta;
        uint16_t index=(uint16_t)(((uint32_t)raw*MT6816_LUT_SIZE)/MT6816_CPR); /* 原始角度对应的LUT分箱。 */
        if(index>=MT6816_LUT_SIZE)index=0U;
        float x=(float)step*FOC_2PI/MT6816_CAL_STEPS; /* 累计开环电角度指令，单位rad。 */
        float y=(float)unwrapped*FOC_2PI/MT6816_CPR; /* 连续编码器机械角度，单位rad。 */
        if(step==0U)commandOrigin=y-x;
        fitSumX+=x;
        fitSumY+=y;
        fitSumXX+=x*x;
        fitSumXY+=x*y;
        fitCount+=1.0f;
        sum[index]+=y-x-commandOrigin;
        count[index]+=1.0f;
    }
    FOC_SetOpenLoopVector(commanded,0.0f);
    HAL_Delay(20);
    if(!wasEnabled)DRV8313_DISABLE();
    if(!wasRunning)FOC_PWM_Stop();
    Encoder_AI_Calibrating=0U;
    float total=0.0f; /* 有效采样总数，用于检查采集覆盖率。 */
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)total+=count[i];
    if(total<MT6816_LUT_SIZE*MT6816_CAL_TURNS*0.5f||fitCount<2.0f)
    {
        MT6816_CalStatus=MT6816_CAL_ERR;
        return MT6816_CAL_ERR;
    }
    float slope=(fitCount*fitSumXY-fitSumX*fitSumY)/(fitCount*fitSumXX-fitSumX*fitSumX); /* 开环指令到编码器角度的比例拟合值。 */
    float intercept=(fitSumY-slope*fitSumX)/fitCount; /* 开环与编码器的固定相位差，单位rad。 */
    if(slope<0.8f||slope>1.2f)
    {
        MT6816_CalStatus=MT6816_CAL_ERR;
        return MT6816_CAL_ERR;
    }
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)
    {
        MT6816_ErrorLUT[i]=count[i]>0.0f?sum[i]/count[i]-(intercept+(slope-1.0f)*(float)i*FOC_2PI/MT6816_LUT_SIZE):0.0f;
    }
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)
    {
        if(count[i]==0.0f)
        {
            uint16_t left=i,right=i,tries=0U; /* 缺样分箱左右最近的有效点。 */
            do{left=(left+MT6816_LUT_SIZE-1U)%MT6816_LUT_SIZE;tries++;}while(count[left]==0.0f&&tries<MT6816_LUT_SIZE);
            tries=0U;
            do{right=(right+1U)%MT6816_LUT_SIZE;tries++;}while(count[right]==0.0f&&tries<MT6816_LUT_SIZE);
            if(count[left]==0.0f||count[right]==0.0f)
            {
                MT6816_CalStatus=MT6816_CAL_ERR;
                return MT6816_CAL_ERR;
            }
            float span=(float)((right+MT6816_LUT_SIZE-left)%MT6816_LUT_SIZE); /* 插值跨越的LUT点数。 */
            float position=(float)((i+MT6816_LUT_SIZE-left)%MT6816_LUT_SIZE); /* 当前缺样点到左侧有效点的距离。 */
            MT6816_ErrorLUT[i]=MT6816_ErrorLUT[left]+(MT6816_ErrorLUT[right]-MT6816_ErrorLUT[left])*position/span;
        }
    }
    float offset=0.0f; /* LUT误差均值，用于去除固定角度偏置。 */
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)offset+=MT6816_ErrorLUT[i];
    offset/=MT6816_LUT_SIZE;
    for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)MT6816_ErrorLUT[i]-=offset;
    if(!Encoder_AI_Save(MT6816_ErrorLUT))
    {
        MT6816_CalStatus=MT6816_CAL_ERR;
        return MT6816_CAL_ERR;
    }
    MT6816_LUT_Ready=1U;
    MT6816_CalStatus=MT6816_CAL_OK;
    return MT6816_CAL_OK;
#endif
}
/* AI接口：输入MT6816原始角度0~16383，输出线性插值后的补偿角度。 */
/* 功能：按128点误差LUT修正编码器周期误差；LUT尚未加载时先使用零误差表。 */
float Encoder_GetCorrectedRaw(uint16_t raw)
{
    uint16_t index=(uint16_t)(raw>>7); /* 当前raw角度对应的LUT左侧节点。 */
    uint16_t next=(uint16_t)((index+1U)%MT6816_LUT_SIZE); /* 插值右侧节点，末点后回到0。 */
    uint16_t sub=(uint16_t)(raw&0x7FU); /* LUT节点间的raw计数偏移。 */
    float frac=(float)sub/128.0f; /* 节点间插值比例，范围0~1。 */
    if(!MT6816_LUT_Ready)
    {
        for(uint16_t i=0;i<MT6816_LUT_SIZE;i++)MT6816_ErrorLUT[i]=0.0f;
        MT6816_LUT_Ready=1U;
    }
    float err=MT6816_ErrorLUT[index]+frac*(MT6816_ErrorLUT[next]-MT6816_ErrorLUT[index]); /* 插值得到的raw误差。 */
    float corrected=(float)raw-err; /* 减去误差后的编码器角度raw值。 */
    if(corrected>=MT6816_CPR)corrected-=MT6816_CPR;
    if(corrected<0.0f)corrected+=MT6816_CPR;
    return corrected;
}
float Encoder_RawToRad(float raw)
{
    return raw*6.28318530717958647692f/(float)MT6816_CPR;
}
