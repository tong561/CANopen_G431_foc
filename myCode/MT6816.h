#ifndef _MT6816_H_
#define _MT6816_H_
#include "main.h"
#define MT6816_LUT_SIZE 128                 /* AI误差表点数，覆盖编码器整圈。 */
#define MT6816_CAL_MAGIC 0x41494C55UL       /* AI参数有效标志，用于识别Flash中的校准数据。 */
#define MT6816_CAL_VERSION 2U               /* AI参数格式版本；当前LUT直接保存raw误差，不再乘16。 */
#define MT6816_CAL_PAGE 62U                 /* AI保存区起始页号，STM32G431每页2KB。 */
#define MT6816_CAL_PAGE_SIZE 0x800U         /* AI Flash页大小，单位字节。0x800=2048 */
#define MT6816_CAL_BASE (FLASH_BASE + MT6816_CAL_PAGE * MT6816_CAL_PAGE_SIZE) /* AI主存储槽首地址。flash起始地址+偏移地址 */
#define MT6816_CAL_REV_BASE (MT6816_CAL_BASE + MT6816_CAL_PAGE_SIZE) /* AI备份存储槽首地址。 前地址+1page*/
#define MT6816_CAL_TURNS 4U                 /* AI强托采集的机械圈数，与电机极对数独立。 */
#define MT6816_CAL_STEPS 128U               /* AI每圈开环角度更新步数。 */
#define MT6816_CAL_AMP 0.10f                /* AI开环电压矢量幅值，归一化范围0~1。 */
#define MT6816_CAL_AUTO 1U                  /* AI自动校准开关，1=无有效数据时自动采集。 */
#define MT6816_CAL_WRITE_FLASH 0U           /* 1=保存校准表到Flash；0=仅更新RAM。 */
#define MT6816_CAL_SETTLE_MS 10U            /* 每个开环角度命令的稳定等待毫秒数。 */
#define MT6816_CAL_OK 1U                    /* AI状态码：校准表已加载或保存成功。 */
#define MT6816_CAL_NO_DATA 0U               /* AI状态码：Flash无有效校准数据。 */
#define MT6816_CAL_ERR 2U                   /* AI状态码：采集、校验或Flash操作失败。 */
/* MT6816 一圈 */
#define MT6816_CPR                   16384L
unsigned short  MT6816_ReadOneAngle(void);
uint8_t Encoder_Read(uint16_t *angle);//读MT6816带状态
int Encoder_GetDelta(uint16_t now, uint16_t last);
/* LUT 补偿 */
uint16_t Encoder_GetCorrectedRaw(uint16_t raw);

float Encoder_RawToRad(float raw);
/**
  * @brief AI接口：初始化编码器误差LUT。
  * @param 无。
  * @retval 返回0表示无有效数据，返回1表示LUT加载或校准成功，返回2表示校准或保存失败。
  * 功能：上电读取Flash；无有效数据时自动开环采集、生成128点补偿表并保存。
  */
uint8_t Encoder_AI_InitCalibration(uint8_t force_refresh);
/**
  * @brief AI接口：查询编码器LUT校准状态。
  * @param 无。
  * @retval 返回0表示未就绪，返回1表示LUT有效，返回2表示失败。
  * 功能：供外部判断编码器补偿表是否可用。
  */
uint8_t Encoder_AI_GetCalibrationStatus(void);
extern int16_t MT6816_ErrorLUT[MT6816_LUT_SIZE];
#endif 
