#ifndef _MT6816_H_
#define _MT6816_H_
#include "main.h"
/* MT6816 一圈 */
#define MT6816_CPR                   16384L
unsigned short  MT6816_ReadOneAngle(void);
uint8_t Encoder_Read(uint16_t *angle);//读MT6816带状态
int Encoder_GetDelta(uint16_t now, uint16_t last);
/* LUT 补偿 */
float Encoder_GetCorrectedRaw(uint16_t raw);
float Encoder_RawToRad(float raw);
/* AI接口：无输入；返回0表示无保存数据，返回1表示LUT可用，返回2表示校准或存储失败。 */
/* 功能：读取掉电保存的LUT；无有效数据时自动开环采集、生成128点补偿表并写入Flash。 */
uint8_t Encoder_AI_InitCalibration(void);
/* AI接口：无输入；返回当前状态，0=未校准，1=已就绪，2=失败。 */
/* 功能：查询编码器LUT加载或校准状态。 */
uint8_t Encoder_AI_GetCalibrationStatus(void);

#endif 
