#ifndef _MT6816_H_
#define _MT6816_H_
#include "main.h"
/* MT6816 Ò»È¦ */
#define MT6816_CPR                   16384L
unsigned short  MT6816_ReadOneAngle(void);
uint8_t Encoder_Read(uint16_t *angle);//¶ÁMT6816´ø×´Ì¬
int Encoder_GetDelta(uint16_t now, uint16_t last);
/* LUT ²¹³¥ */
float Encoder_GetCorrectedRaw(uint16_t raw);
float Encoder_RawToRad(float raw);

#endif 
