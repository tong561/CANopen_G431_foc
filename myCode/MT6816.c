#include "MT6816.h"


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
