#include "MT6816.h"


uint16_t  MT6816_ReadOneAngle()
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
				return	-1;
			add_time++;
			
		}
		add_time=0;
		SPI1->DR=0x8300;//发送读地址3
		while (!(SPI1->SR & SPI_SR_RXNE))
		{
			if(add_time>20000)
				return	-2;
			add_time++;
		}
		GPIOA->ODR|=(0x01<<4);
    angle = SPI1->DR<<5;
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		GPIOA->ODR&=~(0x01<<4);
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		 // 等待发送缓冲区可以写
    while ((SPI1->SR & SPI_SR_TXE) == 0)
		{
			if(add_time>20000)
				return	-1;
			add_time++;
		}
		add_time=0;
		SPI1->DR=0x8400;//发送读地址4
		while (!(SPI1->SR & SPI_SR_RXNE))
		{
			if(add_time>20000)
				return	-2;
			add_time++;
		}
		GPIOA->ODR|=(0x01<<4);
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		return angle|= SPI1->DR;

}