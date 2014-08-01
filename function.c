/*
 * function.c
 *
 *  Created on: 2014Äê8ÔÂ1ÈÕ
 *      Author: zli
 */
 
#include "function.h"

void _delay_us(uint32_t time )
{
uint32_t temp=time,i;
for(i=0;i<temp;i++)
	{
    __asm{NOP};__asm{NOP};
   }
}

void _delay_ms(uint32_t time )
{
	_delay_us(2600);
}
