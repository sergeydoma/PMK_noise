#include "main.h"

extern GPIO led_gpio[10];
extern uint32_t  led_rgb[10];

//void LedCH(uint16_t kDelay, uint16_t* ledCh)
//{
//	static int i, j;
//	if (i < kDelay) { i++;}
//	else 
//	{
//		i=0;
//		for(j=0;j<10;j++)
//    {
//				
//				if (((led_rgb[j]>>16) & 0xff)>2) {ledCh[j] = 1;} // RED
//				else if (((led_rgb[j]>>8) & 0xff)>2) {ledCh[j] = 2;} // grn
//				else if ((led_rgb[j] & 0xff)>2) {ledCh[j] = 3;} // blu
//    }
//	}
//  }
