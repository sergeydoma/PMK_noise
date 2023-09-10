#include "main.h"//



void Blink_LED(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
	HAL_Delay(750);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
	HAL_Delay(250);
}


void LED_Link(uint8_t val) 
{
	uint8_t green = val & 1;
	uint8_t yellow = val & 2;

	HAL_GPIO_WritePin(LINK_GRN_GPIO_Port, LINK_GRN_Pin, (green != 0));
	HAL_GPIO_WritePin(LINK_YEL_GPIO_Port, LINK_YEL_Pin, (yellow != 0));
}


void LED_Fail(uint8_t val) 
{
	uint8_t green = val & 1;
	uint8_t red = val & 2;

	HAL_GPIO_WritePin(FAIL_GRN_GPIO_Port, FAIL_GRN_Pin, (green != 0));
	HAL_GPIO_WritePin(FAIL_RED_GPIO_Port, FAIL_RED_Pin, (red != 0));
}

uint32_t Nblinck (uint16_t mode, uint32_t color, uint16_t period, uint8_t Ch)
  {
//	const uint32_t grn = 0x00000f00; // зеленый цвет	
  static int ik[10]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  static uint32_t ledTemp[4];
	uint32_t out;

      
        if (ik[Ch] <= period *8)
        {ik[Ch]++;}
        else
        {ik[Ch] =0;}
 // led_rgb[adc_current_chan] = 0x1f;                                     
            if ((ik > 0) &      (ik[Ch]<= period))
//							led_rgb[k] = 0x00ff0000;
            {ledTemp[0] = color;ledTemp[1] = color;ledTemp[2] = color;ledTemp[3] = color;}   
             else if ((ik[Ch] > period) & (ik[Ch] <= period*2))
            {ledTemp[0] = 0;ledTemp[1] = 0;ledTemp[2] = 0;ledTemp[3] = 0;}   
             else if ((ik[Ch] > period*2) & (ik[Ch] <= period*3))
            {ledTemp[0] = 0;ledTemp[1] = color;ledTemp[2] = color;ledTemp[3] = color;}
             else if ((ik[Ch] > period*3) & (ik[Ch] <= period*4))
            {ledTemp[0] = 0;ledTemp[1] = 0;ledTemp[2] = 0;ledTemp[3] = 0;}
             else if ((ik[Ch] > period*4) & (ik[Ch] <= period*5))
            {ledTemp[0] = 0;ledTemp[1] = 0;ledTemp[2] = color;ledTemp[3] = color;}
             else if ((ik[Ch] > period*5) & (ik[Ch] <= period*6))
            {ledTemp[0] = 0;ledTemp[1] = 0;ledTemp[2] = 0;ledTemp[3] = 0;}
             else if ((ik[Ch] > period*6) & (ik[Ch] <= period*7))
            {ledTemp[0] = 0;ledTemp[1] = 0;ledTemp[2] = 0;ledTemp[3] = color;}
             else 
            {ledTemp[0] = 0;ledTemp[1] = 0;ledTemp[2] = 0;ledTemp[3] = 0;}

 switch (mode) 				//(mode&0x00FF) //230717
        {
        	case (_mode10P | _mode10P << 8):
						out = ledTemp[0];                
        		break;
        	case (_mode20P |_mode20P <<8) :
                out = ledTemp[1];  
        		break;
          case (_mode30P | _mode30P <<8):
                out = ledTemp[2];  
        		break;
        	default:
                out = ledTemp[3];  
        	break;
        } 
				return out;				
 }