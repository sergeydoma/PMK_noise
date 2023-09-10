#include "main.h"
#include "cmsis_os.h"
static _Bool button[10];
extern uint16_t currentTime;
extern _chlMode cTime[10]; 
extern uint8_t chMode[10];
uint8_t tempPush; 
//_Led_Blinck ledBlinck;
extern led_mon leds[10];
extern GPIO switch_gpio[10];
extern uint32_t  led_rgb[10];
extern uint16_t arrWord[0x400];
// не нажата ни одна кнопка
//extern uint8_t mode;// test
extern int currentPB;

 _Bool All_buttons_NoPush()
{
  _Bool Out;
  Out = 0;	
  for(int i=0;i <10; i++)
  {
		Out = HAL_GPIO_ReadPin(switch_gpio[i].port, switch_gpio[i].pin) == 0;	// out = 1 если кнопка не нажата	
    if (Out == 0){return 0;}	// if (Out == 1){return 0;}
  }
 return 1;  									// return 0;
}

//+++++++++++++++++++++++++++++++++++++++++++


_Bool DeltaBool(uint16_t delay) // Проверка не нажата хотя бы одна кнопка в течении заданного времени
{
  static _Bool Numb;
  static uint16_t curr = 0;
  
		Numb = All_buttons_NoPush();
    curr ++;   //currentTime++;
		if (Numb == 0)
		{
			curr = 0;
		}			
    if (curr >= delay)
    {   
			curr = 0;
			return 1; // на протяжении времени кнопка не была нажата
		}          
	
	return 0;
} 


//_Bool oneButtomPush() // нажата хотябы одна кнопка
//{
//_Bool Out;
//  Out = 0;
//  button[0]= 0;//HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin);
//  button[1]= 0;//HAL_GPIO_ReadPin(BT2_GPIO_Port, BT2_Pin);
//  button[2]= 0;//HAL_GPIO_ReadPin(BT3_GPIO_Port, BT3_Pin);
//  button[3]= 0;//HAL_GPIO_ReadPin(BT4_GPIO_Port, BT4_Pin);
//  button[4]= 0;//HAL_GPIO_ReadPin(BT5_GPIO_Port, BT5_Pin); // ???? ?????? USART3
//  button[5]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);            //HAL_GPIO_ReadPin(BT6_GPIO_Port, BT6_Pin); // ???? ?????? USART3
//  button[6]= 0;//HAL_GPIO_ReadPin(BT7_GPIO_Port, BT7_Pin);
//  button[7]= 0;//HAL_GPIO_ReadPin(BT8_GPIO_Port, BT8_Pin);
//  button[8]= 0;//HAL_GPIO_ReadPin(BT9_GPIO_Port, BT9_Pin);
//  button[9]= 0;//HAL_GPIO_ReadPin(BT10_GPIO_Port, BT10_Pin);
// for(int i =0;i <10;i++)
//  {
//    Out = Out | button[i];
//  }
// return Out;
//}
// Селекция нажатия кнопки по длительности

// Вариант 2
uint8_t timePush (uint16_t delayPush, uint8_t numChannel)
{
		static _Bool begin[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		static int current[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};		 
		_Bool button;
		uint8_t out = 0;
	
		button =  HAL_GPIO_ReadPin(switch_gpio[numChannel].port, switch_gpio[numChannel].pin) == 0; // Состояние кнопки 1 если не нажата
	if (button) 
	{
		begin[numChannel] =1;
	}
	if ((button == 0)&(begin[numChannel]))
	{ current[numChannel]++;
		if (current[numChannel] >= delayPush)
		{			
			out = 2;
			current[numChannel] = 0;
			begin[numChannel] = 0;
			
		}		
	}
	else if ((button == 1)&(begin[numChannel] ==1))
	{
		if (current[numChannel] <= (delayPush>>4))
		{
			out = 0;
			current[numChannel] = 0;
			
		}
		else if (current[numChannel] <= delayPush )
		{
			out = 1;
			current[numChannel] = 0;
			
		}
	}
	return out;
}




_Bool timeNoPush (uint16_t delay, uint8_t numChannel) // 
  {
		static int current = 0;
		_Bool button;
    if (current < delay)
    {
      current ++;
			button =  HAL_GPIO_ReadPin(switch_gpio[numChannel].port, switch_gpio[numChannel].pin) == 0;
      if(button == 0){current = 0; return 0;}         
    }
    else
    {
			current = 0; return 1; // кнопка в течении заданного времени не нажималась
    }
   return 0;  //иначе ругается
}
  
  
  
  
              
  //*****************************************************************************
uint8_t ModeCH (uint8_t nCh, _Bool* Alarm, uint16_t* wordSet) // wordSet массив типа arrWord
  {

//	static uint8_t mode[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//	static uint8_t out;
		uint8_t out;
//	static uint8_t temp;	
		
		if (wordSet[nCh+40] == 0)  // Режим работы канала 
    {
			Alarm[nCh+50] = 1; 
          
      tempPush = timePush (4000, nCh);    // Селектор длительности нажатия
	
        switch (tempPush)
        {
          case 0:
          wordSet[nCh+40] = 0; // monitor off
          break;          
          case 1:
          wordSet[nCh+40] = 1; // redi monitor
          break;
          case 2:
          wordSet[nCh+40] = 3; // select 1
          break;
        }               
				led_rgb[nCh] = _Yellow; // желтый //Nblinck(2, 0x000f0000, 500);                                          
				out = 0;
        
    } 
      else if (wordSet[nCh+40] == 1)
      {
				Alarm[nCh+50] = 1; 
        tempPush = timePush (4000, nCh);    

        switch (tempPush)
        {
          case 0:
          wordSet[nCh+40] = 1; // monitor off
          break;          
          case 1:
          wordSet[nCh+40] = 2; // redi monitor
          break;
          case 2:
          wordSet[nCh+40] = 1; // select 1
          break;
        }
//////				if (((wordSet[nCh]==(_mode30P | _mode30P<<8))|(wordSet[nCh]==(_mode20P | _mode20P<<8)) | (wordSet[nCh] ==(_mode10P | _mode10P<<8)))==0)
//////				{
//////					wordSet[nCh] = _mode20P;
//////					
//////					if ((wordSet[nCh]) == _mode20P)
//////						{
//////							wordSet[nCh] = _mode20P | (_mode20P<<8);
//////						}
//////					
//////				} // 230717
        led_rgb[nCh] = Nblinck( wordSet[nCh], _Green, 500, nCh); // 
        
					out = 0;
        }
      
      else if (wordSet[nCh+40] == 2)
      {
        tempPush = timePush (4000, nCh); 
				
        switch (tempPush)
        {
          case 0:
          wordSet[nCh+40] = 2; // monitor off
          break;          
          case 1:
          wordSet[nCh+40] = 2; // redi monitor
					Alarm[nCh+50] = 1; // ручное квитирование обрыва кабеля
          break;
          case 2:
          wordSet[nCh+40] = 0; // select 1
					Alarm[nCh+50] = 1; // ручное квитирование обрыва кабеля
          break;
        }
        if (Alarm[nCh+60] == 0)
        {                    
//								wordSet[nCh+40] = 6;
								led_rgb[nCh] =  _Red; 
        }			
				else	
				{
				led_rgb[nCh] = _Green; //Nblinck( wordSet[nCh], _Green, 500); // 
				}
				out = 1;	 
    }
       else if (wordSet[nCh+40] == 3)
        {
        tempPush = timePush (4000, nCh);    

        switch (tempPush)
        {
          case 0:
          wordSet[nCh+40] = 3; //set mode 1
          break;          
          case 1:
          wordSet[nCh+40] = 4; // redi monitor  FLASH
         
          break;
          case 2: 											// Monitoring Grn
						wordSet[nCh] = _mode10P; // уходим 
						if ((wordSet[nCh]) == _mode10P)
						{
							wordSet[nCh] = _mode10P | (_mode10P<<8);
						}
						wordSet[nCh+40] = 2; // Переходим на мониторинг с уставкой 10%					
						
          break;
        }  
				
					led_rgb[nCh] = Nblinck( (_mode10P|_mode10P<<8) , _Yellow, 500, nCh); //  
        
        }
				else if (wordSet[nCh+40] == 4) // выбираем уставку
      {
                 tempPush = timePush (4000, nCh);    

        switch (tempPush)
        {
          case 0:
          wordSet[nCh+40] = 4; // monitor off
          break;          
          case 1:
          wordSet[nCh+40] = 5; // redi monitor
          break;
          case 2:
          wordSet[nCh] = _mode20P; // уходим
					
						if ((wordSet[nCh]) == _mode20P)
						{
							wordSet[nCh] = _mode20P | (_mode20P<<8);
						}
						
						
					wordSet[nCh+40] = 2;
									
          break;
        }  
					led_rgb[nCh] = Nblinck( (_mode20P|_mode20P<<8) , _Yellow, 500, nCh); // 
        
			out =0;
      }
        
      
      else if (wordSet[nCh+40] == 5)
      {
        tempPush = timePush (4000, nCh);    

        switch (tempPush)
        {
          case 0:
          wordSet[nCh+40] = 5; // monitor off
          break;          
          case 1:
          wordSet[nCh+40] = 3; // redi monitor          
          break;
          case 2:
          wordSet[nCh] = _mode30P; // уходим 
					
					if ((wordSet[nCh]) == _mode30P)
						
						{
							wordSet[nCh] = _mode30P | (_mode30P<<8);
						}
						 
					wordSet[nCh+40] = 2; // select 1
					
          break;
        }  

       led_rgb[nCh] = Nblinck(( _mode30P| _mode30P<<8) , _Yellow, 500, nCh); //2
				out = 0;
      }
			else if (wordSet[nCh+40] == 6)
				{
        tempPush = timePush (4000, nCh);    

        switch (tempPush)
        {
          case 0:
          wordSet[nCh+40] = 6; // monitor off
          break;          
          case 1:
					Alarm[nCh+20] = 1;
					Alarm[nCh+30] = 1;
					Alarm[nCh+40] = 1;
					Alarm[nCh+50] = 1;
					
          wordSet[nCh+40] = 2; // redi monitor
				
          break;
          case 2:
					Alarm[nCh+20] = 1;
					Alarm[nCh+30] = 1;
					Alarm[nCh+40] = 1;
					Alarm[nCh+50] = 1;
					
          wordSet[nCh+40] = 0; // select 1					
          break;
        }		
				led_rgb[nCh] =  _Red;       
        out =1;   																								  
			}

      else
      {
			wordSet[nCh+40] = 0; 
      led_rgb[nCh]= _Yellow;
      }
			
			
			
			return out;
			
		} 

    