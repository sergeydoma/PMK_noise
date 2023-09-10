//#include "tasks.h"
#include "main.h"
#include "stdlib.h"
#include "stm32f407xx.h"
//#include "stm32f407xx_rcc.h"
// #include "stm32f4xx_crc.h"
extern uint8_t state;

//extern _Time_control st;
//extern _stw TestSTW;  //#Test
uint32_t CRCVal; //#Test
//extern uint32_t CRCPoint[250];//#Test
//extern uint32_t CCRC;
extern CRC_HandleTypeDef hcrc;
uint32_t testCRC;
_Bool CRCerror;
// uint32_t temp_CRC; 
uint32_t  constCRC;
//Функция для аппаратного подсчёта CRC32
//Принимает на вход указатель на массив uint32_t чисел и размер этого массива
//Возвращает CRC32 для этого массива

//uint32_t crc_calc(uint32_t * buffer[], uint32_t buff_len) 
//{
//  uint32_t i;
//  CRC->CR |= CRC_CR_RESET; //Делаем сброс...
//  for(i = 0; i < buff_len; i++) 
//	{
//    CRC->DR = buffer[i]; //Загоняем данные из буфера в регистр данных
//  }
//  return (CRC->DR); //Читаем контрольную сумму и возвращаем её
//}


uint32_t pro_CRC(void)
{
  testCRC=0;
	uint64_t stepCRC;
	uint32_t temp;   
//  uint16_t fulstep=0;
    
	CRC->CR = 1; // сброс
    
    
	for (uint32_t i = 0; i< 0x100000; i++) //0x1DFAC
	{
		stepCRC = 0x8000000 + i*4;	
		CRCVal = (*(__IO uint32_t*)stepCRC);
            
//        temp_CRC = CRCVal;
//        
//////        if (temp_CRC !=_CRC)
//////        {
//////		if (temp_CRC ==0xffffffff)
//////		{ fulstep+=1;
//////            if (fulstep>=3)
//////                {break;}		
//////		}
//////        else
//////        {
//////        fulstep=0;
//////        }
		CRC->DR = CRCVal; // отправить 4 байта
//     }
	}
	temp = CRC->DR; 
//    constCRC=_CRC;
//    if (temp == constCRC)
//    {CRCerror = 0;}
	return temp; // Получить контрольную сумму
}
