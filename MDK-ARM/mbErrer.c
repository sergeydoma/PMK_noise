#include "main.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern uint16_t crcTx;
//extern uint8_t arr[];
extern  uint8_t mbAddr;

_Bool mbError(uint8_t *ARR) // возвращаемое значение разрешает семафор osSemaphoreWait(mySemRSHandle, osWaitForever); 
{
   static uint8_t mArr[16];
  
                        switch (ARR[1])
                                {
                                case 1:
                                    if (byte_to_word(ARR[2], ARR[3])>_ADR_MAX_F0)
                                    {
                                        mArr[0]= ARR[0];
                                        mArr[1]= ARR[1] | 0x80;                                        
                                        mArr[2]= 2;
                                        
                                      
                                        crcTx = crc16((uint8_t*)mArr,3);	
                                      
                                        mArr[5]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[4]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5);
                                      ARR[0]=0xFF;  
                                      return 0;
                                    }
                                    else if ((byte_to_word(ARR[2], ARR[3]))+(byte_to_word(ARR[4], ARR[5]))>(_ADR_MAX_F0+1))
                                    {
                                        mArr[0]= ARR[0];
                                        mArr[1]= ARR[1] | 0x80;
                                        mArr[2]= 3;
                                        crcTx = crc16((uint8_t*)mArr,3);	
                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[3]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5);
                                        return 0;
                                    }
                                    else
                                    {
                                     return 1;
                                    }
                                // чтение DO                               
//                                break;
                                case 2:
                                    if (byte_to_word(ARR[2], ARR[3])>_ADR_MAX_F1)
                                    {
                                        mArr[0]= ARR[0];
                                        mArr[1]= ARR[1] | 0x80;
                                        mArr[2]= 2;
                                        crcTx = crc16((uint8_t*)mArr,3);	
                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[3]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5);
                                        return 0;
                                    }
                                    else if ((byte_to_word(ARR[2], ARR[3]))+(byte_to_word(ARR[4], ARR[5]))>(_ADR_MAX_F1+1))
                                    {
                                        mArr[0]= ARR[0];
                                        mArr[1]= ARR[1] | 0x80;
                                        mArr[2]= 3;
                                        crcTx = crc16((uint8_t*)mArr,3);	
                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[3]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5);
                                        return 0;
                                    }
                                    else
                                    {
                                     return 1;
                                    }
//                                break;
                                case 3:
                                    if (byte_to_word(ARR[2], ARR[3])>_ADR_MAX_F3)
                                    {
                                        mArr[0]= ARR[0];
                                        mArr[1]= ARR[1] | 0x80;
                                        mArr[2]= 2;
                                        crcTx = crc16((uint8_t*)mArr,3);	
                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[3]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5);
                                        return 0;
                                    }
                                    else if ((byte_to_word(ARR[2], ARR[3]))+(byte_to_word(ARR[4], ARR[5]))>(_ADR_MAX_F3+1))
                                    {
                                        mArr[0]= ARR[0];
                                        mArr[1]= ARR[1] | 0x80;
                                        mArr[2]= 3;
                                        crcTx = crc16((uint8_t*)mArr,3);	
                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[3]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5);
                                        return 0;
                                    }
                                    else
                                    {
                                     return 1;
                                    }                                    
                                
//                                break;
                                
                                case 4:
                               if (byte_to_word(ARR[2], ARR[3])>_ADR_MAX_F4)
                                    {
                                        mArr[0]= ARR[0];
                                        mArr[1]= ARR[1] | 0x80;
                                        mArr[2]= 2;
                                        crcTx = crc16((uint8_t*)mArr,3);	
                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[3]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5);
                                        return 0;
                                    }
                                    else if ((byte_to_word(ARR[2], ARR[3]))+(byte_to_word(ARR[4], ARR[5]))>(_ADR_MAX_F4+1))
                                    {
                                        mArr[0]= ARR[0];
                                        mArr[1]= ARR[1] | 0x80;
                                        mArr[2]= 3;
                                        crcTx = crc16((uint8_t*)mArr,3);	
                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[3]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5);
                                        return 0;
                                    }
                                    else
                                    {
                                     return 1;
                                    }
                                   
//                                break;
                                case 5:
                                // чтение AI
                                     return 1; 
//                                break;
                                case 6:
                                // запись одного AO
                                    return 1;
//                                break;
                                case 15:
                                // Запись нескольких DO
                                     return 1;
//                                break;
                                case 16:
                                // запись нескольких AO
                                    return 1;
//                                break;
                                
                                default:
                                    // Функция не обслуживается
                                    mArr[0]=ARR[0];
                                    mArr[1]= ARR[1] | 0x80;
                                    mArr[2]= 1;
                                    crcTx = crc16((uint8_t*)mArr,3);	
                                    mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                                    mArr[3]=(crcTx>>8);
                                    //                    osSemaphoreRelease(mySemWrHandle); 
                                    HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5);
                                    ARR[0]=0xFF;
                                    return 0;
//                                    break;
                                    
                                //       osSemaphoreWait(mySemRSHandle, osWaitForever); 
                               
                                //              
                                }
    
////    HAL_UART_Transmit_DMA(&huart2,(uint8_t*) arr,5);
}
