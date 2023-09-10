#include "main.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern volatile osMutexId myMutex_BoolHandle;
extern volatile osMutexId myMutex_WordHandle;
extern uint16_t crcTx;
extern  uint8_t mbAddr;
_Bool UsartTx;
_Bool temp100;


static uint16_t    AdrW;

static uint16_t    lenW;    // длина считываемых бит (в штуках)



_Bool mbFunkton(_Bool *arrBool, uint16_t *arrWord,uint8_t *mbInput, _Bool block) // возвращаемое значение используется для фиксации обмена по modbus
{
  static uint8_t mArr[160];

  static uint16_t NREG;   // колличесвто регистров на передачу
  static _Bool temp;
  
  
                    switch (mbInput[1])
                                {
                                case 0x01: // Эта команда используется для чтения значений дискретных выходов DO.
//                                    if( xSemaphoreTake( myMutex_BoolHandle, 1) == pdTRUE )//portMAX_DELAY // Забираем семафор для работы с общим ресурсом
//                                {
                                // Здесь происходит защищенный доступ к ресурсу.   
                                  mArr[0]= mbAddr;
                                  mArr[1]= mbInput[1];  
                                //********************************************
                                   AdrW = byte_to_word(mbInput[2], mbInput[3]); 
                                   lenW = byte_to_word(mbInput[4], mbInput[5]);
                
                                  for(int i = 0; i<= (lenW);(i=i+8))
                                    {
                                      NREG++;
                                      for(int j = 7 ;j>=0;j--)
                                      {
                                        if   ((i+j)<lenW)   //((i+j)<=lenW)
                                            {temp = arrBool[AdrW+j+i];} //{temp = arrBool[AdrW+j+i*8];} 
                                        else 
                                            {temp = 0;}
                                            
                                        mArr[2+NREG]= (mArr[2+NREG]<<1)| temp;
                                      }
                                    }
                                  mArr[2]=NREG;
																		
                                  //+++++
																				crcTx = crc16((uint8_t*)mArr,(3+NREG));	
																				mArr[2+NREG+2]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[2+NREG+1]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,(5+NREG));	
										
                                    NREG = 0;
																									                                  
                                  
                                    // Отдаем "жетон".
//                                xSemaphoreGive( myMutex_BoolHandle );
//                                }
//                                else if (( xSemaphoreTake( myMutex_BoolHandle, 1) == pdFAIL ))
//                                {
//                                        mArr[0]= mbInput[0];
//                                        mArr[1]= mbInput[1]+0x80;
//                                        mArr[2]= 0x05;                  // 0x1|0x80;
//                                        crcTx = crc16((uint8_t*)mArr,3);	
//                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
//                                        mArr[3]=(crcTx>>8);
//                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени
																					
//                                
//                                }
																break;
                                     
										case 0x02: // Эта команда используется для чтения значений дискретных вводов DI.
//                                    if( xSemaphoreTake( myMutex_BoolHandle, 1) == pdTRUE ) // // portMAX_DELAY Забираем семафор для работы с общим ресурсом
//                                {
                                // Здесь происходит защищенный доступ к ресурсу.   
                                  mArr[0]= mbAddr;
                                  mArr[1]= mbInput[1];  
                                //********************************************
                                   AdrW = byte_to_word(mbInput[2], mbInput[3]); 
                                   lenW = byte_to_word(mbInput[4], mbInput[5]);
                
                                  for(int i = 0; i<= (lenW);(i=i+8))
                                    {
                                      NREG++;
                                      for(int j = 7 ;j>=0;j--)
                                      {
                                        if   ((i+j)<lenW)   //((i+j)<=lenW)
                                            {temp = arrBool[AdrW+j+i];} //{temp = arrBool[AdrW+j+i*8];} 
                                        else 
                                            {temp = 0;}
                                            
                                        mArr[2+NREG]= (mArr[2+NREG]<<1)| temp;
                                      }
                                    }
                                  mArr[2]=NREG;
																		
                                  //+++++
																				crcTx = crc16((uint8_t*)mArr,(3+NREG));	
																				mArr[2+NREG+2]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[2+NREG+1]=(crcTx>>8);
                                        //                    osSemaphoreRelease(mySemWrHandle); 
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,(5+NREG));	
																			
																		NREG = 0;
																									                                  
                                  
                                    // Отдаем "жетон".
//                                xSemaphoreGive( myMutex_BoolHandle );
//                                }
//                                else if(( xSemaphoreTake( myMutex_BoolHandle, 1) == pdFAIL ))
//                                {
//                                        mArr[0]= mbInput[0];
//                                        mArr[1]= mbInput[1]|0x80;
//                                        mArr[2]= 0x05;
//                                        crcTx = crc16((uint8_t*)mArr,3);	
//                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
//                                        mArr[3]=(crcTx>>8);
//                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени
//                                
//                                }
                                   break;
										case 0x03: // Эта команда используется для чтения значений аналогового вывода AO.
//                                    if( xSemaphoreTake( myMutex_WordHandle, 1) == pdTRUE ) //portMAX_DELAY// Забираем семафор для работы с общим ресурсом
//                                {
                                // Здесь происходит защищенный доступ к ресурсу.   
                                  mArr[0]= mbAddr;
                                  mArr[1]= mbInput[1];  
                                //********************************************
                                   AdrW = byte_to_word(mbInput[2], mbInput[3]); 
                                   lenW = byte_to_word(mbInput[4], mbInput[5]);
																	 NREG = 0;																	
																	for(int i = 0; i< (lenW<<1); i=i+2)
                                  {
																		mArr[3+i]= arrWord[AdrW+(i/2)]>>8;
																		mArr[4+i]= arrWord[AdrW+(i/2)];
																		NREG = i+2;	
                                  }
																	mArr[2]=NREG;
																	crcTx = crc16((uint8_t*)mArr,3+NREG);	
                                        mArr[4+NREG]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[3+NREG]=(crcTx>>8);
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5+NREG);   
																			
																	NREG = 0;		
//								xSemaphoreGive( myMutex_WordHandle );																
//                                }
//                                else if (( xSemaphoreTake( myMutex_WordHandle, 1) == pdFAIL ))
//                                {
//                                        mArr[0]= mbInput[0];
//                                        mArr[1]= mbInput[1]|0x80;
//                                        mArr[2]= 0x5;
//                                        crcTx = crc16((uint8_t*)mArr,3);	
//                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
//                                        mArr[3]=(crcTx>>8);
//                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени                                
//                                }
										break;
										case 0x04: // Эта команда используется для чтения значений аналогового вывода AO.
//                                        if( xSemaphoreTake( myMutex_WordHandle, 1) == pdTRUE ) // Забираем семафор для работы с общим ресурсом
//                                {
                                // Здесь происходит защищенный доступ к ресурсу.   
                                  mArr[0]= mbAddr;
                                  mArr[1]= mbInput[1];  
                                //********************************************
                                   AdrW = byte_to_word(mbInput[2], mbInput[3]); 
                                   lenW = byte_to_word(mbInput[4], mbInput[5]);
																	 NREG = 0;																	
																	for(int i = 0; i< (lenW<<1); i=i+2)
                                  {
																		mArr[3+i]= arrWord[AdrW+(i/2)]>>8;
																		mArr[4+i]= arrWord[AdrW+(i/2)];
																		NREG = i+2;	
                                  }
																	mArr[2]=NREG;
																	crcTx = crc16((uint8_t*)mArr,3+NREG);	
                                        mArr[4+NREG]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[3+NREG]=(crcTx>>8);
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5+NREG);   
																			
																	NREG = 0;		
//								xSemaphoreGive( myMutex_WordHandle);																
//                                }
//                                else if (( xSemaphoreTake( myMutex_WordHandle, 1) == pdFAIL ))
//                                {
//                                        mArr[0]= mbInput[0];
//                                        mArr[1]= mbInput[1]|0x80;
//                                        mArr[2]= 0x5;
//                                        crcTx = crc16((uint8_t*)mArr,3);	
//                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
//                                        mArr[3]=(crcTx>>8);
//                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени                                
//                                }
										break;
																                       
                                
										case 0x05: // Эта команда используется для записи дискретного выхода DO.
                      if (block)
                      {
                        mArr[0]= mbInput[0];
                        mArr[1]= mbInput[1]| 0x80;
                        mArr[2]= 0x5 ;
                        crcTx = crc16((uint8_t*)mArr,3);	
                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                        mArr[3]=(crcTx>>8);
                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени                      
                     
											}
                      else
                      {
//                         if( xSemaphoreTake( myMutex_BoolHandle, 1) == pdTRUE ) // Забираем семафор для работы с общим ресурсом
//                                {
                                // Здесь происходит защищенный доступ к ресурсу.   
                                  mArr[0]= mbAddr;
                                  mArr[1]= mbInput[1];  
                                //********************************************
                                 AdrW = byte_to_word(mbInput[2], mbInput[3]);
                                            if ((mbInput[4]==0xFF) &(mbInput[5]==0x00))
                                            {
                                              arrBool[AdrW]=1;
                                              
                                            }
                                             else if ((mbInput[4]==0x00) &(mbInput[5]==0x00))
                                             {
                                              arrBool[AdrW]=0;																		
                                            }
																		 else
																			 {
																				 return 0 ;  // ошибка команды
																				}
																			 
																			mArr[2] = mbInput [2];
																			mArr[3] = mbInput [3];
																			mArr[4] = mbInput [4];
																			mArr[5] = mbInput [5];
																			crcTx = crc16((uint8_t*)mArr,6);
																			mArr[7]=((uint8_t)crcTx & (uint8_t)0xFF);
                                      mArr[6]=(crcTx>>8);
                                      HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,8); 
																		
                                  //+++++
																				
																									                                  
                                  
                                    // Отдаем "жетон".
//                                xSemaphoreGive( myMutex_BoolHandle );
//                                }
//                                else if (( xSemaphoreTake( myMutex_BoolHandle, 1) == pdFAIL ))
//                                {
//                                        mArr[0]= mbInput[0];
//                                        mArr[1]= mbInput[1]|0x80;
//                                        mArr[2]= 0x05;
//                                        crcTx = crc16((uint8_t*)mArr,3);	
//                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
//                                        mArr[3]=(crcTx>>8);
//                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени
//                                
//                                }
                              }
                                
																break;
                                case 0x06: // Эта команда используется для записи аналогового выхода DO.
                    if (block)
                      {
                        mArr[0]= mbInput[0];
                        mArr[1]= mbInput[1]|0x80;
                        mArr[2]= 0x05;
                        crcTx = crc16((uint8_t*)mArr,3);	
                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                        mArr[3]=(crcTx>>8);
                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени                      
                      
											}
                      else
                      {
//                         if( xSemaphoreTake( myMutex_WordHandle, 1) == pdTRUE ) // Забираем семафор для работы с общим ресурсом
//                                {
                                // Здесь происходит защищенный доступ к ресурсу.   
                                  mArr[0]= mbAddr;
                                  mArr[1]= mbInput[1];  
                                //********************************************
                                 AdrW = byte_to_word(mbInput[2], mbInput[3]);
																			arrWord[AdrW] = byte_to_word(mbInput[4], mbInput[5]);
																			mArr[2] = mbInput [2];
																			mArr[3] = mbInput [3];
																			mArr[4] = mbInput [4];
																			mArr[5] = mbInput [5];
																			crcTx = crc16((uint8_t*)mArr,6);
																			mArr[7]=((uint8_t)crcTx & (uint8_t)0xFF);
                                      mArr[6]=(crcTx>>8);
                                      HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,8); 
																		
                                  //+++++
																				
																									                                  
                                  
                                    // Отдаем "жетон".
//                                xSemaphoreGive( myMutex_WordHandle );
//                                }
//                                else if (( xSemaphoreTake( myMutex_WordHandle, 1) == pdFAIL ))
//                                {
//                                        mArr[0]= mbInput[0];
//                                        mArr[1]= mbInput[1]|0x80;
//                                        mArr[2]= 0x05;
//                                        crcTx = crc16((uint8_t*)mArr,3);	
//                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
//                                        mArr[3]=(crcTx>>8);
//                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени
//                                
//                                }
                              }
																break; 
																case 0x0F: // Эта команда используется для записи нескольких значений дискретных выходов DO.
                                  if (block)
                      {
                        mArr[0]= mbInput[0];
                        mArr[1]= mbInput[1]|0x80;
                        mArr[2]= 0x05;
                        crcTx = crc16((uint8_t*)mArr,3);	
                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                        mArr[3]=(crcTx>>8);
                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени                      
                      
											}
                      else
                      {
//                                    if( xSemaphoreTake( myMutex_BoolHandle, 1) == pdTRUE ) //portMAX_DELAY// Забираем семафор для работы с общим ресурсом
//                                {
                                // Здесь происходит защищенный доступ к ресурсу.   
                                  mArr[0]= mbAddr;
                                  mArr[1]= mbInput[1];
                                  mArr[2]= mbInput[2];
                                  mArr[3]= mbInput[3];
                                  mArr[4]= mbInput[4];
                                  mArr[5]= mbInput[5];
                                  
                                //********************************************
                                   AdrW = byte_to_word(mbInput[2], mbInput[3]); 
                                   lenW = byte_to_word(mbInput[4], mbInput[5]);
																	 NREG = mbInput[6];																	
																	for(int i = 0; i< NREG; i++)
                                  {
																		for(int j=0; j<8; j++)//(int j=7;j>=0;j--)
                                    {
																			if ((i*8+j)<= lenW)
																			{
                                        //bit = (byte >> 3) & 1; Чтение бита из байта
                                        temp100 = (mbInput[i+7]>>j)&1;
																				arrBool[(i*8+j+AdrW)] = temp100;
                                        
																			}
                                    } 

                                  }
                                        crcTx = crc16((uint8_t*)mArr,6);	
                                        mArr[7]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[6]=(crcTx>>8);
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,8);   
																	
																	NREG = 0;		
//																	xSemaphoreGive( myMutex_BoolHandle );																
//                                }
//                                else if (( xSemaphoreTake( myMutex_BoolHandle, 1) == pdFAIL ))
//                                {
//                                        mArr[0]= mbInput[0];
//                                        mArr[1]= mbInput[1]|0x80;
//                                        mArr[2]= 0x5;
//                                        crcTx = crc16((uint8_t*)mArr,3);	
//                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
//                                        mArr[3]=(crcTx>>8);
//                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени                                
//                                }
                              }
																break;
                                case 0x10: // Эта команда используется для записи нескольких значений аналоговых выходов AO.
                    if (block)
                      {
                        mArr[0]= mbInput[0];
                        mArr[1]= mbInput[1]|0x80;
                        mArr[2]= 0x05;
                        crcTx = crc16((uint8_t*)mArr,3);	
                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
                        mArr[3]=(crcTx>>8);
                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени                      
                       
											}
                      else
                      {
//                                    if( xSemaphoreTake( myMutex_WordHandle, portMAX_DELAY) == pdTRUE ) // Забираем семафор для работы с общим ресурсом
//                                {
                                // Здесь происходит защищенный доступ к ресурсу.   
                                  mArr[0]= mbAddr;
                                  mArr[1]= mbInput[1];
                                  mArr[2]= mbInput[2];
                                  mArr[3]= mbInput[3];
                                  mArr[4]= mbInput[4];
                                  mArr[5]= mbInput[5];
                                //********************************************
                                   AdrW = byte_to_word(mbInput[2], mbInput[3]); 
                                   lenW = byte_to_word(mbInput[4], mbInput[5]);
																	 NREG = mbInput[6];																	
																	for(int i = 0; i< NREG; i=i+2)
                                  {
																		arrWord[(i/2)+AdrW] = byte_to_word( mbInput[7+i],mbInput[7+i+1]) ;
                                      //******************************************************************
//																			
                                  }
                                  
                                        crcTx = crc16((uint8_t*)mArr,6);	
                                        mArr[7]=((uint8_t)crcTx & (uint8_t)0xFF);
                                        mArr[6]=(crcTx>>8);
                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,8);
																				
																				NREG = 0;		
//																	xSemaphoreGive( myMutex_WordHandle );																
//                                }
//                                else if (( xSemaphoreTake( myMutex_WordHandle, 1) == pdFAIL ))
//                                {
//                                        mArr[0]= mbInput[0];
//                                        mArr[1]= mbInput[1]|0x80;
//                                        mArr[2]= 0x05;
//                                        crcTx = crc16((uint8_t*)mArr,3);	
//                                        mArr[4]=((uint8_t)crcTx & (uint8_t)0xFF);
//                                        mArr[3]=(crcTx>>8);
//                                        HAL_UART_Transmit_DMA(&huart2,(uint8_t*) mArr,5); // Задача требует времени                                
//                                }
                              }
																break;
															}

    
															return 1;

}

