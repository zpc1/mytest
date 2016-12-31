/**
  ******************************************************************************
  * @file    USART.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   USART buffer.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "USART.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct SCIBuffer SCI1,SCI2;
struct SCIBuffer2 SCI3,SCI4;
extern uint8_t  TX_ADDRESS[TX_ADR_WIDTH];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/	  
uint16_t SCI_Transmit(uint8_t controller,uint16_t len,const uint8_t *txData)
{
    uint16_t i=0;
    uint16_t CurWriPos=0;

    if(controller==1)
    {
         CurWriPos=(SCI1.TxLen+SCI1.TxPos)%SCI_BufferSize;//根据缓存内的数据长度和当前位置确定当前可写入的位置
         if(SCI_BufferSize-SCI1.TxLen>0)
         {
            if(SCI_BufferSize-SCI1.TxLen>=len)
            {
                for(i=0;i<len;i++)
                	SCI1.TxBuffer[(CurWriPos+i)%SCI_BufferSize]=*(txData+i);
                SCI1.TxLen+=len;
            }else
            {
                for(i=0;i<SCI_BufferSize-SCI1.TxLen;i++)
                	SCI1.TxBuffer[(CurWriPos+i)%SCI_BufferSize]=*(txData+i);
                SCI1.TxLen+=i;
            }
         }
         else
         SCI1.TxLen=0;//overflow

    }                                                                    
    else if(controller==2)
    {
         CurWriPos=(SCI2.TxLen+SCI2.TxPos)%SCI_BufferSize;//根据缓存内的数据长度和当前位置确定当前可写入的位置
         if(SCI_BufferSize-SCI2.TxLen>0)
         {
            if(SCI_BufferSize-SCI2.TxLen>=len)
            {
                for(i=0;i<len;i++)
                SCI2.TxBuffer[(CurWriPos+i)%SCI_BufferSize]=*(txData+i);
                SCI2.TxLen+=len;
            }else
            {
                for(i=0;i<SCI_BufferSize-SCI2.TxLen;i++)
                SCI2.TxBuffer[(CurWriPos+i)%SCI_BufferSize]=*(txData+i);
                SCI2.TxLen+=i;
            }
         }
         else
         SCI2.TxLen=0;//overflow
    }
	else if(controller==3)
    {
         CurWriPos=(SCI3.TxLen+SCI3.TxPos)%SCI_BufferSize2;//根据缓存内的数据长度和当前位置确定当前可写入的位置
         if(SCI_BufferSize2-SCI3.TxLen>0)
         {
            if(SCI_BufferSize2-SCI3.TxLen>=len)
            {
                for(i=0;i<len;i++)
                	SCI3.TxBuffer[(CurWriPos+i)%SCI_BufferSize2]=*(txData+i);
                SCI3.TxLen+=len;
            }
			else
            {
                for(i=0;i<SCI_BufferSize2-SCI3.TxLen;i++)
                	SCI3.TxBuffer[(CurWriPos+i)%SCI_BufferSize2]=*(txData+i);
                SCI3.TxLen+=i;
            }
         }
         else
         SCI3.TxLen=0;//overflow
    }
	else if(controller==4)
    {
         CurWriPos=(SCI4.TxLen+SCI4.TxPos)%SCI_BufferSize2;//根据缓存内的数据长度和当前位置确定当前可写入的位置
         if(SCI_BufferSize2-SCI4.TxLen>0)
         {
            if(SCI_BufferSize2-SCI4.TxLen>=len)
            {
                for(i=0;i<len;i++)
                	SCI4.TxBuffer[(CurWriPos+i)%SCI_BufferSize2]=*(txData+i);
                SCI4.TxLen+=len;
            }
			else
            {
                for(i=0;i<SCI_BufferSize2-SCI4.TxLen;i++)
                	SCI4.TxBuffer[(CurWriPos+i)%SCI_BufferSize2]=*(txData+i);
                SCI4.TxLen+=i;
            }
         }
         else
         SCI4.TxLen=0;//overflow
    }
    return i;
}

/* SCI Receiving */
uint16_t SCI_Receive(uint8_t controller,uint16_t len,uint8_t *rxData)
{    
    uint16_t i = 0;

    if(controller==1)
    {
         if(SCI1.RxLen>0)
         {
             if(SCI1.RxLen>=len)
             {
                for(i=0;i<len;i++)
                {
                    *(rxData+i)=SCI1.RxBuffer[SCI1.RxPos];
                    SCI1.RxPos=(SCI1.RxPos+1)%SCI_BufferSize;//确定新的读取位置
                }
                SCI1.RxLen-=len;
                return len;
             }
             else
             {  
                for(i=0;i<SCI1.RxLen;i++)
                {
                    *(rxData+i)=SCI1.RxBuffer[SCI1.RxPos];
                    SCI1.RxPos=(SCI1.RxPos+1)%SCI_BufferSize;//确定新的读取位置
                }
                SCI1.RxLen = 0;
                return i;
             }
         }
    }
    else if(controller==2)
    {
         if(SCI2.RxLen>0)
         {
             if(SCI2.RxLen>=len)
             {
                for(i=0;i<len;i++)
                {
                    *(rxData+i)=SCI2.RxBuffer[SCI2.RxPos];
                    SCI2.RxPos=(SCI2.RxPos+1)%SCI_BufferSize;//确定新的读取位置
                }
                SCI2.RxLen-=len;
                return len;
             }
             else
             {
               for(i=0;i<SCI2.RxLen;i++)
               {
                    *(rxData+i)=SCI2.RxBuffer[SCI2.RxPos];
                    SCI2.RxPos=(SCI2.RxPos+1)%SCI_BufferSize;//确定新的读取位置
               }
               SCI2.RxLen=0;
               return i;
             }
         }
    }
	else if(controller==3)
    {
         if(SCI3.RxLen>0)
         {
             if(SCI3.RxLen>=len)
             {
                for(i=0;i<len;i++)
                {
                    *(rxData+i)=SCI3.RxBuffer[SCI3.RxPos];
                    SCI3.RxPos=(SCI3.RxPos+1)%SCI_BufferSize2;//确定新的读取位置
                }
                SCI3.RxLen-=len;
                return len;
             }
             else
             {
               for(i=0;i<SCI3.RxLen;i++)
               {
                    *(rxData+i)=SCI3.RxBuffer[SCI3.RxPos];
                    SCI3.RxPos=(SCI3.RxPos+1)%SCI_BufferSize2;//确定新的读取位置
               }
               SCI3.RxLen=0;
               return i;
             }
         }
    }
	else if(controller==4)
    {
         if(SCI4.RxLen>0)
         {
             if(SCI4.RxLen>=len)
             {
                for(i=0;i<len;i++)
                {
                    *(rxData+i)=SCI4.RxBuffer[SCI4.RxPos];
                    SCI4.RxPos=(SCI4.RxPos+1)%SCI_BufferSize2;//确定新的读取位置
                }
                SCI4.RxLen-=len;
                return len;
             }
             else
             {
               for(i=0;i<SCI4.RxLen;i++)
               {
                    *(rxData+i)=SCI4.RxBuffer[SCI4.RxPos];
                    SCI4.RxPos=(SCI4.RxPos+1)%SCI_BufferSize2;//确定新的读取位置
               }
               SCI4.RxLen=0;
               return i;
             }
         }
    }
	return 0;
}

uint16_t SCI3_Receive(uint8_t turn,uint16_t len,uint8_t *rxData)
{    
	static uint16_t len_temp = 0;
	uint16_t i = 0;

	if(turn == 0)	//正常读取缓冲区
	{
	    if(SCI3.RxLen>0)
	    {
	    	if(SCI3.RxLen>=len)
	        {				
	        	for(i=0;i<len;i++)
	            {
	            	*(rxData+i)=SCI3.RxBuffer[SCI3.RxPos];
	                SCI3.RxPos=(SCI3.RxPos+1)%SCI_BufferSize2;//确定新的读取位置
	            }
	            SCI3.RxLen-=len;  
				len_temp = len;
	            return len;
	        }
	        else
	        {
	        	for(i=0;i<SCI3.RxLen;i++)
	            {
	            	*(rxData+i)=SCI3.RxBuffer[SCI3.RxPos];
	                SCI3.RxPos=(SCI3.RxPos+1)%SCI_BufferSize2;//确定新的读取位置
	            }
	            SCI3.RxLen=0;
				len_temp = i;
	            return i;
	        }
	    }
	}
	else		 //缓冲区退格
	{
		SCI3.RxPos = ((SCI3.RxPos + SCI_BufferSize2) - len_temp) % SCI_BufferSize2; //确定新的读取位置
	    SCI3.RxLen += len_temp;
		len_temp = 0; 	
	}

	return 0;
}

/* Set the state of SCI1 or SCI2 or SCI3 */
void SetState(uint8_t controller,uint8_t state)
{
    if(controller==1)
      if(state==0)
      SCI1.State=0;//SCI1 OFF
      else
      SCI1.State=1;//SCI1 ON
    else if(controller==2)
      if(state==0)
      SCI2.State=0;//SCI2 OFF
      else
      SCI2.State=1;//SCI2 ON
	else if(controller==3)
      if(state==0)
      SCI3.State=0;//SCI3 OFF
      else
      SCI3.State=1;//SCI3 ON
	else if(controller==4)
      if(state==0)
      SCI4.State=0;//SCI4 OFF
      else
      SCI4.State=1;//SCI4 ON
}

/* Get the number of data in SCI1-buffer or SCI2-buffer */
uint16_t SCI_GetLen(uint8_t controller, uint8_t RorT)
{
    uint16_t a;

    if(controller==1)
    {
        if(RorT==0)
           a=SCI1.RxLen;//Get the receive buffer cell number of SCI1
        else 
           a=SCI1.TxLen;//Get the transmission buffer cell number of SCI1
    }
    else if(controller==2)
    {
        if(RorT==0)
           a=SCI2.RxLen;//Get the receive buffer cell number of SCI2
        else 
           a=SCI2.TxLen;//Get the transmission buffer cell number of SCI2
    }
	else if(controller==3)
    {
        if(RorT==0)
           a=SCI3.RxLen;//Get the receive buffer cell number of SCI2
        else 
           a=SCI3.TxLen;//Get the transmission buffer cell number of SCI2
    }
	else if(controller==4)
    {
        if(RorT==0)
           a=SCI4.RxLen;//Get the receive buffer cell number of SCI4
        else 
           a=SCI4.TxLen;//Get the transmission buffer cell number of SCI4
    }
    return a;
}

uint32_t send=0,recv=0;
/*  From register to buffer, From buffer to register */
void SCI_MainFunction(void)
{
	uint16_t CurWriPos=0;

	if(SCI1.State==1)						//USART1 Ttransmit Bluetooth
   	{
       if(USART_GetFlagStatus(USART1, USART_FLAG_TXE))//发送数据为空                                           
       {	   		
          if(SCI1.TxLen>0)
          {	
						USART_SendData(USART1, SCI1.TxBuffer[SCI1.TxPos]);
            SCI1.TxPos=(SCI1.TxPos+1)%SCI_BufferSize;//确定最新所指位置，防止所指位置超出缓存的大小
            SCI1.TxLen--;
          }
       }
       if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE))//SCI1 Receive
       {  
					USART_ClearFlag(USART1, USART_FLAG_RXNE);
          CurWriPos=(SCI1.RxLen+SCI1.RxPos)%SCI_BufferSize;
          if(SCI1.RxLen<SCI_BufferSize)
          {
            SCI1.RxBuffer[CurWriPos]=USART_ReceiveData(USART1);
            SCI1.RxLen++;
          }
          else
          {         
            SCI1.RxLen = 0;//overflow  
          }
       }
   	} 
   if(SCI2.State==1)					//USART2 Ttransmit K-line                   
   {
       if(USART_GetFlagStatus(USART2, USART_FLAG_TXE))                                           
       {
          if(SCI2.TxLen>0)
          {
            USART_SendData(USART2, SCI2.TxBuffer[SCI2.TxPos]);
            SCI2.TxPos=(SCI2.TxPos+1)%SCI_BufferSize;//确定最新所指位置，防止所指位置超出缓存的大小
            SCI2.TxLen--;
          }
       }
       if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE))//SCI1 Receive
       { 
	   		USART_ClearFlag(USART2, USART_FLAG_RXNE);	   		
          CurWriPos=(SCI2.RxLen+SCI2.RxPos)%SCI_BufferSize;
          if(SCI2.RxLen<SCI_BufferSize)
          {
            SCI2.RxBuffer[CurWriPos]=USART_ReceiveData(USART2);
            SCI2.RxLen++;
          }
          else
          {
            SCI2.RxLen=0;//overflow
          }
       } 
   } 
   if(SCI3.State==1)					//USART3 Ttransmit SIM900A                   
   {
       if(USART_GetFlagStatus(USART3, USART_FLAG_TXE))                                           
       {
          if(SCI3.TxLen>0)
          {
            USART_SendData(USART3, SCI3.TxBuffer[SCI3.TxPos]);
            SCI3.TxPos=(SCI3.TxPos+1)%SCI_BufferSize2;//确定最新所指位置，防止所指位置超出缓存的大小
            SCI3.TxLen--;
          }
       }
       if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE))//SCI3 Receive
       { 
	   		USART_ClearFlag(USART3, USART_FLAG_RXNE);	   		
          	CurWriPos=(SCI3.RxLen+SCI3.RxPos)%SCI_BufferSize2;
          	if(SCI3.RxLen < SCI_BufferSize2)
          	{
            	SCI3.RxBuffer[CurWriPos]=USART_ReceiveData(USART3);
            	SCI3.RxLen++;
          	}
          	else
            	SCI3.RxLen=0;//overflow
       } 
   	} 
   	if(SCI4.State==1)					//USART4 Ttransmit UM220III                   
   {
       if(USART_GetFlagStatus(UART4, USART_FLAG_TXE))                                           
       {
          if(SCI4.TxLen>0)
          {
            USART_SendData(UART4, SCI4.TxBuffer[SCI4.TxPos]);
            SCI4.TxPos=(SCI4.TxPos+1)%SCI_BufferSize2;//确定最新所指位置，防止所指位置超出缓存的大小
            SCI4.TxLen--;
          }
       }
       if(USART_GetFlagStatus(UART4, USART_FLAG_RXNE))//SCI4 Receive
       { 
						USART_ClearFlag(UART4, USART_FLAG_RXNE);	   		
          	CurWriPos=(SCI4.RxLen+SCI4.RxPos)%SCI_BufferSize2;
          	if(SCI4.RxLen < SCI_BufferSize2)
          	{
            	SCI4.RxBuffer[CurWriPos]=USART_ReceiveData(UART4);
            	SCI4.RxLen++;
          	}
          	else
            	SCI4.RxLen=0;//overflow
       } 
   	} 
}
