#include "RS485Connector.h"
#include "ModbusCommon.h"
#include "string.h"
#include "stdlib.h"
#include "UPCommon.h"
#include "ModbusCommon.h"
#include "Memory.h"

extern Memory* memory;

__WEAK uint16_t ModbusRequestCallback(uint8_t* rxDataPtr, uint16_t rxDataLen,uint8_t* txDataPtr)
{
	
	return 0;
}
__WEAK uint16_t UPRequestCallback(uint8_t* rxDataPtr, uint16_t rxDataLen,uint8_t* txDataPtr)
{
	
	return 0;
}

void RS485Server_uartTxSlot(RS485Server* server)
{
	HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_RESET);
}

void RS485Server_uartErrorSlot(RS485Server* server)
{
	HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_RESET);
	
	server->rxDataLen=0;
	HAL_UART_Receive_IT(server->uart,&server->receiveByteTmp,1);
}

void RS485Server_uartRxSlot(RS485Server* server)
{
	server->receiveBuffer[server->rxDataLen++]=server->receiveByteTmp;
	HAL_UART_Receive_IT(server->uart,&server->receiveByteTmp,1);
	
	if(server->rxDataLen==RS485_CON_BUFFER_SIZE)
	{
		server->rxDataLen=0;
		return;
	}
	
	if(server->workMode == RS485_MODBUS_RTU_MODE)
	{
		__HAL_TIM_SET_COUNTER(server->tim, 0x0001);
	  __HAL_TIM_CLEAR_IT(server->tim,TIM_IT_UPDATE);
	  HAL_TIM_Base_Start_IT(server->tim);
	}
	else if(server->workMode == RS485_MODBUS_ASCII_MODE)
	{
		if(server->receiveByteTmp==':')
		{
			server->receiveBuffer[0]=':';
			server->rxDataLen=1;
			return;
		}
		
		if(server->rxDataLen>7 && memcmp(&server->receiveBuffer[server->rxDataLen-2],"\r\n",sizeof("\r\n")-1)==0)
		{
			//Обрабатываем пакет, если есть что отдать - передаем
			for(int j = 0; j <(server->rxDataLen-3)/2 ; j++)
			{
				server->receiveBuffer[j] = twoCharToUint8((char*)&server->receiveBuffer[j*2+1]);
			}
			
			if(Lrc8(server->receiveBuffer,(server->rxDataLen-3)/2 - 1)==server->receiveBuffer[(server->rxDataLen-3)/2 - 1])
			{			
			  if(server->receiveBuffer[0]!=memory->uartProperty.deviceAddress)return;
				
				uint16_t dataLen = (server->rxDataLen-3)/2 - 1;//Без байта контрольной суммы
				if((dataLen = ModbusRequestCallback(server->receiveBuffer,dataLen,server->transmitBuffer))>0)
			  {
					server->transmitBuffer[dataLen]=Lrc8(server->transmitBuffer,dataLen);
					
					for(int j=dataLen; j>-1;j--)
			    {
				    Uint8ToTwoChars((char*)&server->transmitBuffer[j*2+1],server->transmitBuffer[j]);
			    }
					
					server->transmitBuffer[0]=':';
			    server->transmitBuffer[2*(dataLen+1)+1]='\r';
			    server->transmitBuffer[2*(dataLen+1)+2]='\n';
					
					dataLen = 2*(dataLen+1)+3;
					
					HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_SET);
			    HAL_StatusTypeDef result = HAL_UART_Transmit_IT(server->uart,server->transmitBuffer,dataLen);	
			    if(result!=HAL_OK)HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_RESET);
			  }
			}
			server->rxDataLen=0;
		}	
	}	
	else if(server->workMode == RS485_UNIVERSAL_PROTOCOL_MODE)
	{
		if(server->rxDataLen==1 && server->receiveBuffer[0]!=UP_SOF)server->rxDataLen=0;
		else if((server->rxDataLen>5)
			     &&(server->receiveBuffer[3] == (server->rxDataLen - 4)) 
		       &&(server->receiveBuffer[server->rxDataLen-1] == UP_CRC_UART(&server->receiveBuffer[1],server->rxDataLen-2)))//Пришел пакет, выключаем таймер, обрабатываем
		{
			__HAL_TIM_SET_COUNTER(server->tim, 0x0001);
	    __HAL_TIM_CLEAR_IT(server->tim,TIM_IT_UPDATE);
			
			UPHeaderUART* rxHeader = (void*)server->receiveBuffer;
			if(memory->uartProperty.deviceAddress!=rxHeader->destAddr)return;
				
			UPHeaderUART* txHeader = (void*)server->transmitBuffer;
				
			if((txHeader->packLen = UPRequestCallback(rxHeader->data, rxHeader->packLen - 1,txHeader->data))>0)
			{
				txHeader->packLen++;//+ контрольная сумма
				txHeader->SOF=UP_SOF;
			  txHeader->destAddr = rxHeader->srcAddr;
				txHeader->srcAddr = rxHeader->destAddr;
				txHeader->data[txHeader->packLen-1] = UP_CRC_UART(&server->transmitBuffer[1],txHeader->packLen+2);
					
				HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_SET);
			  HAL_StatusTypeDef result = HAL_UART_Transmit_IT(server->uart,server->transmitBuffer,txHeader->packLen+4);	
			  if(result!=HAL_OK)HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_RESET);
			}
			server->rxDataLen=0;
		}
		else //Пакет принят не полностью
		{
			__HAL_TIM_SET_COUNTER(server->tim, 0x0001);
	    __HAL_TIM_CLEAR_IT(server->tim,TIM_IT_UPDATE);
	    HAL_TIM_Base_Start_IT(server->tim);
		}
	}
	else if(server->workMode == RS485_UNDEFINED_MODE)
	{
		__HAL_TIM_SET_COUNTER(server->tim, 0x0001);
	  __HAL_TIM_CLEAR_IT(server->tim,TIM_IT_UPDATE);
	  HAL_TIM_Base_Start_IT(server->tim);
	} 
}

void RS485Server_timerSlot(RS485Server* server)
{	
	HAL_TIM_Base_Stop_IT(server->tim);
	__HAL_TIM_CLEAR_IT(server->tim,TIM_IT_UPDATE);	
		
	if(server->workMode == RS485_MODBUS_RTU_MODE)//Принят пакет Modbus RTU
	{
		if(server->rxDataLen<5)//Недостаточная длинна пакета
		{
			server->rxDataLen=0;
			return;
		}
		
		if(Crc16(server->receiveBuffer, server->rxDataLen-2)!=(server->receiveBuffer[server->rxDataLen-1]<<8|server->receiveBuffer[server->rxDataLen-2]))//Ошибка контрольной суммы пакета
		{
			server->rxDataLen=0;
			return;
		}
		
		if(server->receiveBuffer[0]!=memory->uartProperty.deviceAddress)return;
			
		uint16_t dataLen = server->rxDataLen-2;
		if((dataLen = ModbusRequestCallback(server->receiveBuffer,dataLen,server->transmitBuffer))>0)//Длинна без контрольной суммы
		{
			uint16_t crc = Crc16(server->transmitBuffer, dataLen);
			server->transmitBuffer[dataLen]=crc&0xFF;
			server->transmitBuffer[dataLen+1]=crc>>8;
			dataLen+=2;
	    
			HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_SET);
			HAL_StatusTypeDef result = HAL_UART_Transmit_IT(server->uart,server->transmitBuffer,dataLen);	
			if(result!=HAL_OK)HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_RESET);
		}
		server->rxDataLen=0;
	}
	else if(server->workMode == RS485_UNIVERSAL_PROTOCOL_MODE)server->rxDataLen=0;//Нужен Сброс
	else if(server->workMode == RS485_UNDEFINED_MODE)//Получен первый пакет, определяем тип протокола по которому работает сеть
	{
		if(server->rxDataLen<5)//Недостаточная длинна пакета
		{
			server->rxDataLen=0;
			return;
		}
		
		//Сначала проверим является ли пакет пакетом Modbus RTU
		if(Crc16(server->receiveBuffer, server->rxDataLen-2)==(server->receiveBuffer[server->rxDataLen-1]<<8|server->receiveBuffer[server->rxDataLen-2]))//Пришел пакет Modbus RTU
		{
			server->workMode=RS485_MODBUS_RTU_MODE;
			
			//Настройки таймера устанавливаем в соответствии с протоколом Modbus RTU (стоят по умолчанию, можно не менять
			
			if(server->receiveBuffer[0]!=memory->uartProperty.deviceAddress)return;
			
			uint16_t dataLen = server->rxDataLen-2;
			if((dataLen = ModbusRequestCallback(server->receiveBuffer,dataLen,server->transmitBuffer))>0)//Длинна без контрольной суммы
			{
				uint16_t crc = Crc16(server->transmitBuffer, dataLen);
				server->transmitBuffer[dataLen]=crc&0xFF;
				server->transmitBuffer[dataLen+1]=crc>>8;
				dataLen+=2;
	      
				HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_SET);
			  HAL_StatusTypeDef result = HAL_UART_Transmit_IT(server->uart,server->transmitBuffer,dataLen);	
			  if(result!=HAL_OK)HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_RESET);
			}
			server->rxDataLen=0;
			return;
		}
		
		if(server->receiveBuffer[0]==':' && server->rxDataLen>7 && memcmp(&server->receiveBuffer[server->rxDataLen-2],"\r\n",sizeof("\r\n")-1)==0)
		{
			for(int j = 0; j <(server->rxDataLen-3)/2 ; j++)
			{
				server->transmitBuffer[j] = twoCharToUint8((char*)&server->receiveBuffer[j*2+1]);//Используем буфер для передачи чтобы не испортить пакет
			}
			
			if(Lrc8(server->transmitBuffer,(server->rxDataLen-3)/2 - 1)==server->transmitBuffer[(server->rxDataLen-3)/2 - 1])//Пришел пакет Modbus ASCII
			{
			  server->workMode=RS485_MODBUS_ASCII_MODE;
				
				//Таймер может не использоваться
			
			  if(server->transmitBuffer[0]!=memory->uartProperty.deviceAddress)return;
				
				uint16_t dataLen = (server->rxDataLen-3)/2 - 1;
				memcpy(server->receiveBuffer,server->transmitBuffer,dataLen);
				if((dataLen = ModbusRequestCallback(server->receiveBuffer,dataLen,server->transmitBuffer))>0)
			  {
					server->transmitBuffer[dataLen]=Lrc8(server->transmitBuffer,dataLen);
					
					for(int j=dataLen; j>-1;j--)
			    {
				    Uint8ToTwoChars((char*)&server->transmitBuffer[j*2+1],server->transmitBuffer[j]);
			    }
					
					server->transmitBuffer[0]=':';
			    server->transmitBuffer[2*(dataLen+1)+1]='\r';
			    server->transmitBuffer[2*(dataLen+1)+2]='\n';
					
					dataLen = 2*(dataLen+1)+3;
	        
					HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_SET);
			    HAL_StatusTypeDef result = HAL_UART_Transmit_IT(server->uart,server->transmitBuffer,dataLen);	
			    if(result!=HAL_OK)HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_RESET);
			  }
				server->rxDataLen=0;
				return;
			}
		}
		
		if(server->receiveBuffer[0]==UP_SOF)
		{
			if(server->receiveBuffer[server->rxDataLen-1]==UP_CRC_UART(&server->receiveBuffer[1],server->rxDataLen-2))//Принят пакет универсального протокола
			{
				server->workMode=RS485_UNIVERSAL_PROTOCOL_MODE;
				//Настройки таймера устанавливаем в соответствии с протоколом универсального интерфейса (200 мс означает ошибка приема пакета)
				server->tim->Instance->ARR = 20000;//200мс
								
			  UPHeaderUART* rxHeader = (void*)server->receiveBuffer;
				if(memory->uartProperty.deviceAddress!=rxHeader->destAddr)return;
				
				UPHeaderUART* txHeader = (void*)server->transmitBuffer;
				
				if((txHeader->packLen = UPRequestCallback(rxHeader->data, rxHeader->packLen - 1,txHeader->data))>0)
				{
					txHeader->packLen++;
				  txHeader->SOF=UP_SOF;
					txHeader->destAddr = rxHeader->srcAddr;
					txHeader->srcAddr = rxHeader->destAddr;
					txHeader->data[txHeader->packLen-1] = UP_CRC_UART(&server->transmitBuffer[1],txHeader->packLen+2);
					
					HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_SET);
			    HAL_StatusTypeDef result = HAL_UART_Transmit_IT(server->uart,server->transmitBuffer,txHeader->packLen+4);	
			    if(result!=HAL_OK)HAL_GPIO_WritePin(server->RTS_GPIO_PORT,server->RTS_PIN,GPIO_PIN_RESET);
				}				
				server->rxDataLen=0;
				return;
			}
		}
		server->rxDataLen=0;
	}
}

HAL_StatusTypeDef RS485Connector_Init(RS485Server* rs485Server)
{
	rs485Server->rxDataLen = 0;
	rs485Server->txDataLen = 0;
	
	rs485Server->workMode = RS485_UNDEFINED_MODE;
	
	HAL_GPIO_WritePin(rs485Server->RTS_GPIO_PORT,rs485Server->RTS_PIN,GPIO_PIN_RESET);
	HAL_UART_Receive_IT(rs485Server->uart,&rs485Server->receiveByteTmp,1);
	
	return HAL_OK;
}


