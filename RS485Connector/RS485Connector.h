#ifndef __RS485Connector_H
#define __RS485Connector_H
#include "main.h"

#define RS485_CON_BUFFER_SIZE (uint16_t)256/*Размер входного и выходного буферов коннектора*/

//Режимы работы коннектора. Коннектор может работать в режиме загрузки прошивки в модуль БОР(Любой другой поддерживающий те же команды)
//и в режиме передачи пакетов универсального интерфейса
typedef enum
{
  RS485_UNDEFINED_MODE,
	RS485_MODBUS_RTU_MODE, 
	RS485_MODBUS_ASCII_MODE,   
	RS485_UNIVERSAL_PROTOCOL_MODE,    
} RS485_WorkMode;

//Структура объекта коннектора
typedef struct
{
	TIM_HandleTypeDef* tim;
  UART_HandleTypeDef* uart;
	GPIO_TypeDef* RTS_GPIO_PORT;
  uint16_t RTS_PIN;	
	uint8_t receiveBuffer[RS485_CON_BUFFER_SIZE];
	uint8_t transmitBuffer[RS485_CON_BUFFER_SIZE];	
	uint16_t rxDataLen;
	uint16_t txDataLen;
	uint8_t receiveByteTmp;	
	RS485_WorkMode workMode;
}RS485Server;

HAL_StatusTypeDef RS485Connector_Init(RS485Server* rs485Server);//Функция инициализации коннектора
void RS485Server_uartRxSlot(RS485Server* server);
void RS485Server_uartTxSlot(RS485Server* server);
void RS485Server_uartErrorSlot(RS485Server* server);
void RS485Server_timerSlot(RS485Server* server);	

uint16_t ModbusRequestCallback(uint8_t* rxDataPtr, uint16_t rxDataLen,uint8_t* txDataPtr);//Заглушка колбэка приема данных коннектора.  Переопределить в своем коде.
uint16_t UPRequestCallback(unsigned char* packetData,unsigned short packetDataLenght, unsigned char* outBufferData);//Заглушка колбэка приема данных коннектора.  Переопределить в своем коде.

#endif

