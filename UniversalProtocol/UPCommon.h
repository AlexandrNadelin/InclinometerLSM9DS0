#ifndef LIB_CMD_SET_H
#define LIB_CMD_SET_H
#ifdef __cplusplus
 extern "C" {
#endif
//#include "ClimatControlDevice.h"
 //#include "main.h"

#define OWN_LETTER_ADDRESS    (unsigned char)100
#define OWN_CHANNEL_ADDRESS   (unsigned char)1
//---Структура заголовка универсального протокола при работе через UART---//
typedef struct{
	unsigned char SOF;
	unsigned char srcAddr;
	unsigned char destAddr;
	unsigned char packLen;//Длинна данных (без SOF, srcAddr, destAddr, packLen) плюс байт контрольной суммы
	unsigned char data[];
 	//uint8_t CRC_XOR
}UPHeaderUART;

#define UP_SOF                     (unsigned char)0xBA/*Маркер Start of frame*/

//---Команды ---//
//---Служебные---//
#define UP_CMD_CHECK_LINK          (unsigned char)0x01/*Аналог ping*/
#define UP_CMD_GET_DEVICE_INFO     (unsigned char)0x06/*Запрос информации об устройстве*/
#define UP_CMD_DEBUG               (unsigned char)0x3F/**/
#define UP_CMD_EXT_CMD_SET         (unsigned char)0x0F/**/

//---Пользовательская информация---//
#define UP_CMD_GET_USER_INFO       (unsigned char)0x0D/*Запрос пользовательской информации*/
#define UP_CMD_CHANGE_USER_INFO    (unsigned char)0x0E/*Смена пользовательской информации*/
#define UP_CMD_CHANGE_ENABLE       (unsigned char)0x0F/*Разрешение изменения системных настроек*/

// конфигурирование Ethernet
#define UP_CMD_GET_MAC_ADDRESS		 (unsigned char)0x02
#define UP_CMD_GET_NETPARAMS		   (unsigned char)0x04
#define UP_CMD_CHANGE_NETPARAMS	   (unsigned char)0x05
#define UP_CMD_CHANGE_MAC_ADDRESS	 (unsigned char)0x03
#define UP_CMD_GET_TCP_PORT_STATE	 (unsigned char)0x11

	// свободно: 0x03
#define UP_CMD_UNUSED_03			     (unsigned char)0x03	/*свободно*/

//---для загрузчика---//
//#define UP_CMD_READ_FIRMWARE_BLOCK	(unsigned char)0x07/*Не используется*/
#define UP_CMD_GET_BOOT_PARAMS      (unsigned char)0x08
#define UP_ANS_GET_BOOT_PARAMS	    (unsigned char)0x84
#define UP_CMD_WRITE_FIRMWARE_BLOCK	(unsigned char)0x09
//#define UP_CMD_COPY_FIRMWARE			  (unsigned char)0x0A/*Не используется*/
#define UP_CMD_JUMP_TO_APPLICATION	(unsigned char)0x0B
#define UP_CMD_JUMP_TO_BOOTLOADER	  (unsigned char)0x0C
#define UP_CMD_WRITE_FIRMWARE_CRC		(unsigned char)0x10
#define UP_CMD_GET_APP_INFO_B1      (unsigned char)0x12/*+0x01*/
#define UP_CMD_GET_FIRMWARE_CRC_B1  (unsigned char)0x12/*+0x03*/
#define UP_CMD_GET_APP_INFO_B2      (unsigned char)0x01/*+0x01*/
#define UP_CMD_GET_FIRMWARE_CRC_B2  (unsigned char)0x03/*+0x03*/

//#define UP_CMD_UNUSED_11			      (unsigned char)0x11	/*Не используется*/
// ------------------------------------------------------------------------
// резерв 0x13-0x2F
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// 0x30-0x3F - стандартные команды управления
// ------------------------------------------------------------------------
#define UP_CMD_POWER_OFF			      (unsigned char)0x30
#define UP_CMD_POWER_ON			        (unsigned char)0x31
#define UP_CMD_SET_FREQ_MHz		      (unsigned char)0x32
#define UP_CMD_SET_ATT				      (unsigned char)0x33
#define UP_CMD_READ_STATUS			    (unsigned char)0x34
#define UP_CMD_SELECT_DEVICE_MODE	  (unsigned char)0x35
#define UP_CMD_FILTER_SELECT		    (unsigned char)0x36
#define UP_CMD_SET_BW				        (unsigned char)0x36
#define UP_CMD_SET_SWITCH			      (unsigned char)0x37
#define UP_CMD_RF_OUT_CONFIG		    (unsigned char)0x37
#define UP_CMD_SET_IN_SWITCH		    (unsigned char)0x38
#define UP_CMD_SET_OUT_SWITCH		    (unsigned char)0x39
#define UP_CMD_GET_AUX_PORT		      (unsigned char)0x3D
#define UP_CMD_SET_AUX_PORT		      (unsigned char)0x3E
#define UP_CMD_SET_LNA_MODE		      (unsigned char)0x3A

// свободно 0x3A - 0x3F
// ------------------------------------------------------------------------
// 0x40-0x7F - пользовательские команды управления
// (уникальны для каждого типа устройства)
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// 0x80-0x9F - ответы
// ------------------------------------------------------------------------
#define UP_ANS_OK					          (unsigned char)0x80
#define UP_ANS_MAC_ADDRESS			    (unsigned char)0x81
#define UP_ANS_NETPARAMS			      (unsigned char)0x82
#define UP_ANS_DEVICE_INFO			    (unsigned char)0x83
// 0x84 - ответ от загрузчика - выше

// свободно 0x85-0x89

#define UP_ANS_STATUS				        (unsigned char)0x8A
#define UP_ANS_TCP_PORT_STATE		    (unsigned char)0x8B

// свободно 0x8C-0x8D

#define UP_ANS_USER_INFO			      (unsigned char)0x8E
#define UP_ANS_DELAYED_CMD			    (unsigned char)0x8F
#define UP_INFO_PACK				        (unsigned char)0x90

// свободно 0x91-0x9F

// ------------------------------------------------------------------------
// 0xA0-0xCF - служебные
// ------------------------------------------------------------------------

// 0xA0-0xC0 - свободно
#define UP_CMD_PACK_ID_01			      (unsigned char)0xC1
#define UP_CMD_PACK_ID_02			      (unsigned char)0xC2
// C3 - резерв
#define UP_CMD_PACK_ID_04			      (unsigned char)0xC4
// 0xC5-0xBF - свободно

// ------------------------------------------------------------------------
// 0xD0-0xDF - CMD_EXT_ADDR
// ------------------------------------------------------------------------
#define UP_CMD_EXT_ADDR			        (unsigned char)0xD0

// ------------------------------------------------------------------------
// 0xE0-0xEF - свободно

// ------------------------------------------------------------------------
// 0xF0…0xFF - ответы

#define UP_ANS_PROTO_INFO			      (unsigned char)0xF1
#define UP_ACK_CMD					        (unsigned char)0xF2
#define UP_ACK_BUSY				          (unsigned char)0xF3

#define UP_ANS_ERROR2				        (unsigned char)0xFE
#define UP_ANS_ERROR				        (unsigned char)0xFF

//---Дополнительное поле ошибки---//
#define UP_ERROR_LENGTH_FIELD      (unsigned char)0x51 /*Неправильная длинна команды*/
#define UP_ERROR_DATAFLASH         (unsigned char)0x52 /*Ошибка DataFlash памяти*/
#define UP_ERROR_FILE_SIZE         (unsigned char)0x53 /*Некорректный размер файла*/
#define UP_ERROR_BLOCK_NUMBER      (unsigned char)0x54 /*Некорректный номер блока файла*/
#define UP_ERROR_PARAMETER         (unsigned char)0x55 /*Некорректный параметр команды*/
#define UP_ERROR_HARDWARE          (unsigned char)0x56 /*Ошибка выполнения команды из-за аппаратной неисправности. Независимо от данной ошибки устройство переходит в требуемый режим работы*/
#define UP_ERROR_NOT_SUPPORTED     (unsigned char)0x57 /*Операция не поддерживается (неизвестная команда)*/
#define UP_ERROR_ILLEGAL_CMD       (unsigned char)0x58 /*Команда недопустима для данного режима работы*/
#define UP_ERROR_BUSY_DEVICE       (unsigned char)0x59 /*Устройство занято (выполняется предыдущая команда), команда игнорирована*/
#define UP_ERROR_CRC               (unsigned char)0x5B /*ошибка контрольной суммы пакета*/

#define UP_ERROR_DEVICE_NOT_FOUND  (unsigned char)0x5A /*Устройство, которому предназначена команда, не отвечает*/
#define UP_ERROR_ADDR              (unsigned char)0x5D /*Ошибка адреса устройства*/
#define UP_ERROR_CMD_TIMEOUT       (unsigned char)0x5E /*Ошибка времени выполнения команды (команда выполняется подчинённым устройством слишком долго)*/
#define UP_ERROR_BUSY_BUS          (unsigned char)0x5F /*Шина управления занята (невозможно передать команду)*/
//----------------------------//

unsigned char UP_CRC_UART(unsigned char* data, unsigned short len);

unsigned short UPRequestCallback(unsigned char* packetData,unsigned short packetDataLenght, unsigned char* outBufferData);

unsigned short processUPDebugRequestDataCallback(unsigned char* packetData,unsigned short packetDataLenght, unsigned char* outBufferData);
unsigned short processUPDeviceRequestDataCallback(unsigned char* packetData,unsigned short packetDataLenght, unsigned char* outBufferData);

#ifdef __cplusplus
 }
#endif
#endif
