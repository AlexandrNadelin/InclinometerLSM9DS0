#include "UPCommon.h"
#include "Memory.h"
#include <string.h>
#include <stdio.h>

typedef struct{
	char Descript_ID[7];	// “SOFT_VER”
	uint8_t Descript_ver;	// Номер версии описания (текущая версия #2)
	char MC_type[16];		// Тип микроконтроллера, для которого предназначена прошивка
	char Device_type[16];	// Название устройства, для которого предназначено ПО
	char Soft_type[16];		// Название ПО
	uint8_t Soft_ver[2];	// Версия ПО
	uint16_t Soft_build;	// Номер билда ПО
	char Comp_date[16];		// Дата компиляции ПО
	char Comp_time[16];		// Время компиляции ПО
	uint32_t Start_addr;	// Абсолютный адрес расположения ПО в памяти микроконтроллера
}soft_descript_v2;

//Структура дескриптора устройства
const soft_descript_v2 soft_descript =
{
	"SOFTVER",			//uint8_t Descript_ID[7];	// “SOFT_VER”
	2,					//uint8_t Descript_ver;		// Номер версии описания (текущая версия #2)
	"STM32",			//uint8_t MC_type[16];		// Тип микроконтроллера, для которого предназначена прошивка
	"Inclinometer",/*"Apogee_OPU",*/ 			//uint8_t Device_type[16];	// Название устройства, для которого предназначено ПО
	"application",		//uint8_t Soft_type[16];	// Название/тип ПО
	{1,0},				//uint8_t Soft_ver[2];		// Версия ПО
	1,					//uint16_t Soft_build;		// Номер билда ПО
	__DATE__,			//uint8_t Comp_date[16];//12	// Дата компиляции ПО
	__TIME__,			//uint8_t Comp_time[16];//8		// Время компиляции ПО
	0x08020000			//uint32_t Start_addr;		// Абсолютный адрес расположения ПО в памяти микроконтроллера
};

// Информация об устройстве
const char DeviceInfo[] ="\
DEV=%s\r\n\
FV=%u.%ub%u(%s)\r\n\
";

__attribute__((weak)) unsigned short processUPDebugRequestDataCallback(unsigned char* packetData,unsigned short packetDataLenght, unsigned char* outBufferData)
{
  (void)packetData;//UNUSED
  (void)packetDataLenght;//UNUSED
  (void)outBufferData;//UNUSED
	return 0;
}

__attribute__((weak)) unsigned short processUPDeviceRequestDataCallback(unsigned char* packetData,unsigned short packetDataLenght, unsigned char* outBufferData)
{
  (void)packetData;//UNUSED
  (void)packetDataLenght;//UNUSED
  (void)outBufferData;//UNUSED
	return 0;
}

typedef struct{
		uint32_t rezerved_1;// 0
		uint32_t start_magic_flag;// 1 кодовое слово для запуска загрузчика/приложения
		uint32_t boot_mode_flags;// 2 режим загрузчика
		uint8_t lit_addr;// 3 адрес литеры устройства
		uint8_t chan_addr;// адрес канала устройства
		uint16_t rezerved_2;
		uint8_t boot_ver[2];// 4 версия загрузчика
		uint16_t boot_build;// номер билда загрузчика

		// 5 сетевые параметры
		uint8_t ip_addr[4];	// 5
		uint8_t ip_mask[4];	// 6
		uint8_t ip_gate[4];	// 7
		uint32_t ip_port;	// 8

		uint8_t mac_addr[6];//9,10
		//
		uint16_t rezerved_3;
		// 11..15
		uint32_t unused_11;
		uint32_t unused_12;
		uint32_t unused_13;
		uint32_t unused_14;
		uint32_t unused_15; // 15
}boot_ram_info_t;

//Функция рассчета контрольной суммы пакета UART
unsigned char UP_CRC_UART(unsigned char* data, unsigned short len)
{
  uint8_t ret = 0x00;//-Версия Димы //0xFF; - Версия Степана
  for (uint16_t i = 0; i < len; i++)
	{
       ret ^= data[i];
  }
  return ret;
}


/*void jump_to_boot_tcp(void)
{
	__disable_irq();

	boot_ram_info_t *ramPtr = (void*)(0x20000000);
	uint32_t* uptr = (void*)(0x20000000);

	// очистить структуру
	for (unsigned int i = 0; i < sizeof(boot_ram_info_t)/4; i++)
		uptr[i] = 0;

	ramPtr->start_magic_flag = 0x2A58;//BOOT_STARTING_FROM_MAIN_SOFT_PASS32b;
	ramPtr->lit_addr = 0xFF;
	ramPtr->chan_addr = 0xFF;

	ramPtr->ip_port = new_ETH_parameter.PORT_num == 10000 ? 9999 : 10000;//Если порт 10000 используется по протоколу КБ-4, для универсального протокола используем порт 9999//BOOT_UPDATE_PORT;

	// прочитать параметры текущего интерфейса
	*(uint32_t *)ramPtr->ip_addr = gnetif.ip_addr.addr;
	*(uint32_t *)ramPtr->ip_mask = gnetif.netmask.addr;
	*(uint32_t *)ramPtr->ip_gate = gnetif.gw.addr;
	for (int i = 0; i < 6; i++)
	{
		ramPtr->mac_addr[i] = gnetif.hwaddr[i];
	}
	__ASM volatile ("" ::: "memory");

	NVIC_SystemReset();
}*/

//Подготавливает ответ
unsigned short UPRequestCallback(unsigned char* packetData,unsigned short packetDataLenght, unsigned char* outBufferData)
{
	unsigned short len=0;
	if(packetData[0]==UP_CMD_CHECK_LINK)
	{
		outBufferData[0]=UP_ANS_OK;//Ответ - все хорошо
		outBufferData[1]=packetData[0];//Команда на которую выдается ответ

		return 2;
	}
	/*else if(packetData[0]==UP_CMD_JUMP_TO_BOOTLOADER)
	{
		outBufferData[0]=UP_ANS_OK;//Ответ - все хорошо
		outBufferData[1]=packetData[0];//Команда на которую выдается ответ

		//jump_to_boot();

		UPJumpToBootloaderCallback();

		return 2;
	}*/
	else if(packetData[0]==UP_CMD_GET_DEVICE_INFO)
	{
		outBufferData[0]=UP_ANS_DEVICE_INFO;//Ответ - все хорошо

		unsigned short len = sprintf((char*)&outBufferData[1],DeviceInfo,soft_descript.Device_type,soft_descript.Soft_ver[0],soft_descript.Soft_ver[1],soft_descript.Soft_build,soft_descript.Comp_date);
		outBufferData[1+len]=0;// 0 в конце строки
		return len + 2;//байт команды + 0 в конце строки
	}
	else if((packetData[0]==UP_CMD_DEBUG)&&((len = processUPDebugRequestDataCallback(packetData,packetDataLenght,outBufferData))>0))return len;//Отладочная команда
	else if((len = processUPDeviceRequestDataCallback(packetData,packetDataLenght,outBufferData))>0)return len;//Остальные команды могут относиться к самому устройству
	else//unsupported
	{

		outBufferData[0]=UP_ANS_ERROR;//Команда не поддерживается => ошибка
		outBufferData[1]=packetData[0];//Команда на которую выдается ответ
		outBufferData[2]=UP_ERROR_NOT_SUPPORTED;//Дополнительное поле ответа ошибки - команда не поддерживается
		return 3;
	}
}





