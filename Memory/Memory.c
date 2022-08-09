#include "Memory.h"

uint16_t Crc16_D0DB(uint16_t *pData, int length)
{
  uint64_t crc = 0; 
  volatile int i;
  for (i=0; i < length; i++)crc += pData[i];
 
  return (uint16_t)(crc%0xD0DB);//~(uint16_t)crc+1;
}

void memoryWriteHoldingRegisters(uint16_t* registers,uint16_t startAddr, uint16_t regCount)
{
	uint16_t additionCRCPage1;
	uint32_t PageErr;
	FLASH_EraseInitTypeDef EraseInitStruct;
	volatile int i=0;
			
	HAL_FLASH_Unlock();
		
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FIRST_PAGE_PROPERTY_ADDR;
	EraseInitStruct.NbPages=1;
		
	HAL_StatusTypeDef erraseResult = HAL_FLASHEx_Erase(&EraseInitStruct,&PageErr);
	
	for(i=0;i<startAddr;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,FIRST_PAGE_PROPERTY_ADDR+i*2,(*(uint16_t*)(SECOND_PAGE_PROPERTY_ADDR+i*2)));
	}
		
	for(i=startAddr;i<startAddr+regCount;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,FIRST_PAGE_PROPERTY_ADDR + i*2,registers[i-startAddr]);
	}
	
	for(i=startAddr+regCount;i<FIRST_PAGE_PROPERTY_SIZE/2-1;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,FIRST_PAGE_PROPERTY_ADDR+2*i,(*(uint16_t*)(SECOND_PAGE_PROPERTY_ADDR+2*i)));
	}
									
	additionCRCPage1 = Crc16_D0DB((uint16_t*)FIRST_PAGE_PROPERTY_ADDR,FIRST_PAGE_PROPERTY_SIZE/2-1);
	additionCRCPage1 = 0xD0DB-additionCRCPage1;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FIRST_PAGE_PROPERTY_ADDR+FIRST_PAGE_PROPERTY_SIZE-2, additionCRCPage1);
		
	//-----------------------//
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = SECOND_PAGE_PROPERTY_ADDR;
	EraseInitStruct.NbPages=1;
		
	HAL_FLASHEx_Erase(&EraseInitStruct,&PageErr);
							
	for(i=0;i<SECOND_PAGE_PROPERTY_SIZE/4;i++)HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,SECOND_PAGE_PROPERTY_ADDR+4*i,*(uint32_t*)(FIRST_PAGE_PROPERTY_ADDR+4*i));
		
	HAL_FLASH_Lock();
}

void Memory_Init()
{
	Memory* memory = (Memory*)FIRST_PAGE_PROPERTY_ADDR;
	
	uint16_t crcPage1=0xFFFF;
  uint16_t crcPage2=0xFFFF;
  uint16_t arrayTmp[2];
  uint16_t additionCRCPage1;
	
	arrayTmp[0]=Crc16_D0DB((uint16_t*)FIRST_PAGE_PROPERTY_ADDR,FIRST_PAGE_PROPERTY_SIZE/2-1);
	arrayTmp[1]=*(uint16_t*)(FIRST_PAGE_PROPERTY_ADDR+FIRST_PAGE_PROPERTY_SIZE-2);
	crcPage1=Crc16_D0DB(arrayTmp,2);
		
	arrayTmp[0]=Crc16_D0DB((uint16_t*)SECOND_PAGE_PROPERTY_ADDR,SECOND_PAGE_PROPERTY_SIZE/2-1);
	arrayTmp[1]=*(uint16_t*)(SECOND_PAGE_PROPERTY_ADDR+SECOND_PAGE_PROPERTY_SIZE-2);
	crcPage2=Crc16_D0DB(arrayTmp,2);
	
	if(crcPage1!=0&&crcPage2!=0)
	{
		uint32_t PageErr;
		FLASH_EraseInitTypeDef EraseInitStruct;
			
	  HAL_FLASH_Unlock();
		
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = FIRST_PAGE_PROPERTY_ADDR;
	  EraseInitStruct.NbPages=1;		
		
	  HAL_FLASHEx_Erase(&EraseInitStruct,&PageErr);
		
		//Настройки RS485
		UartProperty uartProperty = {
			.uartBaudrate=9600,
			.deviceAddress=0x01,
		};
		
		for(int i=0;i<sizeof(UartProperty)/4;i++)
	  {
		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)(&memory->uartProperty) + 4*i,(*(uint32_t*)((uint32_t)(&uartProperty)+4*i)));
	  }
		
		
		AdjPoint point;
		point.temperature = 300;//Чтобы при не откалиброванном датчике измеряемая температура была заведомо меньше//
	  point.zeroOffsetVector[0]=0;//Вектор смещения нуля X
	  point.zeroOffsetVector[1]=0;//Вектор смещения нуля Y
	  point.zeroOffsetVector[2]=0;//Вектор смещения нуля Z
		
	  point.transitionMatrix[0][0]=1;//Матрица перехода
	  point.transitionMatrix[0][1]=0;
	  point.transitionMatrix[0][2]=0;
	  point.transitionMatrix[1][0]=0;
	  point.transitionMatrix[1][1]=1;
	  point.transitionMatrix[1][2]=0;
	  point.transitionMatrix[2][0]=0;
	  point.transitionMatrix[2][1]=0;
	  point.transitionMatrix[2][2]=1;
		
		for(int j=0;j<ADJ_POINTS_COUNT;j++)
		{
			for(int i=0;i<sizeof(AdjPoint)/4;i++)
	    {
		    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)(&memory->adjPoints[j]) + 4*i,(*(uint32_t*)((uint32_t)(&point)+4*i)));
	    }
		}		
		
		additionCRCPage1 = Crc16_D0DB((uint16_t*)FIRST_PAGE_PROPERTY_ADDR,FIRST_PAGE_PROPERTY_SIZE/2-1);
		additionCRCPage1 = 0xD0DB-additionCRCPage1;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FIRST_PAGE_PROPERTY_ADDR+FIRST_PAGE_PROPERTY_SIZE-2, additionCRCPage1);
		
		//-----------------------//
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = SECOND_PAGE_PROPERTY_ADDR;
	  EraseInitStruct.NbPages=1;		
		
	  HAL_FLASHEx_Erase(&EraseInitStruct,&PageErr);
							
		for(int i=0;i<SECOND_PAGE_PROPERTY_SIZE/4;i++)HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,SECOND_PAGE_PROPERTY_ADDR+4*i,*(uint32_t*)(FIRST_PAGE_PROPERTY_ADDR+4*i));
		
		HAL_FLASH_Lock();
	}
	else if(crcPage1!=0)
	{
		uint32_t PageErr;
		FLASH_EraseInitTypeDef EraseInitStruct;
			
	  HAL_FLASH_Unlock();
		
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = FIRST_PAGE_PROPERTY_ADDR;
	  EraseInitStruct.NbPages=1;
		
	  HAL_FLASHEx_Erase(&EraseInitStruct,&PageErr);
							
		for(int i=0;i<FIRST_PAGE_PROPERTY_SIZE/4;i++)HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FIRST_PAGE_PROPERTY_ADDR+4*i,*(uint32_t*)(SECOND_PAGE_PROPERTY_ADDR+4*i));
		
		HAL_FLASH_Lock();
	}
	else if(crcPage2!=0)
	{
		uint32_t PageErr;
		FLASH_EraseInitTypeDef EraseInitStruct;
		volatile int i=0;
			
	  HAL_FLASH_Unlock();
		
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = SECOND_PAGE_PROPERTY_ADDR;
	  EraseInitStruct.NbPages=1;
		
	  HAL_FLASHEx_Erase(&EraseInitStruct,&PageErr);
							
		for(i=0;i<FIRST_PAGE_PROPERTY_SIZE/4;i++)HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,SECOND_PAGE_PROPERTY_ADDR+4*i,*(uint32_t*)(FIRST_PAGE_PROPERTY_ADDR+4*i));
		
		HAL_FLASH_Lock();
	}
}

