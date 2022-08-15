#include "LSM9DS0.h"
#include "TimeSpan.h"

//---delete---//
uint16_t lsm9ds0ErrorCounter=0;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}Point;

Point accelerometrDataArray[33];
Point magnetometerDataArray[33];

//-----------------------------//

__weak void adxlAccelVectorReceiveCallback(float adxl357_Vector[3])
{
	
}
__weak void adxlMagVectorReceiveCallback(float adxl357_Vector[3])
{
	
}

__weak void adxlTemperatureReceiveCallback(float temperature)
{
	
}

void _rxCpltCallback(void* _lsm9ds0)
{
	LSM9DS0* lsm9ds0 = (void*)_lsm9ds0;
	static uint8_t vectorCode[6];
//	static uint8_t regState=0x00;
	
	if(lsm9ds0->readStage == LSM9DS0ReadTemperature)
	{				
    lsm9ds0->readStage = LSM9DS0ReadyToRead;		
		float temperature = (lsm9ds0->temperatureCode[1]<<8)|lsm9ds0->temperatureCode[0];//((int)((lsm9ds0->temperatureCode[0]&0x0F)<<8 | lsm9ds0->temperatureCode[1]) - 1852)/-9.05F + 25.0F;		
		adxlTemperatureReceiveCallback(temperature);
		
		lsm9ds0->readStage = LSM9DS0ReadNumFifoElemAccelerometer;
		HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,FIFO_SRC_REG,1,&lsm9ds0->numberOfFIFOElemAccelerometer,1);
		/*lsm9ds0->readStage = LSM9DS0ReadVectors;
		HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,OUT_X_L_A|0x80,1,vectorCode,6);*/
	}
	else if(lsm9ds0->readStage == LSM9DS0ReadNumFifoElemAccelerometer)
	{		
		lsm9ds0->numberOfFIFOElemAccelerometer = lsm9ds0->numberOfFIFOElemAccelerometer&0x1F;
		
		if(lsm9ds0->numberOfFIFOElemAccelerometer==0)//произошел удар, защита
		{
			/*lsm9ds0->readStage = LSM9DS0InitStage1;//LSM9DS0ReadyToRead;
			regState = 0x40|0x1F;//Stream mode, FIFO watermark level - 20//0x7F;//Stream-to-FIFO mode, FIFO watermark level - 32
	    lsm9ds0->status = HAL_I2C_Mem_Write_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,FIFO_CTRL_REG,1,&regState,1);*/
			lsm9ds0->readStage =LSM9DS0Error;
			
			return;
		}		
		
		if(lsm9ds0->numberOfFIFOElemAccelerometer>31)lsm9ds0ErrorCounter++;	
		
		lsm9ds0->currentFIFOElem=0;
		lsm9ds0->readStage = LSM9DS0ReadVectorsAccelerometer;
		HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,OUT_X_L_A|0x80,1,vectorCode,6);
		//lsm9ds0->status = HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1, ADXL357_FIFO_DATA,I2C_MEMADD_SIZE_8BIT,vectorCode,9);
	}
	else if(lsm9ds0->readStage == LSM9DS0ReadVectorsAccelerometer)
	{								
		lsm9ds0->adxl_AccelVector[0] = (int16_t)((vectorCode[1]<<8)|vectorCode[0])*2.0F/32767.0F;
		lsm9ds0->adxl_AccelVector[1] = (int16_t)((vectorCode[3]<<8)|vectorCode[2])*2.0F/32767.0F;
		lsm9ds0->adxl_AccelVector[2] = (int16_t)((vectorCode[5]<<8)|vectorCode[4])*2.0F/32767.0F;
		adxlAccelVectorReceiveCallback(lsm9ds0->adxl_AccelVector);
		
		accelerometrDataArray[lsm9ds0->currentFIFOElem].x=(int16_t)((vectorCode[1]<<8)|vectorCode[0]);
		accelerometrDataArray[lsm9ds0->currentFIFOElem].y=(int16_t)((vectorCode[3]<<8)|vectorCode[2]);
		accelerometrDataArray[lsm9ds0->currentFIFOElem].z=(int16_t)((vectorCode[5]<<8)|vectorCode[4]);
		
		if(++lsm9ds0->currentFIFOElem<lsm9ds0->numberOfFIFOElemAccelerometer)HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,OUT_X_L_A|0x80,1,vectorCode,6);//lsm9ds0->status = HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1, ADXL357_FIFO_DATA,I2C_MEMADD_SIZE_8BIT,vectorCode,9);
		else //lsm9ds0->readStage = LSM9DS0ReadyToRead;
		{
			lsm9ds0->currentFIFOElem=0;
			lsm9ds0->readStage = LSM9DS0ReadVectorsMagnetometer;
			HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,OUT_X_L_M|0x80,1,vectorCode,6);//Дальше читаем показания магнитометра с учетом того что в оччереди столько же элементов
		}
	}	
	//Магнитометр
	else if(lsm9ds0->readStage == LSM9DS0ReadVectorsMagnetometer)
	{								
		lsm9ds0->adxl_MagVector[0] = (int16_t)((vectorCode[1]<<8)|vectorCode[0])*2.0F/32767.0F;
		lsm9ds0->adxl_MagVector[1] = (int16_t)((vectorCode[3]<<8)|vectorCode[2])*2.0F/32767.0F;
		lsm9ds0->adxl_MagVector[2] = (int16_t)((vectorCode[5]<<8)|vectorCode[4])*2.0F/32767.0F;
		adxlMagVectorReceiveCallback(lsm9ds0->adxl_MagVector);
		
		magnetometerDataArray[lsm9ds0->currentFIFOElem].x=(int16_t)((vectorCode[1]<<8)|vectorCode[0]);
		magnetometerDataArray[lsm9ds0->currentFIFOElem].y=(int16_t)((vectorCode[3]<<8)|vectorCode[2]);
		magnetometerDataArray[lsm9ds0->currentFIFOElem].z=(int16_t)((vectorCode[5]<<8)|vectorCode[4]);
		
		if(++lsm9ds0->currentFIFOElem<lsm9ds0->numberOfFIFOElemAccelerometer)HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,OUT_X_L_M|0x80,1,vectorCode,6);//lsm9ds0->status = HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1, ADXL357_FIFO_DATA,I2C_MEMADD_SIZE_8BIT,vectorCode,9);
		else lsm9ds0->readStage = LSM9DS0ReadyToRead;
	}
	/*else if(lsm9ds0->readStage == LSM9DS0InitStage1)
	{	
		lsm9ds0->readStage = LSM9DS0InitStage2;
		regState = 0x40|0x20;//FIFO enable, watermark enable 
	  lsm9ds0->status = HAL_I2C_Mem_Write_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG0_XM,1,&regState,1);
	}
	else if(lsm9ds0->readStage == LSM9DS0InitStage2)
	{	
		lsm9ds0->readStage = LSM9DS0InitStage3;
		regState = 0x60|0x07;//acceleration data rate 100Hz, Axisses X,Y,Z enable
	  lsm9ds0->status = HAL_I2C_Mem_Write_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG1_XM,1,&regState,1);
	}
	else if(lsm9ds0->readStage == LSM9DS0InitStage3)
	{	
		lsm9ds0->readStage = LSM9DS0InitStage4;
		regState = 0xC0;//50Hz anti-aliasing filter
	  lsm9ds0->status = HAL_I2C_Mem_Write_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG2_XM,1,&regState,1);
	}
	else if(lsm9ds0->readStage == LSM9DS0InitStage4)
	{	
		lsm9ds0->readStage = LSM9DS0InitStage5;
		regState = 0x04;
	  lsm9ds0->status = HAL_I2C_Mem_Write_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG3_XM,1,&regState,1);
	}
	else if(lsm9ds0->readStage == LSM9DS0InitStage5)
	{	
		lsm9ds0->readStage = LSM9DS0InitStage6;
		regState = 0x80|0x60|0x14|0x00;//Включаем датчик температуры, устанавливаем высокое разрешение датчика магнитного поля, устанавливаем частоту измерения температуры и магнитного поля 100Гц, Lutch interrupt request disabled
	  lsm9ds0->status = HAL_I2C_Mem_Write_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG5_XM,1,&regState,1);
	}
	else if(lsm9ds0->readStage == LSM9DS0InitStage6)
	{	
		lsm9ds0->readStage = LSM9DS0ReadyToRead;
		regState = 0x00;
	  lsm9ds0->status = HAL_I2C_Mem_Write_IT(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG7_XM,1,&regState,1);
	}*/
}

HAL_StatusTypeDef LSM9DS0_Init(LSM9DS0* lsm9ds0)
{	
	//lsm9ds0->amplitudeRange=0x02;//2g//0x08;//8g
	lsm9ds0->isConnected=0;
	lsm9ds0->devID=0x00;
	lsm9ds0->numberOfFIFOElemAccelerometer =0x00;
	lsm9ds0->rxCpltCallback=_rxCpltCallback;
	lsm9ds0->readStage = LSM9DS0ReadyToRead;
	
	HAL_Delay(500);
	
	uint8_t readIDCounter=0;
	while((lsm9ds0->status = HAL_I2C_Mem_Read(lsm9ds0->i2c, X_M_ADDRESS<<1, WHO_AM_I_XM,I2C_MEMADD_SIZE_8BIT,&lsm9ds0->devID,1,100))!=HAL_OK)//Пробуем прочитать, если не получится - сигнал аварии
	{
		HAL_Delay(500);		
		if(++readIDCounter==10)return (lsm9ds0->status = HAL_ERROR);
	}
	//HAL_I2C_Mem_Read(lsm9ds0->i2c, G_ADDRESS<<1, WHO_AM_I_G,1,&gTest,1,1000);
  
	uint8_t regState =0x00;
	//---Инициализация Гироскопа (выключение)---//
	//При включении находится в режиме power-down mode, поэтому можно не настраивать ничего	
	//---Конец инициализации гироскопа---//
	
	//---Инициализация Акселерометра---//
	regState = 0x40|0x1F;//Stream mode, FIFO watermark level - 20//0x7F;//Stream-to-FIFO mode, FIFO watermark level - 32
	lsm9ds0->status = HAL_I2C_Mem_Write(lsm9ds0->i2c,X_M_ADDRESS<<1,FIFO_CTRL_REG,1,&regState,1,100);
	
	/* CTRL_REG0_XM (0x1F) (Default value: 0x00)
	Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
	BOOT - Reboot memory content (0: normal, 1: reboot)
	FIFO_EN - Fifo enable (0: disable, 1: enable)
	WTM_EN - FIFO watermark enable (0: disable, 1: enable)
	HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
	HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
	HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled)   */	
	regState = 0x40|0x20;//FIFO enable (stream mode), watermark enable 
	lsm9ds0->status = HAL_I2C_Mem_Write(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG0_XM,1,&regState,1,100);
	
	/* CTRL_REG1_XM (0x20) (Default value: 0x07)
	Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
	AODR[3:0] - select the acceleration data rate:
		0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz, 
		0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
		1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
	BDU - block data update for accel AND mag
		0: Continuous update
		1: Output registers aren't updated until MSB and LSB have been read.
	AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
		0: Axis disabled, 1: Axis enabled									 */	
	regState=0x60|0x07;//acceleration data rate 100Hz, Axisses X,Y,Z enable
	lsm9ds0->status = HAL_I2C_Mem_Write(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG1_XM,1,&regState,1,100);
	
	/* CTRL_REG2_XM (0x21) (Default value: 0x00)
	Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
	ABW[1:0] - Accelerometer anti-alias filter bandwidth
		00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
	AFS[2:0] - Accel full-scale selection
		000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
	AST[1:0] - Accel self-test enable
		00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
	SIM - SPI mode selection
		0=4-wire, 1=3-wire													 */	
	regState=0xC0;//50Hz anti-aliasing filter
	lsm9ds0->status = HAL_I2C_Mem_Write(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG2_XM,1,&regState,1,100);
	
	/* CTRL_REG3_XM is used to set interrupt generators on INT1_XM
	Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY
	*/
	// Accelerometer data ready on INT1_XM (0x04)
  regState=0x04;
	lsm9ds0->status = HAL_I2C_Mem_Write(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG3_XM,1,&regState,1,100);
  //---Конец инициализации акселерометра---//
		
	//---Инициализация датчика температуры---//
	regState = 0x01<<5;//Разрешение датчика магнитного поля 4 Гаусса
	lsm9ds0->status = HAL_I2C_Mem_Write(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG6_XM,1,&regState,1,100);
	regState = 0x80|0x60|0x14|0x00;//Включаем датчик температуры, устанавливаем высокое разрешение датчика магнитного поля, устанавливаем частоту измерения температуры и магнитного поля 100Гц, Lutch interrupt request disabled
	lsm9ds0->status = HAL_I2C_Mem_Write(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG5_XM,1,&regState,1,100);
	//---Конец инициализации датчика температуры---//

  //---Инициализация датчика температуры---//
	regState = 0x00;
	lsm9ds0->status = HAL_I2C_Mem_Write(lsm9ds0->i2c,X_M_ADDRESS<<1,CTRL_REG7_XM,1,&regState,1,100);
	
	return HAL_OK;
}

void LSM9DS0_Loop(LSM9DS0* lsm9ds0)
{
	static uint32_t lastReadDataTime = 0;
	static uint32_t lastReadyToReadTime=0;
	
	if(lsm9ds0->readStage == LSM9DS0Error)
	{
		LSM9DS0_Init(lsm9ds0);
		return;
	}
	
	if(GetTimeSpan(lastReadDataTime, HAL_GetTick())>50)//Время заполнения буфера 96/3 = 32 - 256 мс, чтобы не опрашивать постоянно - опрашиваем с интервалом 100 мс
	{
		lastReadDataTime = HAL_GetTick();
		if(lsm9ds0->readStage == LSM9DS0ReadyToRead || GetTimeSpan(lastReadyToReadTime, HAL_GetTick())>500)
		{	
			lastReadyToReadTime = HAL_GetTick();
			lsm9ds0->readStage = LSM9DS0ReadTemperature;
		  lsm9ds0->status = HAL_I2C_Mem_Read_IT(lsm9ds0->i2c,X_M_ADDRESS<<1, OUT_TEMP_L_XM|0x80,I2C_MEMADD_SIZE_8BIT,lsm9ds0->temperatureCode,2);
		}
	}
}




