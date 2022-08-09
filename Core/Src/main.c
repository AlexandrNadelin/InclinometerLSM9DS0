/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RS485Connector.h"
#include "Memory.h"
#include "ModbusCommon.h"
#include "LSM9DS0.h"
#include "TimeSpan.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Memory* memory = (void*)FIRST_PAGE_PROPERTY_ADDR;

RS485Server rs485Server = {
	.tim = &htim4,
	.uart = &huart2,
	.RTS_GPIO_PORT = RS485_UART2_RTS_GPIO_Port,
  .RTS_PIN = RS485_UART2_RTS_Pin,
};

LSM9DS0 lsm9ds0 ={
	.i2c=&hi2c1,
	//.address = 0x1D,//0x53,//
	//.address = 0x1E,//Адрес акселерометра в составе LSM9DS0
	.isConnected = 0x00,
};

InclinometrData inclinometrData={
	.Pitch = 0.0F,             //Тангаж [град.]
	.Roll = 0.0F,              //Крен [град.]
	.AccelerationX = 0.0F,     //Ускорение по оси X [g]
	.AccelerationY = 0.0F,     //Ускорение по оси Y [g]
	.AccelerationZ = 0.0F,     //Ускорение по оси Z [g]
	.Temperature = -273.0F,    //Температура [град. C]
	.AccelerationRAWX = 0.0F,  //Ускорение по оси X до выполнения корректировки [g]
	.AccelerationRAWY = 0.0F,  //Ускорение по оси Y до выполнения корректировки [g]
	.AccelerationRAWZ = 0.0F,  //Ускорение по оси Z до выполнения корректировки [g]
	.VibroAcceleratuonRMS=0.0F,//Среднеквадратическое значение виброускорения (вибрация) [g]
};

const char infoStringFirstPart[]="Inclinometr ID_";
char infoString[sizeof(infoStringFirstPart)+24];
InfoRegisters infoRegisters={(uint8_t*)infoString,sizeof(infoString)+24};

InputRegisters inputRegisters={
  .registers = (uint16_t*)&inclinometrData,
  .count = sizeof(InclinometrData)/2
};

HoldingRegisters holdingRegisters={(uint16_t*)FIRST_PAGE_PROPERTY_ADDR,sizeof(Memory)/2};

ContactRegisters contactRegisters = {NULL,0};
CoilRegisters coilRegisters = {NULL,0};

//----------------------------
#define ACCEL_FREQUENCY_SUMPLING 100

float vibroAccelArrayX[ACCEL_FREQUENCY_SUMPLING];
float vibroAccelArrayY[ACCEL_FREQUENCY_SUMPLING];
float vibroAccelArrayZ[ACCEL_FREQUENCY_SUMPLING];
uint16_t vibroAccelCounter=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//if(huart->Instance==rs485Server.uart->Instance)
	RS485Server_uartTxSlot(&rs485Server);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//if(huart->Instance==rs485Server.uart->Instance)
	RS485Server_uartRxSlot(&rs485Server);	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==rs485Server.uart->Instance)
	{
		RS485Server_uartErrorSlot(&rs485Server);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==rs485Server.tim->Instance)RS485Server_timerSlot(&rs485Server);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance==lsm9ds0.i2c->Instance)
	{
		lsm9ds0.rxCpltCallback(&lsm9ds0);
	}
}

void adxlVectorReceiveCallback(float adxl357_Vector[3])
{
	static uint8_t isVectorAlreadyReceived = 0;
	
	inclinometrData.AccelerationRAWX = adxl357_Vector[0];
	inclinometrData.AccelerationRAWY = adxl357_Vector[1];
	inclinometrData.AccelerationRAWZ = adxl357_Vector[2];
	
	static float AccelTmpX;
	static float AccelTmpY;
	static float AccelTmpZ;
	
	/*if(isVectorAlreadyReceived==0)//Первый вызов, фильтрацию проводить смысла нет
	{
		isVectorAlreadyReceived=1;
		
		AccelTmpX = adxl357_Vector[0];
	  AccelTmpY = adxl357_Vector[1];
	  AccelTmpZ = adxl357_Vector[2];
	}
	else //применяем фильтр Калмана
	{
		AccelTmpX = AccelTmpX*0.96F + adxl357_Vector[0]*0.04F;//Коэффициенты взяты примерно, с учетом что выборка происходит с частотой 125 Гц
	  AccelTmpY = AccelTmpY*0.96F + adxl357_Vector[1]*0.04F;
	  AccelTmpZ = AccelTmpZ*0.96F + adxl357_Vector[2]*0.04F;
	}
	
	vibroAccelArrayX[vibroAccelCounter] = adxl357_Vector[0] - AccelTmpX;
	vibroAccelArrayY[vibroAccelCounter] = adxl357_Vector[1] - AccelTmpY;
	vibroAccelArrayZ[vibroAccelCounter] = adxl357_Vector[2] - AccelTmpZ;*/
	
	if(++vibroAccelCounter==ACCEL_FREQUENCY_SUMPLING)
	{
		vibroAccelCounter=0;
		float accelTmpX=0;
		float accelTmpY=0;
		float accelTmpZ=0;
		
		for(int i =0;i<ACCEL_FREQUENCY_SUMPLING;i++)
		{
			accelTmpX+=vibroAccelArrayX[i]*vibroAccelArrayX[i];
			accelTmpY+=vibroAccelArrayY[i]*vibroAccelArrayY[i];
			accelTmpZ+=vibroAccelArrayZ[i]*vibroAccelArrayZ[i];
		}
		accelTmpX = sqrtf(accelTmpX);
		accelTmpY = sqrtf(accelTmpY);
		accelTmpZ = sqrtf(accelTmpZ);
		inclinometrData.VibroAcceleratuonRMS = sqrtf(accelTmpX*accelTmpX + accelTmpY*accelTmpY + accelTmpZ*accelTmpZ);
	}
	
	if(memory->adjPoints[1].temperature==300.0F)//Если калибровка произведена только на одной точке
	{
		//Добавить смещение нуля
		float offsetX = memory->adjPoints[0].zeroOffsetVector[0];
		float offsetY = memory->adjPoints[0].zeroOffsetVector[1];
		float offsetZ = memory->adjPoints[0].zeroOffsetVector[2];
		
		if(isVectorAlreadyReceived==0)//Первый вызов, фильтрацию проводить смысла нет
	  {
		  isVectorAlreadyReceived=1;
		
		  AccelTmpX = (adxl357_Vector[0]+offsetX);
	    AccelTmpY = (adxl357_Vector[1]+offsetY);
	    AccelTmpZ = (adxl357_Vector[2]+offsetZ);
	  }
	  else //применяем фильтр Калмана
	  {
		  AccelTmpX = AccelTmpX*0.96F + (adxl357_Vector[0]+offsetX)*0.04F;//Коэффициенты взяты примерно, с учетом что выборка происходит с частотой 100 Гц
	    AccelTmpY = AccelTmpY*0.96F + (adxl357_Vector[1]+offsetY)*0.04F;
	    AccelTmpZ = AccelTmpZ*0.96F + (adxl357_Vector[2]+offsetZ)*0.04F;
	  }
	
	  vibroAccelArrayX[vibroAccelCounter] = (adxl357_Vector[0]+offsetX) - AccelTmpX;
	  vibroAccelArrayY[vibroAccelCounter] = (adxl357_Vector[1]+offsetY) - AccelTmpY;
	  vibroAccelArrayZ[vibroAccelCounter] = (adxl357_Vector[2]+offsetZ) - AccelTmpZ;
		
		/*AccelTmpX+= offsetX;
		AccelTmpY+= offsetY;
		AccelTmpZ+= offsetZ;*/
		
		//Умножить на матрицу перехода
		inclinometrData.AccelerationX = AccelTmpX*memory->adjPoints[0].transitionMatrix[0][0]+AccelTmpY*memory->adjPoints[0].transitionMatrix[1][0]+AccelTmpZ*memory->adjPoints[0].transitionMatrix[2][0];
		inclinometrData.AccelerationY = AccelTmpX*memory->adjPoints[0].transitionMatrix[0][1]+AccelTmpY*memory->adjPoints[0].transitionMatrix[1][1]+AccelTmpZ*memory->adjPoints[0].transitionMatrix[2][1];		
		inclinometrData.AccelerationZ = AccelTmpX*memory->adjPoints[0].transitionMatrix[0][2]+AccelTmpY*memory->adjPoints[0].transitionMatrix[1][2]+AccelTmpZ*memory->adjPoints[0].transitionMatrix[2][2];
	}
	else//Если калибровка произведена на группе точек, причем количество точек может быть произвольное количество меньше или равно 10
	{
		int i = 1;
	  for(; i<ADJ_POINTS_COUNT;i++)
	  {
		  if(inclinometrData.Temperature<memory->adjPoints[i].temperature)//Нашли диапазон в котором находится температура
		  {
				//рассчитать смещения нуля в соответствии с линейной аппроксимацией температурной зависимости, добавить смещение нуля
			  float offsetX = ((inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			              *(memory->adjPoints[i].zeroOffsetVector[0]-memory->adjPoints[i-1].zeroOffsetVector[0])
			              /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			              +memory->adjPoints[i-1].zeroOffsetVector[0]);
			  float offsetY = ((inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			              *(memory->adjPoints[i].zeroOffsetVector[1]-memory->adjPoints[i-1].zeroOffsetVector[1])
			              /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			              +memory->adjPoints[i-1].zeroOffsetVector[1]);
			  float offsetZ = ((inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			              *(memory->adjPoints[i].zeroOffsetVector[2]-memory->adjPoints[i-1].zeroOffsetVector[2])
			              /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			              +memory->adjPoints[i-1].zeroOffsetVector[2]);
				
				if(isVectorAlreadyReceived==0)//Первый вызов, фильтрацию проводить смысла нет
	      {
		      isVectorAlreadyReceived=1;
		
		      AccelTmpX = (adxl357_Vector[0]+offsetX);
	        AccelTmpY = (adxl357_Vector[1]+offsetY);
	        AccelTmpZ = (adxl357_Vector[2]+offsetZ);
	      }
	      else //применяем фильтр Калмана
	      {
		      AccelTmpX = AccelTmpX*0.96F + (adxl357_Vector[0]+offsetX)*0.04F;//Коэффициенты взяты примерно, с учетом что выборка происходит с частотой 100 Гц
	        AccelTmpY = AccelTmpY*0.96F + (adxl357_Vector[1]+offsetY)*0.04F;
	        AccelTmpZ = AccelTmpZ*0.96F + (adxl357_Vector[2]+offsetZ)*0.04F;
	      }
	
	      vibroAccelArrayX[vibroAccelCounter] = (adxl357_Vector[0]+offsetX) - AccelTmpX;
	      vibroAccelArrayY[vibroAccelCounter] = (adxl357_Vector[1]+offsetY) - AccelTmpY;
	      vibroAccelArrayZ[vibroAccelCounter] = (adxl357_Vector[2]+offsetZ) - AccelTmpZ;
		
		    /*AccelTmpX+= offsetX;
		    AccelTmpY+= offsetY;
		    AccelTmpZ+= offsetZ;*/
				
				//рассчитать коэффициеты матрицы перехода в соответствии с линейной аппроксимацией температурной зависимости 
				float x11,x12,x13,x21,x22,x23,x31,x32,x33;
				x11 = (inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			        *(memory->adjPoints[i].transitionMatrix[0][0]-memory->adjPoints[i-1].transitionMatrix[0][0])
			        /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			        +memory->adjPoints[i-1].transitionMatrix[0][0];
				x12 = (inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			        *(memory->adjPoints[i].transitionMatrix[0][1]-memory->adjPoints[i-1].transitionMatrix[0][1])
			        /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			        +memory->adjPoints[i-1].transitionMatrix[0][1];
				x13 = (inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			        *(memory->adjPoints[i].transitionMatrix[0][2]-memory->adjPoints[i-1].transitionMatrix[0][2])
			        /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			        +memory->adjPoints[i-1].transitionMatrix[0][2];
				x21 = (inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			        *(memory->adjPoints[i].transitionMatrix[1][0]-memory->adjPoints[i-1].transitionMatrix[1][0])
			        /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			        +memory->adjPoints[i-1].transitionMatrix[1][0];
				x22 = (inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			        *(memory->adjPoints[i].transitionMatrix[1][1]-memory->adjPoints[i-1].transitionMatrix[1][1])
			        /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			        +memory->adjPoints[i-1].transitionMatrix[1][1];
				x23 = (inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			        *(memory->adjPoints[i].transitionMatrix[1][2]-memory->adjPoints[i-1].transitionMatrix[1][2])
			        /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			        +memory->adjPoints[i-1].transitionMatrix[1][2];
				x31 = (inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			        *(memory->adjPoints[i].transitionMatrix[2][0]-memory->adjPoints[i-1].transitionMatrix[2][0])
			        /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			        +memory->adjPoints[i-1].transitionMatrix[2][0];
				x32 = (inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			        *(memory->adjPoints[i].transitionMatrix[2][1]-memory->adjPoints[i-1].transitionMatrix[2][1])
			        /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			        +memory->adjPoints[i-1].transitionMatrix[2][1];
				x33 = (inclinometrData.Temperature-memory->adjPoints[i-1].temperature)
			        *(memory->adjPoints[i].transitionMatrix[2][2]-memory->adjPoints[i-1].transitionMatrix[2][2])
			        /(memory->adjPoints[i].temperature-memory->adjPoints[i-1].temperature)
			        +memory->adjPoints[i-1].transitionMatrix[2][2];
							
			  //Умножить на матрицу перехода
				inclinometrData.AccelerationX = AccelTmpX*x11+AccelTmpY*x21+AccelTmpZ*x31;
		    inclinometrData.AccelerationY = AccelTmpX*x12+AccelTmpY*x22+AccelTmpZ*x32;		
		    inclinometrData.AccelerationZ = AccelTmpX*x13+AccelTmpY*x23+AccelTmpZ*x33;
		  }
	  }
		if(i == ADJ_POINTS_COUNT)//Значение температуры больше максимальной, забитой в калибровочной таблице
		{
			float offsetX = ((inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			            *(memory->adjPoints[i-1].zeroOffsetVector[0]-memory->adjPoints[i-2].zeroOffsetVector[0])
			            /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			            +memory->adjPoints[i-2].zeroOffsetVector[0]);
			float offsetY = ((inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			            *(memory->adjPoints[i-1].zeroOffsetVector[1]-memory->adjPoints[i-2].zeroOffsetVector[1])
			            /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			            +memory->adjPoints[i-2].zeroOffsetVector[1]);
			float offsetZ = ((inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			            *(memory->adjPoints[i-1].zeroOffsetVector[2]-memory->adjPoints[i-2].zeroOffsetVector[2])
			            /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			            +memory->adjPoints[i-2].zeroOffsetVector[2]);
		
		  if(isVectorAlreadyReceived==0)//Первый вызов, фильтрацию проводить смысла нет
	    {
		    isVectorAlreadyReceived=1;
		
		    AccelTmpX = (adxl357_Vector[0]+offsetX);
	      AccelTmpY = (adxl357_Vector[1]+offsetY);
	      AccelTmpZ = (adxl357_Vector[2]+offsetZ);
	    }
	    else //применяем фильтр Калмана
	    {
		    AccelTmpX = AccelTmpX*0.96F + (adxl357_Vector[0]+offsetX)*0.04F;//Коэффициенты взяты примерно, с учетом что выборка происходит с частотой 100 Гц
	      AccelTmpY = AccelTmpY*0.96F + (adxl357_Vector[1]+offsetY)*0.04F;
	      AccelTmpZ = AccelTmpZ*0.96F + (adxl357_Vector[2]+offsetZ)*0.04F;
	    }
	
	    vibroAccelArrayX[vibroAccelCounter] = (adxl357_Vector[0]+offsetX) - AccelTmpX;
	    vibroAccelArrayY[vibroAccelCounter] = (adxl357_Vector[1]+offsetY) - AccelTmpY;
	    vibroAccelArrayZ[vibroAccelCounter] = (adxl357_Vector[2]+offsetZ) - AccelTmpZ;
		
		  /*AccelTmpX+= offsetX;
		  AccelTmpY+= offsetY;
		  AccelTmpZ+= offsetZ;*/
				
			//рассчитать коэффициеты матрицы перехода в соответствии с линейной аппроксимацией температурной зависимости 
			float x11,x12,x13,x21,x22,x23,x31,x32,x33;
			x11 = (inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			     *(memory->adjPoints[i-1].transitionMatrix[0][0]-memory->adjPoints[i-2].transitionMatrix[0][0])
			     /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			      +memory->adjPoints[i-2].transitionMatrix[0][0];
			x12 = (inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			     *(memory->adjPoints[i-1].transitionMatrix[0][1]-memory->adjPoints[i-2].transitionMatrix[0][1])
			     /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			      +memory->adjPoints[i-2].transitionMatrix[0][1];
			x13 = (inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			     *(memory->adjPoints[i-1].transitionMatrix[0][2]-memory->adjPoints[i-2].transitionMatrix[0][2])
			     /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			      +memory->adjPoints[i-2].transitionMatrix[0][2];
			x21 = (inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			     *(memory->adjPoints[i-1].transitionMatrix[1][0]-memory->adjPoints[i-2].transitionMatrix[1][0])
			     /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			      +memory->adjPoints[i-2].transitionMatrix[1][0];
			x22 = (inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			     *(memory->adjPoints[i-1].transitionMatrix[1][1]-memory->adjPoints[i-2].transitionMatrix[1][1])
			     /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			      +memory->adjPoints[i-2].transitionMatrix[1][1];
			x23 = (inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			     *(memory->adjPoints[i-1].transitionMatrix[1][2]-memory->adjPoints[i-2].transitionMatrix[1][2])
			     /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			      +memory->adjPoints[i-2].transitionMatrix[1][2];
			x31 = (inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			     *(memory->adjPoints[i-1].transitionMatrix[2][0]-memory->adjPoints[i-2].transitionMatrix[2][0])
			     /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			      +memory->adjPoints[i-2].transitionMatrix[2][0];
			x32 = (inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			     *(memory->adjPoints[i-1].transitionMatrix[2][1]-memory->adjPoints[i-2].transitionMatrix[2][1])
			     /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			      +memory->adjPoints[i-2].transitionMatrix[2][1];
			x33 = (inclinometrData.Temperature-memory->adjPoints[i-2].temperature)
			     *(memory->adjPoints[i-1].transitionMatrix[2][2]-memory->adjPoints[i-2].transitionMatrix[2][2])
			     /(memory->adjPoints[i-1].temperature-memory->adjPoints[i-2].temperature)
			      +memory->adjPoints[i-2].transitionMatrix[2][2];
							
			//Умножить на матрицу перехода
			inclinometrData.AccelerationX = AccelTmpX*x11+AccelTmpY*x21+AccelTmpZ*x31;
		  inclinometrData.AccelerationY = AccelTmpX*x12+AccelTmpY*x22+AccelTmpZ*x32;		
		  inclinometrData.AccelerationZ = AccelTmpX*x13+AccelTmpY*x23+AccelTmpZ*x33;
		}
  }	
		
	//Рассчитываем тангаж и крен
	if(inclinometrData.AccelerationX>0)//Т
	{
		inclinometrData.Pitch=acosf((inclinometrData.AccelerationX*0.0F + inclinometrData.AccelerationZ*1.0F)
			                           /(sqrtf(inclinometrData.AccelerationX*inclinometrData.AccelerationX + inclinometrData.AccelerationZ*inclinometrData.AccelerationZ)*sqrtf(0.0F*0.0F + -1.0F*-1.0F)))*180.0F/3.14159265F;
	}
	else 
	{
		inclinometrData.Pitch=-acosf((inclinometrData.AccelerationX*0.0F + inclinometrData.AccelerationZ*1.0F)
			                           /(sqrtf(inclinometrData.AccelerationX*inclinometrData.AccelerationX + inclinometrData.AccelerationZ*inclinometrData.AccelerationZ)*sqrtf(0.0F*0.0F + -1.0F*-1.0F)))*180.0F/3.14159265F;
	}
		
	if(inclinometrData.AccelerationY>0)//Т
	{
		inclinometrData.Roll=acosf((inclinometrData.AccelerationY*0.0F + inclinometrData.AccelerationZ*1.0F)
			                           /(sqrtf(inclinometrData.AccelerationY*inclinometrData.AccelerationY + inclinometrData.AccelerationZ*inclinometrData.AccelerationZ)*sqrtf(0.0F*0.0F + -1.0F*-1.0F)))*180.0F/3.14159265F;
	}
	else 
	{
		inclinometrData.Roll=-acosf((inclinometrData.AccelerationY*0.0F + inclinometrData.AccelerationZ*1.0F)
			                           /(sqrtf(inclinometrData.AccelerationY*inclinometrData.AccelerationY + inclinometrData.AccelerationZ*inclinometrData.AccelerationZ)*sqrtf(0.0F*0.0F + -1.0F*-1.0F)))*180.0F/3.14159265F;
	}
}

void adxlTemperatureReceiveCallback(float temperature)
{
	if(inclinometrData.Temperature==-273.0F) inclinometrData.Temperature = temperature;
	else inclinometrData.Temperature = inclinometrData.Temperature*0.9F + temperature*0.1F;//Фильтр Калмана
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	uint32_t* id = (void*)0x1FFFF7E8;
  infoRegisters.len = sprintf((char*)infoRegisters.data,"%s%.8X%.8X%.8X",infoStringFirstPart,id[0],id[1],id[2]);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	Memory_Init();
	
  RS485Connector_Init(&rs485Server);
	
	uint32_t lastBlinkTime = HAL_GetTick();
	uint32_t blinkTime = HAL_GetTick();
	
	if(LSM9DS0_Init(&lsm9ds0)!=HAL_OK)
	{
		while(1)
		{			 
			if(GetTimeSpan(lastBlinkTime, blinkTime = HAL_GetTick())>100)
			{
				lastBlinkTime = blinkTime;
			  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			}
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(GetTimeSpan(lastBlinkTime, blinkTime = HAL_GetTick())>500)
		{
		  lastBlinkTime = blinkTime;
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		}
		
		LSM9DS0_Loop(&lsm9ds0);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

	#ifdef OLD_CODE
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 719;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 365;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
	#else
	htim4.Instance = TIM4;
  htim4.Init.Prescaler = 719;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = (uint16_t)(10.0F*3.5F*100000.0F/memory->uartProperty.uartBaudrate/*huart2.Instance->BRR*/)+1;//365;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  #endif
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
#ifdef OLD_CODE
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
#else 
	huart2.Instance = USART2;
  huart2.Init.BaudRate = memory->uartProperty.uartBaudrate;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
#endif

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_UART2_RTS_GPIO_Port, RS485_UART2_RTS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_UART2_RTS_Pin */
  GPIO_InitStruct.Pin = RS485_UART2_RTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(RS485_UART2_RTS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADXL357_DRDY_Pin */
  GPIO_InitStruct.Pin = ADXL357_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ADXL357_DRDY_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

