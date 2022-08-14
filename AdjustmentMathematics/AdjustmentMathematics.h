#ifndef __ADJ_MATH_H
#define __ADJ_MATH_H
#include "main.h"

#define ADJ_POINTS_COUNT (uint16_t)10

typedef struct{
	float temperature;
	float zeroOffsetVector[3];//Вектор смещения нуля 0-X, 1-Y, 2-Z
	float transitionMatrix[3][3];//Матрица перехода к новому векторному пространству (в ней могут быть учтены регулировочные углы наклона, а так же не ортоганальность системы координат)
}AdjPoint;

typedef struct{
	float Pitch;               //Тангаж
	float Roll;                //Крен
	float AccelerationX;       //Ускорение по оси X [g]
	float AccelerationY;       //Ускорение по оси Y [g]
	float AccelerationZ;       //Ускорение по оси Z [g]
	float Temperature;         //Температура датчика [град]
	float AccelerationRAWX;    //Ускорение по оси X до выполнения корректировки [g]
	float AccelerationRAWY;    //Ускорение по оси Y до выполнения корректировки [g]
	float AccelerationRAWZ;    //Ускорение по оси Z до выполнения корректировки [g]
	float VibroAcceleratuonRMS;//Ускорение по оси X до выполнения корректировки [g]
	
	//Данные магнитометра
	float MagnetometerRAWX;    //Напряженность магнитного поля по оси X до выполнения корректировки [гаусс]
	float MagnetometerRAWY;    //Напряженность магнитного поля по оси Y до выполнения корректировки [гаусс]
	float MagnetometerRAWZ;    //Напряженность магнитного поля по оси Z до выполнения корректировки [гаусс]
	
	float MagnetometerX;    //Напряженность магнитного поля по оси X [гаусс]
	float MagnetometerY;    //Напряженность магнитного поля по оси Y [гаусс]
	float MagnetometerZ;    //Напряженность магнитного поля по оси Z [гаусс]
	
	float MagnetometerRMS;    //Ускорение по оси X до выполнения корректировки [гаусс]
}InclinometrData;


#endif

