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
	float AccelerationX;       //Ускорение по оси X
	float AccelerationY;       //Ускорение по оси Y
	float AccelerationZ;       //Ускорение по оси Z
	float Temperature;         //Температура датчика
	float AccelerationRAWX;    //Ускорение по оси X до выполнения корректировки
	float AccelerationRAWY;    //Ускорение по оси Y до выполнения корректировки
	float AccelerationRAWZ;    //Ускорение по оси Z до выполнения корректировки
	float VibroAcceleratuonRMS;//Ускорение по оси X до выполнения корректировки
}InclinometrData;


#endif

