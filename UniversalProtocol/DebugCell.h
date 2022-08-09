#ifndef __DEBUG_CELL_H
#define __DEBUG_CELL_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
unsigned short processUPDebugRequestDataCallback(unsigned char* packetData,unsigned short packetDataLenght, unsigned char* outBufferData);

void preReadDebugCellCallback(unsigned short debugCellNumber);
void writeDebugCellCallback(unsigned short debugCellNumber);


#ifdef __cplusplus
 }
#endif
#endif

