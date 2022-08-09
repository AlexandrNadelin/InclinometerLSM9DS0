#include "DebugCellFormat.h"

unsigned short GetDebugCellLenght(unsigned char  format)
{
	if(format>0 && format<=DEBUG_CELL_FORMAT_UINT64)return format;
	else if(format == DEBUG_CELL_FORMAT_Float)return 4;
	else if(format == DEBUG_CELL_FORMAT_Double)return 8;
	else if(format == DEBUG_CELL_FORMAT_ByteStream)return 0;
	else if(format == DEBUG_CELL_FORMAT_StringStream)return 0;
	else if(format>=DEBUG_CELL_FORMAT_FixByteArray_len_1 && format<=DEBUG_CELL_FORMAT_FixByteArray_len_31)return format&0x1F;
	else if(format>=DEBUG_CELL_FORMAT_FixStringArray_len_1 && format<=DEBUG_CELL_FORMAT_FixStringArray_len_31)return format&0x1F;
	else if(format>=DEBUG_CELL_FORMAT_INT8 && format<=DEBUG_CELL_FORMAT_INT64)return format - DEBUG_CELL_FORMAT_INT8 + 1;
	else if(format>=DEBUG_CELL_FORMAT_FixByteArray256_LOW && format<=DEBUG_CELL_FORMAT_FixByteArray256_HIGH)
	{
		unsigned short n = (format>>2)&0x07;
		unsigned short k = format&0x03;//(k+4)*2^(n+6)
		unsigned short res =2;
    for(int i =1;i<n+6;i++)
    {
    	res*=2;
    }
    return res*(k+4);
	}
	else if(format>=DEBUG_CELL_FORMAT_VarByteArray_len_0 && format<=DEBUG_CELL_FORMAT_VarByteArray_len_31)return format&0x1F;
	else if(format>=DEBUG_CELL_FORMAT_VarStringArray_len_0 && format<=DEBUG_CELL_FORMAT_VarStringArray_len_31)return format&0x1F;
	else if(format>=DEBUG_CELL_FORMAT_FixByteArray8_LOW /*&& format<=DEBUG_CELL_FORMAT_FixByteArray8_HIGH*/)return (format&0x1F)*8;

	return 0;
}

