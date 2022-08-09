#ifndef __LSM9DS0_H
#define __LSM9DS0_H
#include "main.h"

#define X_M_ADDRESS         (uint8_t)0x1E
#define G_ADDRESS           (uint8_t)0x6A

////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G			    (uint8_t)0x0F
#define CTRL_REG1_G			    (uint8_t)0x20
#define CTRL_REG2_G			    (uint8_t)0x21
#define CTRL_REG3_G			    (uint8_t)0x22
#define CTRL_REG4_G			    (uint8_t)0x23
#define CTRL_REG5_G			    (uint8_t)0x24
#define REFERENCE_G			    (uint8_t)0x25
#define STATUS_REG_G		    (uint8_t)0x27
#define OUT_X_L_G			      (uint8_t)0x28
#define OUT_X_H_G			      (uint8_t)0x29
#define OUT_Y_L_G			      (uint8_t)0x2A
#define OUT_Y_H_G			      (uint8_t)0x2B
#define OUT_Z_L_G			      (uint8_t)0x2C
#define OUT_Z_H_G			      (uint8_t)0x2D
#define FIFO_CTRL_REG_G		  (uint8_t)0x2E
#define FIFO_SRC_REG_G		  (uint8_t)0x2F
#define INT1_CFG_G			    (uint8_t)0x30
#define INT1_SRC_G			    (uint8_t)0x31
#define INT1_THS_XH_G		    (uint8_t)0x32
#define INT1_THS_XL_G		    (uint8_t)0x33
#define INT1_THS_YH_G		    (uint8_t)0x34
#define INT1_THS_YL_G		    (uint8_t)0x35
#define INT1_THS_ZH_G		    (uint8_t)0x36
#define INT1_THS_ZL_G		    (uint8_t)0x37
#define INT1_DURATION_G		  (uint8_t)0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM		    (uint8_t)0x05
#define OUT_TEMP_H_XM		    (uint8_t)0x06
#define STATUS_REG_M		    (uint8_t)0x07
#define OUT_X_L_M			      (uint8_t)0x08
#define OUT_X_H_M			      (uint8_t)0x09
#define OUT_Y_L_M			      (uint8_t)0x0A
#define OUT_Y_H_M			      (uint8_t)0x0B
#define OUT_Z_L_M			      (uint8_t)0x0C
#define OUT_Z_H_M			      (uint8_t)0x0D
#define WHO_AM_I_XM			    (uint8_t)0x0F
#define INT_CTRL_REG_M		  (uint8_t)0x12
#define INT_SRC_REG_M		    (uint8_t)0x13
#define INT_THS_L_M			    (uint8_t)0x14
#define INT_THS_H_M			    (uint8_t)0x15
#define OFFSET_X_L_M		    (uint8_t)0x16
#define OFFSET_X_H_M		    (uint8_t)0x17
#define OFFSET_Y_L_M		    (uint8_t)0x18
#define OFFSET_Y_H_M		    (uint8_t)0x19
#define OFFSET_Z_L_M		    (uint8_t)0x1A
#define OFFSET_Z_H_M		    (uint8_t)0x1B
#define REFERENCE_X			    (uint8_t)0x1C
#define REFERENCE_Y			    (uint8_t)0x1D
#define REFERENCE_Z			    (uint8_t)0x1E
#define CTRL_REG0_XM		    (uint8_t)0x1F
#define CTRL_REG1_XM		    (uint8_t)0x20
#define CTRL_REG2_XM		    (uint8_t)0x21
#define CTRL_REG3_XM		    (uint8_t)0x22
#define CTRL_REG4_XM		    (uint8_t)0x23
#define CTRL_REG5_XM		    (uint8_t)0x24
#define CTRL_REG6_XM		    (uint8_t)0x25
#define CTRL_REG7_XM		    (uint8_t)0x26
#define STATUS_REG_A		    (uint8_t)0x27
#define OUT_X_L_A			      (uint8_t)0x28
#define OUT_X_H_A			      (uint8_t)0x29
#define OUT_Y_L_A			      (uint8_t)0x2A
#define OUT_Y_H_A			      (uint8_t)0x2B
#define OUT_Z_L_A			      (uint8_t)0x2C
#define OUT_Z_H_A			      (uint8_t)0x2D
#define FIFO_CTRL_REG		    (uint8_t)0x2E
#define FIFO_SRC_REG		    (uint8_t)0x2F
#define INT_GEN_1_REG		    (uint8_t)0x30
#define INT_GEN_1_SRC		    (uint8_t)0x31
#define INT_GEN_1_THS		    (uint8_t)0x32
#define INT_GEN_1_DURATION	(uint8_t)0x33
#define INT_GEN_2_REG		    (uint8_t)0x34
#define INT_GEN_2_SRC		    (uint8_t)0x35
#define INT_GEN_2_THS		    (uint8_t)0x36
#define INT_GEN_2_DURATION	(uint8_t)0x37
#define CLICK_CFG			      (uint8_t)0x38
#define CLICK_SRC			      (uint8_t)0x39
#define CLICK_THS			      (uint8_t)0x3A
#define TIME_LIMIT			    (uint8_t)0x3B
#define TIME_LATENCY		    (uint8_t)0x3C
#define TIME_WINDOW			    (uint8_t)0x3D
#define ACT_THS				      (uint8_t)0x3E
#define ACT_DUR				      (uint8_t)0x3F  
	
// gyro_scale defines the possible full-scale ranges of the gyroscope:
enum gyro_scale{
	G_SCALE_245DPS,		// 00:  245 degrees per second
	G_SCALE_500DPS,		// 01:  500 dps
	G_SCALE_2000DPS,	// 10:  2000 dps
};

// accel_scale defines all possible FSR's of the accelerometer:
enum accel_scale{
	A_SCALE_2G,	      // 000:  2g
	A_SCALE_4G,	      // 001:  4g
	A_SCALE_6G,	      // 010:  6g
	A_SCALE_8G,	      // 011:  8g
	A_SCALE_16G	      // 100:  16g
};
	
// mag_scale defines all possible FSR's of the magnetometer:
enum mag_scale{
	M_SCALE_2GS,	    // 00:  2Gs
	M_SCALE_4GS, 	    // 01:  4Gs
	M_SCALE_8GS,	    // 10:  8Gs
	M_SCALE_12GS,	    // 11:  12Gs
};
		
// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
enum gyro_odr{						// ODR (Hz) --- Cutoff
	G_ODR_95_BW_125  = 0x0, //   95         12.5
	G_ODR_95_BW_25   = 0x1, //   95          25
	// 0x2 and 0x3 define the same data rate and bandwidth
	G_ODR_190_BW_125 = 0x4, //   190        12.5
	G_ODR_190_BW_25  = 0x5, //   190         25
	G_ODR_190_BW_50  = 0x6, //   190         50
	G_ODR_190_BW_70  = 0x7, //   190         70
	G_ODR_380_BW_20  = 0x8, //   380         20
	G_ODR_380_BW_25  = 0x9, //   380         25
	G_ODR_380_BW_50  = 0xA, //   380         50
	G_ODR_380_BW_100 = 0xB, //   380         100
	G_ODR_760_BW_30  = 0xC, //   760         30
	G_ODR_760_BW_35  = 0xD, //   760         35
	G_ODR_760_BW_50  = 0xE, //   760         50
	G_ODR_760_BW_100 = 0xF, //   760         100
};

// accel_oder defines all possible output data rates of the accelerometer:
enum accel_odr{
	A_POWER_DOWN, 	// Power-down mode (0x0)
	A_ODR_3125,		  // 3.125 Hz	(0x1)
	A_ODR_625,		  // 6.25 Hz (0x2)
	A_ODR_125,		  // 12.5 Hz (0x3)
	A_ODR_25,		    // 25 Hz (0x4)
	A_ODR_50,		    // 50 Hz (0x5)
	A_ODR_100,		  // 100 Hz (0x6)
	A_ODR_200,		  // 200 Hz (0x7)
	A_ODR_400,		  // 400 Hz (0x8)
	A_ODR_800,		  // 800 Hz (9)
	A_ODR_1600		  // 1600 Hz (0xA)
};
	
// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
enum accel_abw{
	A_ABW_773,	// 773 Hz (0x0)
	A_ABW_194,	// 194 Hz (0x1)
	A_ABW_362,	// 362 Hz (0x2)
	A_ABW_50,		//  50 Hz (0x3)
};


// mag_oder defines all possible output data rates of the magnetometer:
enum mag_odr{
	M_ODR_3125,	// 3.125 Hz (0x00)
	M_ODR_625,	// 6.25 Hz (0x01)
	M_ODR_125,	// 12.5 Hz (0x02)
	M_ODR_25,	  // 25 Hz (0x03)
	M_ODR_50,	  // 50 (0x04)
	M_ODR_100,	// 100 Hz (0x05)
};                        

typedef enum{
	/*LSM9DS0InitStage1,
	LSM9DS0InitStage2,
	LSM9DS0InitStage3,
	LSM9DS0InitStage4,
	LSM9DS0InitStage5,
	LSM9DS0InitStage6,*/
	LSM9DS0ReadyToRead,
	LSM9DS0ReadNumFifoElem,
	LSM9DS0ReadVectors,
	LSM9DS0ReadTemperature,
	LSM9DS0Error,
}LSM9DS0ReadStage;

typedef struct{
	//---Connection---//
	I2C_HandleTypeDef* i2c;
	HAL_StatusTypeDef status;
	
	void (*rxCpltCallback)(void* adxl357);
	
	//uint8_t address;//0x1D - MISO/ASEL pin low; 0x53 - MISO/ASEL pin high.
	uint8_t devID;
	//uint8_t amplitudeRange; always 10g it is enough
	uint8_t isConnected;
	
	LSM9DS0ReadStage readStage;
	uint8_t temperatureCode[2];
	uint8_t numberOfFIFOElem;
	uint8_t currentFIFOElem;	
  float adxl_Vector[3];	
	
}LSM9DS0;


HAL_StatusTypeDef LSM9DS0_Init(LSM9DS0* lsm9ds0);
void LSM9DS0_Loop(LSM9DS0* lsm9ds0);

void adxlVectorReceiveCallback(float lsm9ds0_Vector[3]);
void adxlTemperatureReceiveCallback(float temperature);

#endif


