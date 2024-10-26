/*
 * bmp180.c
 *
 *  Created on: Jan 27, 2024
 *      Author: Kubilay
 */

#include "bmp180.h"


short AC1,AC2,AC3,B1,B2,MB,MC,MD;
unsigned short AC4,AC5,AC6;


void bmp180_read_calibration_data(I2C_HandleTypeDef *I2Cx)
{
	  uint8_t calibration_data[22];
	  HAL_I2C_Mem_Read(I2Cx, 0xEF, 0xAA, 1, calibration_data, 22, 100);

	  AC1 = (calibration_data[0] << 8) | (calibration_data[1]);
	  AC2 = (calibration_data[2] << 8) | (calibration_data[3]);
	  AC3 = (calibration_data[4] << 8) | (calibration_data[5]);
	  AC4 = (calibration_data[6] << 8) | (calibration_data[7]);
	  AC5 = (calibration_data[8] << 8) | (calibration_data[9]);
	  AC6 = (calibration_data[10] << 8) | (calibration_data[11]);
	  B1 = (calibration_data[12] << 8) | (calibration_data[13]);
	  B2 = (calibration_data[14] << 8) | (calibration_data[15]);
	  MB = (calibration_data[16] << 8) | (calibration_data[17]);
	  MC = (calibration_data[18] << 8) | (calibration_data[19]);
	  MD = (calibration_data[20] << 8) | (calibration_data[21]);
}













