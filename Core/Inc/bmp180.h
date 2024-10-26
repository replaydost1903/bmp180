/*
 * bmp180.h
 *
 *  Created on: Dec 30, 2023
 *      Author: Kubilay
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "main.h"


/*BMP180 Device Definitions*/
#define DEVICE_ADDRESS			(0x77)
#define DEVICE_READ_ADDRESS		(0xEF)
#define DEVICE_WRITE_ADDRESS	(0xEE)


/*BMP180 Register Definition*/
#define OUT_XLSB				(0xF8)
#define OUT_LSB					(0xF7)
#define OUT_MSB					(0xF6)
#define CTRL_MEAS				(0xF4)
#define SOFT_RESET				(0xE0)
#define ID						(0xD0)



/*BMP180 APIs*/
void bmp180_read_calibration_data(I2C_HandleTypeDef *I2Cx);


















#endif /* INC_BMP180_H_ */
