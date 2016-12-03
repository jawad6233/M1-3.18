/* LPS25H Pressure Sensor Driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef LPS25H_BARO_H
#define LPS25H_BARO_H

#include <linux/ioctl.h>
/*****************************************************
|  sensor    |   chip id  |     7-bit i2c address    |
|----------------------------------------------------|
|  lps25h/b  |    0xBD    |           0x5C           |
*****************************************************/

/* apply low pass filter on output */
#define CONFIG_LPS25H_LOWPASS
/*#define CONFIG_ID_TEMPERATURE*/

#define LPS25H_DRIVER_VERSION	"V1.2"

#define LPS25H_DEV_NAME		"lps25h"

#define C_MAX_FIR_LENGTH	(32)
#define MAX_SENSOR_NAME		(32)
#define LPS25H_DATA_NUM		(1)
#define LPS25H_PRESSURE		(0)
#define LPS25H_BUFSIZE		(128)

#define LPS25H_WHO_AM_I		0x0F

/*********************************[LPS25H]*************************************/
/* chip id */
#define LPS25H_CHIP_ID		0xBD

/* i2c address */
#define LPS25H_I2C_ADDRESS	0x5C // 0x5C: SA0 is Low, 0x5D: SA0 is High

#endif/* LPS25H_BARO_H */
