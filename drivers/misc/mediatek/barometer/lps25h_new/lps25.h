/*
* include/linux/input/lps25.h
*
* STMicroelectronics LPS25 Pressure / Temperature Sensor module driver
*
* Copyright (C) 2014 STMicroelectronics- HESA BU - Environmental Sensor Application
* (lorenzo.sarchi@st.com)
* This work is based n the previous job from Matteo Dameno and Carmine Iascone
*
*
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or 
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, see <http://www.gnu.org/licenses> or
* write to the Free Software * Foundation, Inc., 51 Franklin St, Fifth Floor, 
* Boston, MA 02110-1301 USA
*
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*
*/
/******************************************************************************
 Revision 0.0.2 2014/Apr/04:
	second release
******************************************************************************/

#ifndef	__LPS25_H__
#define	__LPS25_H__

#define LPS25_PRS_MIN_POLL_PERIOD_MS	1

#define	SAD0L				0x00
#define	SAD0H				0x01
#define	LPS25_PRS_I2C_SADROOT	0x2E
#define	LPS25_PRS_I2C_SAD_L		((LPS25_PRS_I2C_SADROOT<<1)|SAD0L)
#define	LPS25_PRS_I2C_SAD_H		((LPS25_PRS_I2C_SADROOT<<1)|SAD0H)
#define	LPS25_PRS_DEV_NAME		"lps25h"

/* input define mappings */
#define ABS_PR		ABS_PRESSURE
#define ABS_TEMP	ABS_GAS
#define ABS_DLTPR	ABS_MISC

/* Barometer and Termometer output data rate ODR */
#define	LPS25_PRS_ODR_ONESH	0x00	/* one shot both		*/
#define	LPS25_PRS_ODR_1_1	0x10	/*  1  Hz baro,  1  Hz term ODR	*/
#define	LPS25_PRS_ODR_7_7	0x50	/*  7  Hz baro,  7  Hz term ODR	*/
#define	LPS25_PRS_ODR_12_12	0x60	/* 12.5Hz baro, 12.5Hz term ODR	*/
#define	LPS25_PRS_ODR_25_25	0x70	/* 25  Hz baro, 25  Hz term ODR	*/

/*	Pressure section defines		*/
/*	Pressure Sensor Operating Mode		*/
#define	LPS25_PRS_ENABLE		0x01
#define	LPS25_PRS_DISABLE		0x00

/*	Output conversion factors		*/
#define	SENSITIVITY_T		480	/* =	480 LSB/degrC	*/
#define	SENSITIVITY_P		4096	/* =	LSB/mbar	*/
#define	SENSITIVITY_P_SHIFT	12	/* =	4096 LSB/mbar	*/
#define	TEMPERATURE_OFFSET	42.5f	/* =	42.5 degrC	*/

#ifdef __KERNEL__
struct lps25_prs_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;
};

#endif /* __KERNEL__ */

#endif  /* __LPS25_H__ */
