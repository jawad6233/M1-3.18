/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#define REGFLAG_DELAY                                       0XFFE
#define REGFLAG_END_OF_TABLE                                0xFFF   // END OF REGISTERS MARKER
#define LCM_ID_NT35596 (0x96)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#define   LCM_DSI_CMD_MODE							0

typedef struct {
     unsigned cmd;
     unsigned char count;
     unsigned char para_list[64];
}LCM_setting_table ;

static LCM_setting_table lcm_initialization_setting[] =
{
	//TURN ON NT50198
{0xFF,1,{0x05}},
{0xFB,1,{0x01}},
{0xC5,1,{0x01}},
{REGFLAG_DELAY, 120, {}}, 
//6.5AUO+NT35596
{0XFF,  1,  {0X01}},
{0XFB,  1,  {0X01}},
{0X00,  1,  {0X01}},
{0X01,  1,  {0X55}},
{0X02,  1,  {0X40}},
{0X05,  1,  {0X40}},
{0X06,  1,  {0X19}},
{0X07,  1,  {0X1E}},
{0X08,  1,  {0X0C}},
{0X0B,  1,  {0X7D}},
{0X0C,  1,  {0X7D}},
{0X0E,  1,  {0XB5}},
{0X0F,  1,  {0XB3}},
{0X11,  1,  {0X17}},//27
{0X12,  1,  {0X17}},
{0X13,  1,  {0X01}},
{0X14,  1,  {0X4A}},
{0X15,  1,  {0X19}},
{0X16,  1,  {0X19}},
{0X18,  1,  {0X00}},
{0X19,  1,  {0X77}},
{0X1A,  1,  {0X55}},
{0X1B,  1,  {0X13}},
{0X1C,  1,  {0X00}},
{0X1D,  1,  {0X00}},
{0X1E,  1,  {0X13}},
{0X1F,  1,  {0X00}},
{0X23,  1,  {0X12}},
{0X24,  1,  {0X70}},
{0X25,  1,  {0X61}},
{0X26,  1,  {0X56}},
{0X27,  1,  {0X1B}},
{0X28,  1,  {0X01}},
{0X35,  1,  {0X00}},
{0X66,  1,  {0X00}},
{0X58,  1,  {0X82}},
{0X59,  1,  {0X02}},
{0X5A,  1,  {0X02}},
{0X5B,  1,  {0X02}},
{0X5C,  1,  {0X82}},
{0X5D,  1,  {0X82}},
{0X5E,  1,  {0X02}},
{0X5F,  1,  {0X02}},
{0X6D,  1,  {0X22}},
{0X72,  1,  {0X31}},

//gamma setting part 1
{0x75,  1,  {0x00}},
{0x76,  1,  {0x19}},
{0x77,  1,  {0x00}},
{0x78,  1,  {0x36}},
{0x79,  1,  {0x00}},
{0x7A,  1,  {0x5E}},
{0x7B,  1,  {0x00}},
{0x7C,  1,  {0x7C}},
{0x7D,  1,  {0x00}},
{0x7E,  1,  {0x94}},
{0x7F,  1,  {0x00}},
{0x80,  1,  {0xA9}},
{0x81,  1,  {0x00}},
{0x82,  1,  {0xBC}},
{0x83,  1,  {0x00}},
{0x84,  1,  {0xCC}},
{0x85,  1,  {0x00}},
{0x86,  1,  {0xDB}},
{0x87,  1,  {0x01}},
{0x88,  1,  {0x0E}},
{0x89,  1,  {0x01}},
{0x8A,  1,  {0x35}},
{0x8B,  1,  {0x01}},
{0x8C,  1,  {0x73}},
{0x8D,  1,  {0x01}},
{0x8E,  1,  {0xA3}},
{0x8F,  1,  {0x01}},
{0x90,  1,  {0xED}},
{0x91,  1,  {0x02}},
{0x92,  1,  {0x25}},
{0x93,  1,  {0x02}},
{0x94,  1,  {0x26}},
{0x95,  1,  {0x02}},
{0x96,  1,  {0x59}},
{0x97,  1,  {0x02}},
{0x98,  1,  {0x8F}},
{0x99,  1,  {0x02}},
{0x9A,  1,  {0xB2}},
{0x9B,  1,  {0x02}},
{0x9C,  1,  {0xE6}},
{0x9D,  1,  {0x03}},
{0x9E,  1,  {0x0D}},
{0x9F,  1,  {0x03}},
{0xA0,  1,  {0x4F}},
{0xA2,  1,  {0x03}},
{0xA3,  1,  {0x6F}},
{0xA4,  1,  {0x03}},
{0xA5,  1,  {0xC1}},
{0xA6,  1,  {0x03}},
{0xA7,  1,  {0xCC}},
{0xA9,  1,  {0x03}},
{0xAA,  1,  {0xCB}},
{0xAB,  1,  {0x03}},
{0xAC,  1,  {0xCB}},
{0xAD,  1,  {0x03}},
{0xAE,  1,  {0xCB}},
{0xAF,  1,  {0x03}},
{0xB0,  1,  {0xCB}},
{0xB1,  1,  {0x03}},
{0xB2,  1,  {0xCB}},
{0xB3,  1,  {0x00}},
{0xB4,  1,  {0x19}},
{0xB5,  1,  {0x00}},
{0xB6,  1,  {0x36}},
{0xB7,  1,  {0x00}},
{0xB8,  1,  {0x5E}},
{0xB9,  1,  {0x00}},
{0xBA,  1,  {0x7C}},
{0xBB,  1,  {0x00}},
{0xBC,  1,  {0x94}},
{0xBD,  1,  {0x00}},
{0xBE,  1,  {0xA9}},
{0xBF,  1,  {0x00}},
{0xC0,  1,  {0xBC}},
{0xC1,  1,  {0x00}},
{0xC2,  1,  {0xCC}},
{0xC3,  1,  {0x00}},
{0xC4,  1,  {0xDB}},
{0xC5,  1,  {0x01}},
{0xC6,  1,  {0x0E}},
{0xC7,  1,  {0x01}},
{0xC8,  1,  {0x35}},
{0xC9,  1,  {0x01}},
{0xCA,  1,  {0x73}},
{0xCB,  1,  {0x01}},
{0xCC,  1,  {0xA3}},
{0xCD,  1,  {0x01}},
{0xCE,  1,  {0xED}},
{0xCF,  1,  {0x02}},
{0xD0,  1,  {0x25}},
{0xD1,  1,  {0x02}},
{0xD2,  1,  {0x26}},
{0xD3,  1,  {0x02}},
{0xD4,  1,  {0x59}},
{0xD5,  1,  {0x02}},
{0xD6,  1,  {0x8F}},
{0xD7,  1,  {0x02}},
{0xD8,  1,  {0xB2}},
{0xD9,  1,  {0x02}},
{0xDA,  1,  {0xE6}},
{0xDB,  1,  {0x03}},
{0xDC,  1,  {0x0D}},
{0xDD,  1,  {0x03}},
{0xDE,  1,  {0x4F}},
{0xDF,  1,  {0x03}},
{0xE0,  1,  {0x6F}},
{0xE1,  1,  {0x03}},
{0xE2,  1,  {0xC1}},
{0xE3,  1,  {0x03}},
{0xE4,  1,  {0xCC}},
{0xE5,  1,  {0x03}},
{0xE6,  1,  {0xCB}},
{0xE7,  1,  {0x03}},
{0xE8,  1,  {0xCB}},
{0xE9,  1,  {0x03}},
{0xEA,  1,  {0xCB}},
{0xEB,  1,  {0x03}},
{0xEC,  1,  {0xCB}},
{0xED,  1,  {0x03}},
{0xEE,  1,  {0xCB}},
{0xEF,  1,  {0x00}},
{0xF0,  1,  {0x19}},
{0xF1,  1,  {0x00}},
{0xF2,  1,  {0x36}},
{0xF3,  1,  {0x00}},
{0xF4,  1,  {0x5E}},
{0xF5,  1,  {0x00}},
{0xF6,  1,  {0x7C}},
{0xF7,  1,  {0x00}},
{0xF8,  1,  {0x94}},
{0xF9,  1,  {0x00}},
{0xFA,  1,  {0xA9}},

{0xFF,  1,  {0x02}},
{0xFB,  1,  {0x01}},

//gamma setting part 2
{0x00,  1,  {0x00}},
{0x01,  1,  {0xBC}},
{0x02,  1,  {0x00}},
{0x03,  1,  {0xCC}},
{0x04,  1,  {0x00}},
{0x05,  1,  {0xDB}},
{0x06,  1,  {0x01}},
{0x07,  1,  {0x0E}},
{0x08,  1,  {0x01}},
{0x09,  1,  {0x35}},
{0x0A,  1,  {0x01}},
{0x0B,  1,  {0x73}},
{0x0C,  1,  {0x01}},
{0x0D,  1,  {0xA3}},
{0x0E,  1,  {0x01}},
{0x0F,  1,  {0xED}},
{0x10,  1,  {0x02}},
{0x11,  1,  {0x25}},
{0x12,  1,  {0x02}},
{0x13,  1,  {0x26}},
{0x14,  1,  {0x02}},
{0x15,  1,  {0x59}},
{0x16,  1,  {0x02}},
{0x17,  1,  {0x8F}},
{0x18,  1,  {0x02}},
{0x19,  1,  {0xB2}},
{0x1A,  1,  {0x02}},
{0x1B,  1,  {0xE6}},
{0x1C,  1,  {0x03}},
{0x1D,  1,  {0x0D}},
{0x1E,  1,  {0x03}},
{0x1F,  1,  {0x4F}},
{0x20,  1,  {0x03}},
{0x21,  1,  {0x6F}},
{0x22,  1,  {0x03}},
{0x23,  1,  {0xC1}},
{0x24,  1,  {0x03}},
{0x25,  1,  {0xCC}},
{0x26,  1,  {0x03}},
{0x27,  1,  {0xCB}},
{0x28,  1,  {0x03}},
{0x29,  1,  {0xCB}},
{0x2A,  1,  {0x03}},
{0x2B,  1,  {0xCB}},
{0x2D,  1,  {0x03}},
{0x2F,  1,  {0xCB}},
{0x30,  1,  {0x03}},
{0x31,  1,  {0xCB}},
{0x32,  1,  {0x00}},
{0x33,  1,  {0x19}},
{0x34,  1,  {0x00}},
{0x35,  1,  {0x36}},
{0x36,  1,  {0x00}},
{0x37,  1,  {0x5E}},
{0x38,  1,  {0x00}},
{0x39,  1,  {0x7C}},
{0x3A,  1,  {0x00}},
{0x3B,  1,  {0x94}},
{0x3D,  1,  {0x00}},
{0x3F,  1,  {0xA9}},
{0x40,  1,  {0x00}},
{0x41,  1,  {0xBC}},
{0x42,  1,  {0x00}},
{0x43,  1,  {0xCC}},
{0x44,  1,  {0x00}},
{0x45,  1,  {0xDB}},
{0x46,  1,  {0x01}},
{0x47,  1,  {0x0E}},
{0x48,  1,  {0x01}},
{0x49,  1,  {0x35}},
{0x4A,  1,  {0x01}},
{0x4B,  1,  {0x73}},
{0x4C,  1,  {0x01}},
{0x4D,  1,  {0xA3}},
{0x4E,  1,  {0x01}},
{0x4F,  1,  {0xED}},
{0x50,  1,  {0x02}},
{0x51,  1,  {0x25}},
{0x52,  1,  {0x02}},
{0x53,  1,  {0x26}},
{0x54,  1,  {0x02}},
{0x55,  1,  {0x59}},
{0x56,  1,  {0x02}},
{0x58,  1,  {0x8F}},
{0x59,  1,  {0x02}},
{0x5A,  1,  {0xB2}},
{0x5B,  1,  {0x02}},
{0x5C,  1,  {0xE6}},
{0x5D,  1,  {0x03}},
{0x5E,  1,  {0x0D}},
{0x5F,  1,  {0x03}},
{0x60,  1,  {0x4F}},
{0x61,  1,  {0x03}},
{0x62,  1,  {0x6F}},
{0x63,  1,  {0x03}},
{0x64,  1,  {0xC1}},
{0x65,  1,  {0x03}},
{0x66,  1,  {0xCC}},
{0x67,  1,  {0x03}},
{0x68,  1,  {0xCB}},
{0x69,  1,  {0x03}},
{0x6A,  1,  {0xCB}},
{0x6B,  1,  {0x03}},
{0x6C,  1,  {0xCB}},
{0x6D,  1,  {0x03}},
{0x6E,  1,  {0xCB}},
{0x6F,  1,  {0x03}},
{0x70,  1,  {0xCB}},
{0x71,  1,  {0x00}},
{0x72,  1,  {0x19}},
{0x73,  1,  {0x00}},
{0x74,  1,  {0x36}},
{0x75,  1,  {0x00}},
{0x76,  1,  {0x5E}},
{0x77,  1,  {0x00}},
{0x78,  1,  {0x7C}},
{0x79,  1,  {0x00}},
{0x7A,  1,  {0x94}},
{0x7B,  1,  {0x00}},
{0x7C,  1,  {0xA9}},
{0x7D,  1,  {0x00}},
{0x7E,  1,  {0xBC}},
{0x7F,  1,  {0x00}},
{0x80,  1,  {0xCC}},
{0x81,  1,  {0x00}},
{0x82,  1,  {0xDB}},
{0x83,  1,  {0x01}},
{0x84,  1,  {0x0E}},
{0x85,  1,  {0x01}},
{0x86,  1,  {0x35}},
{0x87,  1,  {0x01}},
{0x88,  1,  {0x73}},
{0x89,  1,  {0x01}},
{0x8A,  1,  {0xA3}},
{0x8B,  1,  {0x01}},
{0x8C,  1,  {0xED}},
{0x8D,  1,  {0x02}},
{0x8E,  1,  {0x25}},
{0x8F,  1,  {0x02}},
{0x90,  1,  {0x26}},
{0x91,  1,  {0x02}},
{0x92,  1,  {0x59}},
{0x93,  1,  {0x02}},
{0x94,  1,  {0x8F}},
{0x95,  1,  {0x02}},
{0x96,  1,  {0xB2}},
{0x97,  1,  {0x02}},
{0x98,  1,  {0xE6}},
{0x99,  1,  {0x03}},
{0x9A,  1,  {0x0D}},
{0x9B,  1,  {0x03}},
{0x9C,  1,  {0x4F}},
{0x9D,  1,  {0x03}},
{0x9E,  1,  {0x6F}},
{0x9F,  1,  {0x03}},
{0xA0,  1,  {0xC1}},
{0xA2,  1,  {0x03}},
{0xA3,  1,  {0xCC}},
{0xA4,  1,  {0x03}},
{0xA5,  1,  {0xCB}},
{0xA6,  1,  {0x03}},
{0xA7,  1,  {0xCB}},
{0xA9,  1,  {0x03}},
{0xAA,  1,  {0xCB}},
{0xAB,  1,  {0x03}},
{0xAC,  1,  {0xCB}},
{0xAD,  1,  {0x03}},
{0xAE,  1,  {0xCB}},
{0xAF,  1,  {0x00}},
{0xB0,  1,  {0x19}},
{0xB1,  1,  {0x00}},
{0xB2,  1,  {0x36}},
{0xB3,  1,  {0x00}},
{0xB4,  1,  {0x5E}},
{0xB5,  1,  {0x00}},
{0xB6,  1,  {0x7C}},
{0xB7,  1,  {0x00}},
{0xB8,  1,  {0x94}},
{0xB9,  1,  {0x00}},
{0xBA,  1,  {0xA9}},
{0xBB,  1,  {0x00}},
{0xBC,  1,  {0xBC}},
{0xBD,  1,  {0x00}},
{0xBE,  1,  {0xCC}},
{0xBF,  1,  {0x00}},
{0xC0,  1,  {0xDB}},
{0xC1,  1,  {0x01}},
{0xC2,  1,  {0x0E}},
{0xC3,  1,  {0x01}},
{0xC4,  1,  {0x35}},
{0xC5,  1,  {0x01}},
{0xC6,  1,  {0x73}},
{0xC7,  1,  {0x01}},
{0xC8,  1,  {0xA3}},
{0xC9,  1,  {0x01}},
{0xCA,  1,  {0xED}},
{0xCB,  1,  {0x02}},
{0xCC,  1,  {0x25}},
{0xCD,  1,  {0x02}},
{0xCE,  1,  {0x26}},
{0xCF,  1,  {0x02}},
{0xD0,  1,  {0x59}},
{0xD1,  1,  {0x02}},
{0xD2,  1,  {0x8F}},
{0xD3,  1,  {0x02}},
{0xD4,  1,  {0xB2}},
{0xD5,  1,  {0x02}},
{0xD6,  1,  {0xE6}},
{0xD7,  1,  {0x03}},
{0xD8,  1,  {0x0D}},
{0xD9,  1,  {0x03}},
{0xDA,  1,  {0x4F}},
{0xDB,  1,  {0x03}},
{0xDC,  1,  {0x6F}},
{0xDD,  1,  {0x03}},
{0xDE,  1,  {0xC1}},
{0xDF,  1,  {0x03}},
{0xE0,  1,  {0xCC}},
{0xE1,  1,  {0x03}},
{0xE2,  1,  {0xCB}},
{0xE3,  1,  {0x03}},
{0xE4,  1,  {0xCB}},
{0xE5,  1,  {0x03}},
{0xE6,  1,  {0xCB}},
{0xE7,  1,  {0x03}},
{0xE8,  1,  {0xCB}},
{0xE9,  1,  {0x03}},
{0xEA,  1,  {0xCB}},


{0xFF,  1,  {0x05}},
{0xFB,  1,  {0x01}},
{0x00,  1,  {0x01}},
{0x01,  1,  {0x0B}},
{0x02,  1,  {0x0C}},
{0x03,  1,  {0x09}},
{0x04,  1,  {0x0A}},
{0x05,  1,  {0x00}},
{0x06,  1,  {0x0F}},
{0x07,  1,  {0x10}},
{0x08,  1,  {0x00}},
{0x09,  1,  {0x00}},
{0x0A,  1,  {0x00}},
{0x0B,  1,  {0x00}},
{0x0C,  1,  {0x00}},
{0x0D,  1,  {0x13}},
{0x0E,  1,  {0x15}},
{0x0F,  1,  {0x17}},
{0x10,  1,  {0x01}},
{0x11,  1,  {0x0B}},
{0x12,  1,  {0x0C}},
{0x13,  1,  {0x09}},
{0x14,  1,  {0x0A}},
{0x15,  1,  {0x00}},
{0x16,  1,  {0x0F}},
{0x17,  1,  {0x10}},
{0x18,  1,  {0x00}},
{0x19,  1,  {0x00}},
{0x1A,  1,  {0x00}},
{0x1B,  1,  {0x00}},
{0x1C,  1,  {0x00}},
{0x1D,  1,  {0x13}},
{0x1E,  1,  {0x15}},
{0x1F,  1,  {0x17}},
{0x20,  1,  {0x00}},
{0x21,  1,  {0x03}},
{0x22,  1,  {0x01}},
{0x23,  1,  {0x40}},
{0x24,  1,  {0x40}},
{0x25,  1,  {0xED}},
{0x29,  1,  {0x58}},
{0x2A,  1,  {0x12}},
{0x2B,  1,  {0x01}},
{0x4B,  1,  {0x06}},
{0x4C,  1,  {0x11}},
{0x4D,  1,  {0x20}},
{0x4E,  1,  {0x02}},
{0x4F,  1,  {0x02}},
{0x50,  1,  {0x20}},
{0x51,  1,  {0x61}},
{0x52,  1,  {0x01}},
{0x53,  1,  {0x63}},
{0x54,  1,  {0x77}},
{0x55,  1,  {0xED}},
{0x5B,  1,  {0x00}},
{0x5C,  1,  {0x00}},
{0x5D,  1,  {0x00}},
{0x5E,  1,  {0x00}},
{0x5F,  1,  {0x15}},
{0x60,  1,  {0x75}},
{0x61,  1,  {0x00}},
{0x62,  1,  {0x00}},
{0x63,  1,  {0x00}},
{0x64,  1,  {0x00}},
{0x65,  1,  {0x00}},
{0x66,  1,  {0x00}},
{0x67,  1,  {0x00}},
{0x68,  1,  {0x04}},
{0x69,  1,  {0x00}},
{0x6A,  1,  {0x00}},
{0x6C,  1,  {0x40}},
{0x75,  1,  {0x01}},
{0x76,  1,  {0x01}},
{0x7A,  1,  {0x80}},
{0x7B,  1,  {0xA3}},
{0x7C,  1,  {0xD8}},
{0x7D,  1,  {0x60}},
{0x7F,  1,  {0x15}},
{0x81,  1,  {0x0C}},
{0x8A,  1,  {0x00}},
{0x93,  1,  {0x06}},
{0x94,  1,  {0x08}},
{0x9B,  1,  {0x0F}},
{0xFB,  1,  {0x01}},
{0xEA,  1,  {0xFF}},
{0xEC,  1,  {0x00}},


{0xFF,  1,  {0x00}},
{0x35,  1,  {0x01}},
{0xD3,  1,  {0x08}},
{0xD4,  1,  {0x06}},
{0x11, 0, {0x00}},           
{REGFLAG_DELAY, 120, {}}, 
                          
// Display ON             
{0x29, 0, {0x00}},        
{REGFLAG_DELAY, 60, {}}, 

{REGFLAG_END_OF_TABLE, 0x00, {}}

};


static LCM_setting_table lcm_sleep_in_setting[] =
{
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Sleep Mode On
    {0x10, 0, {0x00}},
     {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
				
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }

}

static void init_lcm_registers(void)
{
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(LCM_setting_table), 1);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		//params->dsi.mode   = SYNC_PULSE_VDO_MODE;
		params->dsi.mode   = BURST_VDO_MODE;
//		params->dsi.mode   = SYNC_EVENT_VDO_MODE; 
		
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
    	params->dsi.LANE_NUM     = LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		//params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		//params->dsi.word_count=720*3;	

		
		params->dsi.vertical_sync_active				= 2;    
		params->dsi.vertical_backporch					= 4;// 
		params->dsi.vertical_frontporch					= 4; // 4//20
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				=10;//= 1
		params->dsi.horizontal_backporch				=118;//= 30
		params->dsi.horizontal_frontporch				=118;//= 30
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		//params->dsi.pll_select=0;	//0: MIPI_PLL; 1: LVDS_PLL

		//params->dsi.compatibility_for_nvk=1;
        //	params->dsi.HS_PRPR=6;
//	    params->dsi.LPX=8; 
		//params->dsi.HS_PRPR=5;
		//params->dsi.HS_TRAIL=13;
//		params->dsi.CLK_TRAIL = 10;
		// Bit rate calculation
		
		params->dsi.PLL_CLOCK = 420;
		
		params->dsi.noncont_clock = TRUE;
		params->dsi.noncont_clock_period = 2;

		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;

		params->dsi.lcm_esd_check_table[0].cmd = 0x0a; //0xac; //0x0A;
		params->dsi.lcm_esd_check_table[0].count = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c; //0x00; //0x9C;
	
}

static void lcm_init(void)
{

	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(60);
	
	SET_RESET_PIN(1);
	MDELAY(150);      

	init_lcm_registers();

}


static void lcm_suspend(void)
{
    push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(LCM_setting_table), 1);
   SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(60);

	SET_RESET_PIN(1);
	MDELAY(250); 	

}


static void lcm_resume(void)
{
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(60);

	SET_RESET_PIN(1);
	MDELAY(250); 	 

	init_lcm_registers();
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(&data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
		return 1;
#if 0	
	unsigned char lcd_id = 0;
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(100);

	array[0] = 0x00023700;	// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0];		//we only need ID
#ifdef BUILD_LK
	printf("%s, LK 6_5_nt35596 debug: nt35596 0x%08x\n", __func__,
	       id);
#else
	printk("%s,kernel 6_5_nt35596 horse debug: nt35596 id = 0x%08x\n",
	       __func__, id);
#endif

	if ((id == LCM_ID_NT35596) && lcd_id == 1)
		return 1;
	else
		return 0;	
#endif
}

#if 0
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
//#if 1
	char  buffer[3];
	int   array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	printk("jacob test lcm_esd_check buffer[0]=0x%x\n",buffer[0]);
	if(buffer[0]==0x9c)
	{
		return FALSE;
	}
	else
	{	
		return TRUE;
	}
#else
	return FALSE;
#endif
}

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK	
	printk("jacob test lcm_esd_recover\n");
	lcm_init();
#endif
	return TRUE;
}
#endif

LCM_DRIVER nt35596_fhd_dsi_vdo_lcm_drv = 
{
    .name			= "nt35596_fhd_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,		
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
