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

#if defined(BUILD_LK)
#include <string.h>
#else
#include <linux/string.h>
#endif

#ifndef BUILD_LK
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(320)
#define FRAME_HEIGHT 										(320)
#define LCM_ID       										(0x00)
#define REGFLAG_DELAY             							(0XFE)
#define REGFLAG_END_OF_TABLE      							(0xFF)   // END OF REGISTERS MARKER


#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
#define LCM_DSI_CMD_MODE									1   //modified by zhuqiang for PR750060 20140814

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

typedef struct {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
}LCM_setting_table ;

static LCM_setting_table lcm_initialization_setting[] = {

	/*
	Note :

	Data ID will depends on the following rule.

	count of parameters > 1	=> Data ID = 0x39
	count of parameters = 1	=> Data ID = 0x15
	count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag

	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
#if 0
	{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 150, {}},
	{0xF0,	1,	{0xC3}},
	{0xF0,	1,	{0x96}},
	{0x35,	1,	{0x00}},
	{0xb1,  2,  {0x80,0x10}},
	{0xb5,	4,	{0x08,0x08,0x00,0x08}},//02/02/00/04
	{0x36,	1,	{0x48}},
	{0xE8,	8,	{0x40, 0x8A, 0x00, 0x00, 0x29, 0x1D, 0x25, 0x33}},
	{0xC1,	1,	{0x13}},
	{0xC2,	1,	{0xA7}},	
	{0xC5,	1,	{0x17}},
	{0xE0,	14,	{0xF0, 0x07, 0x11, 0x10, 0x11, 0x0B, 0x3E, 0x32, 0x4C, 0x0B, 0x17, 0x18, 0x1C, 0x1E}},
	{0xE1,	14,	{0xF0, 0x07, 0x11, 0x0F, 0x10, 0x1A, 0x3B, 0x44, 0x4B, 0x0B, 0x16, 0x17, 0x1B, 0x1E}},
	{0xF0,	1,	{0x3C}},
	{0xF0,	1,	{0x69}},
	{0x21,0,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 50, {}},

	//{0x2C,	1,	{0x00}},
	//{REGFLAG_DELAY, 50, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}  
	#else
	//sleep out
    {0x11, 0, {}},
    {REGFLAG_DELAY, 150, {}},
	{0xf0,1,{0xc3}},
	{0xf0,1,{0x96}},	
	{0xb6,3,{0x8a,0x07,0x3b}},  //st engineer add 2016.06.24
//	{0xb9,2,{0x02,0x70}},
   {0x35,1,{0x00}},
	{0x36,1,{0x48}},	
	{0xb4,1,{0x01}},	
	{0xe8,8,{0x40,0x82,0x07,0x18,0x27,0x0a,0xb6,0x33}},	
	{0xc2,1,{0xa7}},	
	{0xc5,1,{0x0b}},	
	{0xe0,14,{0xf0,0x07,0x11,0x14,0x16,0x0c,0x42,0x55,0x50,0x0b,0x16,0x16,0x20,0x23}},	
	{0xe1,14,{0xf0,0x06,0x11,0x13,0x14,0x1c,0x42,0x54,0x51,0x0b,0x16,0x15,0x20,0x22}},	
	{0xf0,1,{0x3c}},	
	{0xf0,1,{0x69}},	
  {REGFLAG_DELAY, 120, {}},
	{0x36,1,{0x48}},
	{0x3a,1,{0x77}},	
	//{0x11,0,{0x00}},	
	{0x29,0,{0x00}},	
	{0x21,0,{0x00}},	
	{0x2c,0,{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}  
	#endif
};


#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static LCM_setting_table lcm_sleep_out_setting[] = {

	{0x11, 1, {0x00}},

	{0x29, 1, {0x00}},
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

	{0x2C, 1, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
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
	// add by zhuqiang for command mode FR507765 at 2013.8.15
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

	//DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_ONE_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	//params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB666;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	//Highly depends on LCD driver capability.
	//params->dsi.packet_size=256;
	//add by zhuqiang for command mode FR507765 at 2013.8.15 begin
	params->dsi.word_count=320*3;	//DSI CMD mode need set these two bellow params, different to 6577
	params->dsi.vertical_active_line=320;
	//add by zhuqiang for command mode FR507765 at 2013.8.15 end

	//Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	//add by zhuqiang for command mode FR507765 at 2013.8.15 begin
	/*
	params->dsi.word_count=FRAME_WIDTH*3;	
	params->dsi.vertical_sync_active=2;
	params->dsi.vertical_backporch=2;
	params->dsi.vertical_frontporch=2;
	params->dsi.vertical_active_line=FRAME_HEIGHT;

	params->dsi.line_byte=2180;		// 2256 = 752*3

	params->dsi.horizontal_sync_active=26;
	params->dsi.horizontal_backporch=206;
	params->dsi.horizontal_frontporch=206;
	params->dsi.horizontal_active_pixel=FRAME_WIDTH;
	*/

	/*
	params->dsi.horizontal_sync_active_byte=26;
	params->dsi.horizontal_backporch_byte=206;
	params->dsi.horizontal_frontporch_byte=206;	
	params->dsi.rgb_byte=(FRAME_WIDTH*3+6);		// NC

	params->dsi.horizontal_sync_active_word_count=20;	
	params->dsi.horizontal_backporch_word_count=200;
	params->dsi.horizontal_frontporch_word_count=200;
	*/

	//add by zhuqiang for command mode FR507765 at 2013.8.15 end   

	//add by zhuqiang for FR507765 at 2013.8.14 begin
	/*
	params->dsi.pll_div1=0;         //  div1=0,1,2,3;  div1_real=1,2,4,4
	params->dsi.pll_div2=2;         //  div2=0,1,2,3;div2_real=1,2,4,4
	params->dsi.fbk_div =0x1C;      //  fref=26MHz,  fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
	*/

	//add by zhuqiang for FR507765 at 2013.8.14 end
	params->dsi.PLL_CLOCK = 90;  //modified by zhuqiang for PR836523 20141111
}

static void lcm_init(void)
{

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	//Remove by ygm due to white display problem. 20121010.     
	//lcm_init();
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(LCM_setting_table), 1);
}

static void lcm_update(unsigned int x, unsigned int y,unsigned int width, unsigned int height)
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
	//add by zhuqiang for command mode FR507765 at 2013.8.15 begin
	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
	// add by zhuqiang for command mode FR507765 at 2013.8.15 end
}

// added by zhuqiang for lcd esd begin 2012.11.19
#if 0
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	unsigned char buffer_vcom[4];
	unsigned char buffer_0a[1];
	unsigned int array[16];
	
	array[0]=0x00341500;
	dsi_set_cmdq(array, 1, 1);

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0A,buffer_0a, 1);

	/*array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x09,buffer_0b, 5);
	*/

	array[0] = 0x00043700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xC5, buffer_vcom, 4);

	array[0]=0x00351500;
	dsi_set_cmdq(array, 1, 1);
	//printk(" yueli lcm 0x0a is %x,lcm 0x0b is %x--------------\n", buffer_0a[0], buffer_0b[1]);
	printk(" yueli lcm 0x0a is %x--------------\n", buffer_0a[0]);
	printk(" yueli lcm 0xc5 is %x,%x,%x,%x--------------\n", buffer_vcom[0], buffer_vcom[1] ,buffer_vcom[2], buffer_vcom[3]);
	if ((buffer_vcom[0]==0x00)&&(buffer_vcom[1]==0x18)&&(buffer_vcom[2]==0x80)&&(buffer_0a[0]==0x9C))
	{
		return 0;
	}
	else
	{
		return 1;
	}
#endif
}

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK
	lcm_init();
	return 1;
#endif 
}
#endif

// added by zhuqiang for lcd esd end 2012.11.19
static unsigned int lcm_compare_id(void)
{
	return 1;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER st7796s_qvga_vdo_lcm_drv=
{
	.name			= "st7796s_dsi_qvga_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif      
	//.esd_check    = lcm_esd_check,
	//.esd_recover  = lcm_esd_recover,
	.compare_id     = lcm_compare_id,
};

