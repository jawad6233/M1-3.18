#ifdef BUILD_LK
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#endif

#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define LCM_ID (0x8394)
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#ifdef BUILD_LK
int global_lcd_id_hx8394 = 0;
#endif

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
//#define dsi_set_cmdq_V2_TIN(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2_TIN(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
       

typedef struct {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[128];
}LCM_setting_table;


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

    //sleep out
    	{0x11, 0, {}},

    	{REGFLAG_DELAY, 220, {}},

    //SET PASSWORD
       	{0xb9,3,{0xff,0x83,0x94}},


    //SET POWER
	{0xb1,15,{0x7c,0x00,0x24,0x06,0x01,0x11,0x11,0x26,0x2e,0x1d,
		  0x1d,0x57,0x12,0x01,0xe6}},

    // Invoff    Exit inversion mode
	{0x20,0,{}},	
    
    // Madctl    Set address mode
    	{0x36,1,{0x00}},

    // Colmod  Set pixel format
	{0x3a,1,{0x70}},

     //Set Cyc Set display waveform cycle
	{0xb4,18,{0x00,0x00,0x00,0x05,0x08,0x05,0x53,0x04,0x05,0x53,
	          0x23,0x27,0x19,0x5c,0x6c,0x08,0x05,0x04}},

     // Set Gip  Set gate in panel 
	{0xd5, 24,{0x00,0x00,0x00,0x01,0xcd,0x23,0xef,0x45,0x67,0x89,
	           0xab,0xcc,0xcc,0xdc,0x10,0xfe,0x32,0xba,0x98,0x76,
		   0x54,0xcc,0xcc,0xc0}},

    // Set DGC Set Digital Gamma Correction setting
	{0xc1,127,{0x01,0x00,0x06,0x0e,0x16,0x1e,0x26,0x2e,0x36,0x3e,
		   0x46,0x4e,0x56,0x5e,0x66,0x6e,0x76,0x7e,0x86,0x8e,
		   0x96,0x9e,0xa6,0xae,0xb6,0xbe,0xc6,0xce,0xd6,0xde,
		   0xe6,0xee,0xf6,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,
		   0x00,0x00,0x00,0x00,0x03,0x0b,0x13,0x1b,0x23,0x2b,
/* line 6 */	   0x33,0x3b,0x43,0x4b,0x53,0x5b,0x63,0x6b,0x73,0x7b,
		   0x83,0x8b,0x93,0x9b,0xa3,0xab,0xb3,0xbb,0xc3,0xcb,
		   0xd3,0xdb,0xe3,0xeb,0xf3,0xfb,0x00,0x00,0x00,0x00,
		   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x10,0x18,
/* line 10 */	   0x1e,0x26,0x2e,0x36,0x3e,0x46,0x4e,0x56,0x5e,0x66,
		   0x6e,0x76,0x7e,0x86,0x8e,0x96,0x9e,0xa6,0xae,0xb6,
		   0xbe,0xc6,0xce,0xd6,0xde,0xe6,0xed,0xf5,0x00,0x00,
		   0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

     //Set Gamma  Set gamma curve related setting
	{0xe0,34,{0x00,0x00,0x00,0x12,0x1d,0x34,0x13,0x2a,0x04,0x0c,
	          0x12,0x16,0x17,0x16,0x16,0x0e,0x11,0x00,0x00,0x00,
		  0x18,0x24,0x3f,0x16,0x2c,0x04,0x0d,0x10,0x16,0x17,
		  0x16,0x16,0x10,0x11}},

    //Set Cabc   Set Cabc control (pwm=35kHz)
	{0xc9,9, {0x0f,0x00,0x1e,0x1e,0x00,0x00,0x00,0x01,0x3e}},

   //Wrdisbv Write display brigtness 
	{0x53,1, {0x00}},

   //Set VDC    
	{0xbc,1, {0x07}},

	{REGFLAG_DELAY, 10, {}},

    //Set panel 
        {0xcc,1, {0x09}},

	{REGFLAG_DELAY, 60, {}},

    //Set mipi control 
	{0xba,1, {0x03}},

    //Display on 
	{0x29,0, {}},

	{REGFLAG_DELAY, 60, {}},
};



/*
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 0, {}},
    {REGFLAG_DELAY, 220, {}},

    // Display ON
    {0x29, 0, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/


static LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {

    // Sleep Mode On
    {0x10, 0, {}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
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
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

    params->dsi.mode   = SYNC_PULSE_VDO_MODE;


    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
//chenwj add 15.03.24
    params->dsi.vertical_sync_active				= 3;
    params->dsi.vertical_backporch				= 3;
    params->dsi.vertical_frontporch				= 6;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 15;
    params->dsi.horizontal_backporch				= 70;
    params->dsi.horizontal_frontporch				= 91;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
//end

	params->dsi.PLL_CLOCK = 200; //this value must be in MTK suggested table
}
static void lcm_init(void)
{
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(20);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(LCM_setting_table), 1);

}


static void lcm_resume(void)
{
	MDELAY(10);
	lcm_init();
}

#define HX8394D_HD720_ID  (0x94)
static unsigned int lcm_compare_id(void)
{
//	unsigned int id = 0;
//	unsigned char buffer[3];
//
//	unsigned int data_array[16];
//
//
//	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
//	MDELAY(10);
//
//	SET_RESET_PIN(0);
//	MDELAY(10);
//
//	SET_RESET_PIN(1);
//	MDELAY(10);
//	MDELAY(20);
//
//	data_array[0]=0x00043902;
//	data_array[1]=0x9483FFB9;
//	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);
//
//	data_array[0] = 0x00033700;
//	dsi_set_cmdq(data_array, 1, 1);
//
//	read_reg_v2(0x04, buffer, 3);
//	id = (buffer[0] << 8) | buffer [1]; //we only need ID
//
//	printf(" hx8394_int_vdo read id, buf:0x%02x ,0x%02x,0x%02x, id=0X%X", buffer[0], buffer[1], buffer[2], id);
//
//	return (LCM_ID == id)?1:0;
	//return 1;
	char  buffer;
	unsigned int data_array[2];

	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= (0x33<<8)|0xba;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00013700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xf4, &buffer, 1);

#ifdef BUILD_LK
	printf("%s, LK debug: hx8394d id = 0x%08x\n", __func__, buffer);
	global_lcd_id_hx8394 = (buffer == HX8394D_HD720_ID ? 1 : 0);
#else
    printk("%s, kernel debug: hx8394d id = 0x%08x\n", __func__, buffer);
#endif
	return (buffer == HX8394D_HD720_ID ? 1 : 0);

}

LCM_DRIVER hx8394_hd720_dsi_vdo_lcm_drv = 
{
    .name			= "hx8394_hd720_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    = lcm_compare_id,
};
