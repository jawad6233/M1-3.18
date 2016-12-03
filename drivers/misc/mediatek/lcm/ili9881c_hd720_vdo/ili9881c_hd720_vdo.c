#ifdef BUILD_LK
//#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
//#else
//#include <mach/mt_gpio.h>
#endif

#include "lcm_drv.h"
//#include "cust_gpio_usage.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             								0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

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
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2_TIN(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2_TIN(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

typedef struct {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
} LCM_setting_table;

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




	{0xff,3,{0x98,0x81,0x03}},

	{0x01,1,{0x00}},
	{0x02,1,{0x00}},
	{0x03,1,{0x72}},
	{0x04,1,{0x00}},
	{0x05,1,{0x00}},
	{0x06,1,{0x09}},
	{0x07,1,{0x00}},
	{0x08,1,{0x00}},
	{0x09,1,{0x01}},
	{0x0A,1,{0x00}},
	{0x0B,1,{0x00}},
	{0x0C,1,{0x01}},
	{0x0D,1,{0x00}},
	{0x0E,1,{0x00}},
	{0x0F,1,{0x00}},
	{0x10,1,{0x00}},
	{0x11,1,{0x00}},
	{0x12,1,{0x00}},
	{0x13,1,{0x00}},
	{0x14,1,{0x00}},
	{0x15,1,{0x00}},
	{0x16,1,{0x00}},
	{0x17,1,{0x00}},
	{0x18,1,{0x00}},
	{0x19,1,{0x00}},
	{0x1A,1,{0x00}},
	{0x1B,1,{0x00}},
	{0x1C,1,{0x00}},
	{0x1D,1,{0x00}},
	{0x1E,1,{0x40}},
	{0x1F,1,{0x80}},
	{0x20,1,{0x05}},
	{0x21,1,{0x02}},
	{0x22,1,{0x00}},
	{0x23,1,{0x00}},
	{0x24,1,{0x00}},
	{0x25,1,{0x00}},
	{0x26,1,{0x00}},
	{0x27,1,{0x00}},
	{0x28,1,{0x33}},
	{0x29,1,{0x02}},
	{0x2A,1,{0x00}},
	{0x2B,1,{0x00}},
	{0x2C,1,{0x00}},
	{0x2D,1,{0x00}},
	{0x2E,1,{0x00}},
	{0x2F,1,{0x00}},
	{0x30,1,{0x00}},
	{0x31,1,{0x00}},
	{0x32,1,{0x00}},
	{0x33,1,{0x00}},
	{0x34,1,{0x04}},
	{0x35,1,{0x00}},
	{0x36,1,{0x00}},
	{0x37,1,{0x00}},
	{0x38,1,{0x3c}},
	{0x39,1,{0x00}},
	{0x3A,1,{0x40}},
	{0x3B,1,{0x40}},
	{0x3C,1,{0x00}},
	{0x3D,1,{0x00}},
	{0x3E,1,{0x00}},
	{0x3F,1,{0x00}},
	{0x40,1,{0x00}},
	{0x41,1,{0x00}},
	{0x42,1,{0x00}},
	{0x43,1,{0x00}},
	{0x44,1,{0x00}},

	{0x50,1,{0x01}},
	{0x51,1,{0x23}},
	{0x52,1,{0x45}},
	{0x53,1,{0x67}},
	{0x54,1,{0x89}},
	{0x55,1,{0xab}},
	{0x56,1,{0x01}},
	{0x57,1,{0x23}},
	{0x58,1,{0x45}},
	{0x59,1,{0x67}},
	{0x5A,1,{0x89}},
	{0x5B,1,{0xab}},
	{0x5C,1,{0xcd}},
	{0x5D,1,{0xef}},
	{0x5E,1,{0x11}},
	{0x5F,1,{0x01}},
	{0x60,1,{0x00}},
	{0x61,1,{0x15}},
	{0x62,1,{0x14}},
	{0x63,1,{0x0e}},
	{0x64,1,{0x0f}},
	{0x65,1,{0x0c}},
	{0x66,1,{0x0d}},
	{0x67,1,{0x06}},
	{0x68,1,{0x02}},
	{0x69,1,{0x02}},
	{0x6A,1,{0x02}},
	{0x6B,1,{0x02}},
	{0x6C,1,{0x02}},
	{0x6D,1,{0x02}},
	{0x6E,1,{0x07}},
	{0x6F,1,{0x02}},
	{0x70,1,{0x02}},
	{0x71,1,{0x02}},
	{0x72,1,{0x02}},
	{0x73,1,{0x02}},
	{0x74,1,{0x02}},
	{0x75,1,{0x01}},
	{0x76,1,{0x00}},
	{0x77,1,{0x14}},
	{0x78,1,{0x15}},
	{0x79,1,{0x0e}},
	{0x7A,1,{0x0f}},
	{0x7B,1,{0x0c}},
	{0x7C,1,{0x0d}},
	{0x7D,1,{0x06}},
	{0x7E,1,{0x02}},//00
	{0x7F,1,{0x02}},//00
	{0x80,1,{0x02}},//00
	{0x81,1,{0x02}},//00
	{0x82,1,{0x02}},//00
	{0x83,1,{0x02}},//00
	{0x84,1,{0x07}},//00
	{0x85,1,{0x02}},//00
	{0x86,1,{0x02}},//00
	{0x87,1,{0x02}},//00
	{0x88,1,{0x02}},//00
	{0x89,1,{0x02}},//00
	{0x8A,1,{0x02}},//00
	
	{0xff,3,{0x98,0x81,0x04}},
	{0x6c,1,{0x15}},
	{0x6e,1,{0x3b}},
	{0x6f,1,{0x33}},
	{0x3a,1,{0x94}},
	{0x8d,1,{0x1a}},
	{0x87,1,{0xba}},
	{0x26,1,{0x76}},
	{0xb2,1,{0xd1}},
	{0xb5,1,{0x06}},
	
	{0xff,3,{0x98,0x81,0x01}},
	{0x22,1,{0x09}},
	{0x31,1,{0x00}},
	{0x53,1,{0x84}},
	{0x55,1,{0x90}},
	{0x50,1,{0x98}},//new add
	{0x51,1,{0x98}},
	{0x60,1,{0x28}},
	{0xa0,1,{0x08}},
	{0xa1,1,{0x10}},

	{0xA2,1,{0x18}},
	{0xA3,1,{0x0e}},
	{0xA4,1,{0x0d}},
	{0xA5,1,{0x1e}},
	{0xA6,1,{0x12}},
	{0xA7,1,{0x16}},
	{0xA8,1,{0x4f}},
	{0xA9,1,{0x1a}},
	{0xAA,1,{0x26}},//18
	{0xAB,1,{0x50}},
	{0xAC,1,{0x1a}},
	{0xAD,1,{0x18}},
	{0xAE,1,{0x4a}},
	{0xAF,1,{0x1c}},
	{0xB0,1,{0x1c}},
	{0xB1,1,{0x54}},
	{0xB2,1,{0x65}},
	{0xB3,1,{0x39}},

	{0xC0,1,{0x08}},
	{0xC1,1,{0x10}},
	{0xC2,1,{0x18}},
	{0xC3,1,{0x0e}},
	{0xC4,1,{0x0d}},
	{0xC5,1,{0x1e}},
	{0xC6,1,{0x12}},
	{0xC7,1,{0x16}},
	{0xC8,1,{0x4f}},
	{0xC9,1,{0x1a}},
	{0xCA,1,{0x26}},
	{0xCB,1,{0x50}},
	{0xCC,1,{0x1a}},
	{0xCD,1,{0x18}},
	{0xCE,1,{0x4a}},
	{0xCF,1,{0x1c}},
	{0xD0,1,{0x1c}},
	{0xD1,1,{0x54}},
	{0xD2,1,{0x65}},
	{0xD3,1,{0x39}},
	{0xFF,3,{0x98,0x81,0x00}},
	{0x35,1,{0x00}},
	{0x3A,1,{0x77}},
	{0x11,0,{}},

	{REGFLAG_DELAY, 200, {}},

	{0x29,0,{}},
    {REGFLAG_DELAY, 200, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}},
};

/*
static LCM_setting_table lcm_sleep_out_setting[] = {
	   {0x11, 0, {}},
    {REGFLAG_DELAY, 120, {}},
    {0x29, 0, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {

    // Sleep Mode On
    {0x10, 0, {}},
    {REGFLAG_DELAY, 80, {}},
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
        params->dsi.vertical_sync_active				= 4;//4;
        params->dsi.vertical_backporch				= 40;//40;
        params->dsi.vertical_frontporch				= 40;//40;
        params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
        params->dsi.horizontal_sync_active			= 10;//4;
        params->dsi.horizontal_backporch				= 120;//82;
        params->dsi.horizontal_frontporch				= 120;//82;
        params->dsi.horizontal_active_pixel			= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 200; //this value must be in MTK suggested table
}
static void lcm_init(void)
{
    MDELAY(30); 
    SET_RESET_PIN(1);
    MDELAY(80);	


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
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];
    	MDELAY(30);
    	SET_RESET_PIN(1);
    	MDELAY(80);

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xDA, buffer, 1);
	
	id = buffer[0]; //we only need ID
#ifdef BUILD_LK
	printf("%s,  id ili9881c_hd720_vdo = 0x%08x\n", __func__, id);
#else
	printk("%s,  id ili9881c_hd720_vdo = 0x%08x\n", __func__, id);
#endif
	//return (0x40 == id)?1:0;
	return 1;
}

LCM_DRIVER ili9881c_hd720_vdo_lcm_drv= 
{
    .name			= "ili9881c_hd720_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    = lcm_compare_id,
};
