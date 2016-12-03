#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#endif

#include "lcm_drv.h"
#include "cust_gpio_usage.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------


#define FRAME_WIDTH  										(1200)
#define FRAME_HEIGHT 										(1920)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

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

#define dsi_set_cmdq_V3(ppara, size, force_update)	        	lcm_util.dsi_set_cmdq_V3(ppara, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2_TIN(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2_TIN(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
       


static void lcm_init_register(void)
{
   unsigned int data_array[16];

    MDELAY(50);

    data_array[0] = 0x00010500; 
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

    data_array[0] = 0x00B02300; 						 
    dsi_set_cmdq(data_array, 1, 1); 

    data_array[0] = 0x00022902;
    data_array[1] = 0x000001D6;  
    dsi_set_cmdq(data_array, 2, 1); 

    data_array[0] = 0x00062902;  //interface setting	
    data_array[1] = 0x000814B3; 
    data_array[2] = 0x00000022; 
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00022902;  //interface ID setting
    data_array[1] = 0x00000CB4;  
    dsi_set_cmdq(data_array, 2, 1); 
		
    data_array[0] = 0x00032902;
    data_array[1] = 0x00C33AB6; 
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0xE6511500;
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0] = 0x2C531500;
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0] = 0x773A1500;
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0]= 0x00053902;
    data_array[1]= 0x0400002A;
    data_array[2]= 0x000000AF;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= 0x0700002B;
    data_array[2]= 0x0000007F;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);

	
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
    //params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
//params->dsi.mode   = SYNC_EVENT_VDO_MODE;
//params->dsi.mode   = BURST_VDO_MODE;


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
    //params->dsi.packet_size=256;
	params->dsi.word_count=FRAME_WIDTH*3;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
//chenwj add 15.03.24

   // Video mode setting		
	params->dsi.vertical_sync_active = 2;//2 8 8
    params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 17;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 16;// 12 60 120
	params->dsi.horizontal_backporch = 32;
	params->dsi.horizontal_frontporch = 120;	
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

     params->dsi.PLL_CLOCK = 440;

//	params->dsi.cont_clock				= 0;

//	params->dsi.noncont_clock			=1;
//	params->dsi.noncont_clock_period 	= 2;
}
static void lcm_init(void)
{
	
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(30);
	SET_RESET_PIN(1);
	MDELAY(100);
	lcm_init_register();
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];  

//Ink add 2016.06.30
      data_array[0] = 0x00B02300;
      dsi_set_cmdq(data_array, 1, 1);
      data_array[0] = 0x00B12301;
      dsi_set_cmdq(data_array, 1, 1);
  //end
    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);

	data_array[0]=0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(80);	
	mt_set_gpio_mode(GPIO_VDD_3V3_EN_PIN,GPIO_MODE_00); 
	mt_set_gpio_dir(GPIO_VDD_3V3_EN_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_VDD_3V3_EN_PIN,GPIO_OUT_ZERO);
}


static void lcm_resume(void)
{
	mt_set_gpio_mode(GPIO_VDD_3V3_EN_PIN,GPIO_MODE_00);   
	mt_set_gpio_dir(GPIO_VDD_3V3_EN_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_VDD_3V3_EN_PIN,GPIO_OUT_ONE);
	MDELAY(80);
	lcm_init();
}

int jdi_lcm_set_backlight(int level)
{
    unsigned int data_array[16];  
	int new_level;
	new_level=level;
	if (new_level)
	{
		dsi_set_cmdq_V2(0x51, 1, &new_level, 1);
		data_array[0] = 0x2C531500;
		dsi_set_cmdq(data_array, 1, 1);
	}else
	{
		mt_set_gpio_mode(GPIO_LCM_PWR,GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCM_PWR, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ZERO);
	}
}


static unsigned int lcm_compare_id(void)
{
	mt_set_gpio_mode(GPIO_VDD_3V3_EN_PIN,GPIO_MODE_00);   // This pin control DCDC. Enable it boot,resume．Disable it suspend
	mt_set_gpio_dir(GPIO_VDD_3V3_EN_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_VDD_3V3_EN_PIN,GPIO_OUT_ONE);

return 1;

}

LCM_DRIVER jdi0012_fhd_dsi_vdo_lcm_drv = 
{
    .name			= "jdi0012_fhd_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    = lcm_compare_id,
};
