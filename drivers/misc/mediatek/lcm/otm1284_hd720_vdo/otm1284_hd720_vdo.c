#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
 #include <linux/proc_fs.h>
#include <linux/fs.h>
 #include <linux/slab.h>
#endif

#include "lcm_drv.h"
//#include "../../../lentk6735_65c_l1/dct/dct/cust_gpio_usage.h"
#include "cust_gpio_usage.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(800)
#define FRAME_HEIGHT 										(1280)

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2_TIN(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2_TIN(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
       

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
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
    {REGFLAG_DELAY, 120, {}},

//#Command 2 Enable(step-1}
{0x00,1,{0x00}}, 
{0xFF,3,{0x12,0x84,0x01}}, 

//#Command 2 Enable(step-2} 
{0x00,1,{0x80}}, 
{0xFF,2,{0x12,0x84}}, 

{0x00,1,{0x92}},                  
{0xff,2,{0x30,0x02}},  
//#---------panel setting------------
//#TCON Setting Parameter
{0x00,1,{0x80}},
{0xc0,9,{0x00,0x64,0x00,0x15,0x15,0x00,0x64,0x15,0x15}}, 

//#Panel Timing Setting Parameter
{0x00,1,{0x90}}, 
{0xC0,6,{0x00,0x5C,0x00,0x01,0x00,0x04}}, 



//#Oscillator Adjustment for ldle/Normal Mode
{0x00,1,{0x81}}, 
{0xC1,1,{0x63}}, //66



//#Zigzag inversion 
{0x00,1,{0xA6}}, 
{0xB3,2,{0x0F,0x01}}, 



//#internal data latch timing
{0x00,1,{0x92}}, 
{0xC4,1,{0x00}}, 


//#---------power setting------------
//#DC2DC Setting
{0x00,1,{0xA0}}, 
{0xC4,14,{0x05,0x10,0x06,0x02,0x05,0x15,0x10,0x05,0x10,0x07,0x02,0x05,0x15,0x10}}, 

//#Power Control Setting
//# BOOSTCLP (C4B0~C4B1h}: VSP/VSN voltage setting 
{0x00,1,{0xB0}},              
{0xC4,2,{0x22,0x00}},            

//# Power Control Setting
//# HVSET (C591~C593h}: VGH/VGL voltage setting    p.130  
{0x00,1,{0x91}},                  
{0xC5,2,{0x46,0x42}},            

//#GVDDSET (D800h}: GVDD/NGVDD Voltage setting   
{0x00,1,{0x00}},               
{0xD8,2,{0xC7,0xC7}},           

//# VCOMDC (D900h}: Voltage setting   §ï0x65ªº­È§ä³Ì¨ÎVCOM 
{0x00,1,{0x00}}, 
{0xD9,2,{0x65,0x65}}, 

   

{0x00,1,{0x87}}, 
{0xC4,1,{0x18}},      


 

//#VDD_18V/LVDSVDD Voltage setting
{0x00,1,{0xB3}}, 
{0xC5,1,{0x84}}, 

//# Power Control Setting
//# LVDSET (C5BBh}: LVD Setting 
{0x00,1,{0xBB}}, 
{0xC5,1,{0x8a}}, 

//#Power Control Setting
{0x00,1,{0x82}}, 
{0xC4,1,{0x0a}}, 

//#Power Control Setting
{0x00,1,{0xC6}}, 
{0xB0,1,{0x03}}, 

//#precharge disable
{0x00,1,{0xC2}}, 
{0xF5,1,{0x40}}, 

//#sample hold gvdd
{0x00,1,{0xC3}}, 
{0xF5,1,{0x85}}, 

//#ID1
{0x00,1,{0x00}}, 
{0xD0,1,{0x40}}, 

//#ID2.ID3
{0x00,1,{0x00}}, 
{0xD1,2,{0x00,0x00}}, 


//#---------power IC------------
//#Power IC Setting1
{0x00,1,{0x90}}, 
{0xF5,4,{0x02,0x11,0x02,0x15}}, 

//#Power IC Setting2
{0x00,1,{0x90}}, 
{0xC5,1,{0x50}}, 

//#Power IC Setting3        
{0x00,1,{0x94}}, 
{0xC5,3,{0x66,0x66,0x63}},          

//#VGLO1 setting
{0x00,1,{0xB2}}, 
{0xF5,2,{0x00,0x00}}, 

//#VGLO1_s setting
{0x00,1,{0xB4}}, 
{0xF5,2,{0x00,0x00}}, 

//#VGLO2 setting
{0x00,1,{0xB6}}, 
{0xF5,2,{0x00,0x00}}, 

//#VGLO2_s setting
{0x00,1,{0xB8}}, 
{0xF5,2,{0x00,0x00}}, 

//#VCL on
{0x00,1,{0x94}}, 
{0xF5,2,{0x00,0x00}}, 

//#VCL reg.en
{0x00,1,{0xD2}}, 
{0xF5,2,{0x06,0x15}}, 

//#VGL 1/2 pull low
{0x00,1,{0xB4}}, 
{0xC5,1,{0xCC}}, 

//#VSP on
//#{0x00,1,{0xBA}}, 
//#{0xF5,0x03}}, 

//#VGHO Option
//#{0x00,1,{0xB2}}, 
//#{0xC5,0x40}}, 

//#VGLO Option
{0x00,1,{0xB4}}, 
{0xC5,1,{0xCC}}, 

//#---------power IC------------

//#TCON_GOA_WAVE(Panel timing state control}
{0x00,1,{0xC0}}, 
{0xCB,15,{0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x00,0x15,0x15,0x00,0x00,0x00}}, 

//#TCON_GOA_WAVE(Panel timing state control}
{0x00,1,{0xD0}}, 
{0xCB,15,{0x15,0x15,0x00,0x15,0x15,0x15,0x00,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15}}, 

//#TCON_GOA_WAVE(Panel timing state control}
{0x00,1,{0xE0}}, 
{0xCB,14,{0x15,0x00,0x15,0x15,0x00,0x00,0x00,0x15,0x15,0x00,0x15,0x15,0x15,0x00}}, 

//#TCON_GOA_WAVE(Panel timing state control}
{0x00,1,{0xF0}}, 
{0xCB,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}}, 

//#---------goa mapping------------
//#TCON_GOA_WAVE(Panel pad mapping control}
{0x00,1,{0x80}}, 
{0xCC,15,{0x14,0x0c,0x12,0x0a,0x0e,0x16,0x10,0x18,0x02,0x00,0x2f,0x2f,0x00,0x00,0x00}}, 
               
//#TCON_GOA_WAVE(Panel pad mapping control}
{0x00,1,{0x90}}, 
{0xCC,15,{0x29,0x2a,0x00,0x2d,0x2e,0x06,0x00,0x13,0x0b,0x11,0x09,0x0d,0x15,0x0f,0x17}},  


//#TCON_GOA_WAVE(Panel pad mapping control}
{0x00,1,{0xA0}}, 
{0xCC,14,{0x01,0x00,0x2f,0x2f,0x00,0x00,0x00,0x29,0x2a,0x00,0x2d,0x2e,0x05,0x00}}, 

                    
//#TCON_GOA_WAVE(Panel pad mapping control}
{0x00,1,{0xB0}}, 
{0xCC,15,{0x0d,0x15,0x0f,0x17,0x13,0x0b,0x11,0x09,0x05,0x00,0x2f,0x2f,0x00,0x00,0x00}}, 
                     
//#TCON_GOA_WAVE(Panel pad mapping control}
{0x00,1,{0xC0}}, 
{0xCC,15,{0x29,0x2a,0x00,0x2e,0x2d,0x01,0x00,0x0e,0x16,0x10,0x18,0x14,0x0c,0x12,0x0a}}, 

                    
//#TCON_GOA_WAVE(Panel pad mapping control}
{0x00,1,{0xD0}}, 
{0xCC,14,{0x06,0x00,0x2f,0x2f,0x00,0x00,0x00,0x29,0x2a,0x00,0x2e,0x2d,0x02,0x00}}, 

                   
//#---------goa timing------------
//#TCON_GOA_WAVE(VST setting}
{0x00,1,{0x80}}, 
{0xCE,12,{0x8F,0x0D,0x1A,0x8E,0x0D,0x1A,0x00,0x00,0x00,0x00,0x00,0x00}}, 
                     
//# TCON_GOA_WAVE(VEND stting}
{0x00,1,{0x90}}, 
{0xCE,14,{0x75,0x00,0x1A,0x75,0x01,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 

                     
//# TCON_GOA_WAVE(TCON_GOA_WAVE}
{0x00,1,{0xA0}}, 
{0xCE,14,{0x78,0x07,0x84,0xF0,0x90,0x1A,0x00,0x78,0x06,0x84,0xF1,0x91,0x1A,0x00}}, 
                     
//# TCON_GOA_WAVE(TCON_GOA_WAVE}
{0x00,1,{0xB0}}, 
{0xCE,14,{0x78,0x05,0x84,0xF2,0x90,0x1A,0x00,0x78,0x04,0x84,0xF3,0x91,0x1A,0x00}}, 
                     
//# TCON_GOA_WAVE(TCON_GOA_WAVE}
{0x00,1,{0xC0}}, 
{0xCE,14,{0x78,0x03,0x84,0xF4,0x90,0x1A,0x00,0x78,0x02,0x84,0xF5,0x91,0x1A,0x00}}, 
                     
//# TCON_GOA_WAVE(TCON_GOA_WAVE}
{0x00,1,{0xD0}}, 
{0xCE,14,{0x78,0x01,0x84,0xF6,0x90,0x1A,0x00,0x78,0x00,0x84,0xF7,0x91,0x1A,0x00}}, 
                      
//# TCON_GOA_WAVE(TCON_GOA_WAVE}
{0x00,1,{0x80}}, 
{0xCF,14,{0x70,0x00,0x84,0xF8,0x90,0x1A,0x00,0x70,0x01,0x84,0xF9,0x91,0x1A,0x00}}, 
                     
//# TCON_GOA_WAVE(TCON_GOA_WAVE}
{0x00,1,{0x90}}, 
{0xCF,14,{0x70,0x02,0x84,0xFA,0x90,0x1A,0x00,0x70,0x03,0x84,0xFB,0x91,0x1A,0x00}}, 
                     
//# TCON_GOA_WAVE(TCON_GOA_WAVE}
{0x00,1,{0xA0}}, 
{0xCF,14,{0x70,0x04,0x84,0xFC,0x90,0x1A,0x00,0x70,0x05,0x84,0xFD,0x91,0x1A,0x00}}, 
                     
//# TCON_GOA_WAVE(TCON_GOA_WAVE}
{0x00,1,{0xB0}}, 
{0xCF,14,{0x70,0x06,0x84,0xFE,0x90,0x1A,0x00,0x70,0x07,0x84,0xFF,0x91,0x1A,0x00}}, 

//# TCON_GOA_WAVE(TCON_GOA_WAVE} 
{0x00,1,{0xC0}},                     
{0xCF,11,{0x01,0x01,0x64,0x64,0x00,0x00,0x01,0x00,0x00,0x00,0x00}}, 
                                                              
//# TCON_GOA_OUT Setting
{0x00,1,{0xB5}}, 
{0xC5,6,{0x3f,0xff,0xff,0x3f,0xff,0xff}}, 
                     
//# Gamma 2.2+
{0x00,1,{0x00}}, 
{0xE1,20,{0x00,0x19,0x29,0x3A,0x4C,0x5C,0x5D,0x8B,0x7D,0x98,0x67,0x53,0x63,0x40,0x3E,0x32,0X26,0X1A,0X0B,0X00}}, 


//# Gamma 2.2-
{0x00,1,{0x00}}, 
{0xE2,20,{0x00,0x19,0x29,0x3A,0x4C,0x5C,0x5D,0x8B,0x7D,0x98,0x67,0x53,0x63,0x40,0x3E,0x32,0X26,0X1A,0X0B,0X00}}, 

//#---------------------255--253--251--248--244--239-231--203--175--143--112---80---52---24---16---11---7----4----2----0                     
                     

{0x00,1,{0xA4}}, 
{0xC1,1,{0xF0}}, 


{0x00,1,{0x92}}, 
{0xC4,1,{0x00}}, 

{0x00,1,{0xB4}}, 
{0xC5,1,{0xCC}}, 

//Ink 2016.05.03
{0x51,1,{0xA6}},
{0x53,1,{0x2C}},
//end

{0x00,1,{0x90}}, 
{0xB6,1,{0xB6}}, 

{0x00,1,{0x92}}, 
{0xB3,1,{0x06}}, 

{0x00,1,{0xC2}}, 
{0xF5,1,{0x40}}, 


{0x00,1,{0x80}}, 
{0xFF,2,{0xFF,0xFF}}, 

    {0x29, 0, {}},
{REGFLAG_DELAY, 20, {}},
};

//Ink 2016.04.26
int dt900_lcd_is_otm =0;
extern char* saved_command_line;
int dt900_find_lcd(void){
	char* p;
	//printk("dt900_find_lcd %s\n",saved_command_line);
    p=strstr(saved_command_line, "otm1284_hd720_vdo");
        if (p!=NULL)
		{
			printk("dt900_find_lcd otm1284_hd720_vdo found\n");
			return 1;
		}
        else
		{
			printk("dt900_find_lcd otm1284_hd720_vdo not found\n");
			return 2;
		}
}
int otm1284_lcm_set_backlight(int level)
{
    unsigned int data_array[16];
	int new_level;
	new_level = 256-level;
	//printk("otm1284_lcm_set_backlight level:%d\n",new_level);
	if (new_level)
	{
		dsi_set_cmdq_V2(0x51, 1, &new_level, 1);
		data_array[0] = 0x2C531500;
		dsi_set_cmdq(data_array, 1, 1);
	}
	else
		{
		mt_set_gpio_mode(GPIO_LCM_PWR,GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCM_PWR, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ZERO);
	}
}
//end

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 0, {}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 0, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {

    // Sleep Mode On
    {0x10, 0, {}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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
	params->dsi.vertical_sync_active				= 8;
    params->dsi.vertical_backporch					= 8;
    params->dsi.vertical_frontporch					= 8;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 76;
    params->dsi.horizontal_backporch				= 16;
    params->dsi.horizontal_frontporch				= 16;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
//end

	params->dsi.PLL_CLOCK = 220; //this value must be in MTK suggested table
/*
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
*/
}
//#define GPIO_VDD_3V3_EN_PIN 95
static void lcm_init(void)
{
    mt_set_gpio_mode(GPIO_VDD_3V3_EN_PIN,GPIO_MODE_00);   //enable vdd=3.3v
    mt_set_gpio_dir(GPIO_VDD_3V3_EN_PIN,GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_VDD_3V3_EN_PIN,GPIO_OUT_ONE);
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	mt_set_gpio_mode(GPIO_LCM_PWR,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ZERO);
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_resume(void)
{
	MDELAY(10);
	lcm_init();
	mt_set_gpio_mode(GPIO_LCM_PWR,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ONE);
}

static unsigned int lcm_compare_id(void)
{
     unsigned int id = 0;
     unsigned char buffer[2];
     unsigned int array[16];
     mt_set_gpio_mode(GPIO_VDD_3V3_EN_PIN,GPIO_MODE_00);   //enable vdd=3.3v
     mt_set_gpio_dir(GPIO_VDD_3V3_EN_PIN,GPIO_DIR_OUT);
     mt_set_gpio_out(GPIO_VDD_3V3_EN_PIN,GPIO_OUT_ONE);
 
    SET_RESET_PIN(1);
    MDELAY(10);
     SET_RESET_PIN(0);
     MDELAY(50);
     SET_RESET_PIN(1);
     MDELAY(120);
 
     array[0] = 0x00023700;// read id return two byte,version and id
     dsi_set_cmdq(array, 1, 1);
     read_reg_v2(0xDA, buffer, 1);
 
     id = buffer[0]; //we only need ID
#ifdef BUILD_LK
     printf("%s,  id otm1284A= 0x%08x\n", __func__, id);
#else 
     printk("%s,  id otm1284A= 0x%08x\n", __func__, id);
#endif
     if (id == 0x40)
     {
     	return 1 ;
     }
     else 
     {
     	return 0;
     }
}

LCM_DRIVER otm1284_hd720_vdo_lcm_drv= 
{
    .name			= "otm1284_hd720_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    = lcm_compare_id,
};
