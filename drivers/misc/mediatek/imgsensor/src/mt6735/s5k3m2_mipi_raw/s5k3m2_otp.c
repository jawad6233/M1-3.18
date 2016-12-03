#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k3m2mipi_Sensor.h"

extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static kal_uint16 s5k3m2_slave_addr = 0xA0;
#define SENSORDB(fmt,args...) printk("%s [%d] : "fmt, __FUNCTION__, __LINE__, ##args)	
#define Sunny_SUPPORT_OTP
#ifdef Sunny_SUPPORT_OTP

#define USHORT             unsigned short
#define BYTE               unsigned char

#define SUNNY_ID           0x01

#define GAIN_DEFAULT       0x0100
#define GAIN_GREEN1_ADDR   0x020E
#define GAIN_RED_ADDR      0x0210
#define GAIN_BLUE_ADDR     0x0212
#define GAIN_GREEN2_ADDR   0x0214


kal_uint32 sunny_r_ratio;
kal_uint32 sunny_b_ratio;
kal_uint32 sunny_g_ratio;

kal_uint32 GOLDEN_RG_RATIO = 562;	// R/G Typical value
kal_uint32 GOLDEN_BG_RATIO = 569;	// B/G Typical value

static kal_uint16 s5k3m2_eeprom_addr = 0xA0;	



static inline void S5K3M2_wordwrite_cmos_sensor(u16 addr, u32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,  (char)(para >> 8),	(char)(para & 0xFF) };
	iWriteRegI2C(puSendCmd , 4,s5k3m2_slave_addr);
}

static inline void S5K3M2_bytewrite_cmos_sensor(u16 addr, u32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF)  ,	(char)(para & 0xFF) };
	iWriteRegI2C(puSendCmd , 3,s5k3m2_slave_addr);
}

	
static bool read_eeprom_byte(kal_uint16 addr, u8* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > 0xFFFF)
        return false;
	
	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, s5k3m2_eeprom_addr)<0)
		return false;
    return true;
}

static u8 read_eeprom_byte_8(kal_uint16 addr)
{
    
    u8 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,s5k3m2_eeprom_addr);
    return get_byte;
}

static void read_eeprom_awb(void)
{
	int i=0;
	u8 eeprom_data[100]={0};
	kal_uint16 addr = 0x001D;
	
	for(i=0;i<6;i++)
	{
		eeprom_data[i] = read_eeprom_byte_8(addr);
		addr++;
		printk("===mlk=== read eeprom_data[%d] = 0x%x \n",i,eeprom_data[i]);
	}
	
	sunny_r_ratio = eeprom_data[0]<<8 | eeprom_data[1];
	sunny_b_ratio = eeprom_data[2]<<8 | eeprom_data[3];
	sunny_g_ratio = eeprom_data[4]<<8 | eeprom_data[5];
	printk("sunny_r_ratio = 0x%x\n",sunny_r_ratio );
	printk("sunny_b_ratio = 0x%x\n",sunny_b_ratio );
	printk("sunny_g_ratio = 0x%x\n",sunny_g_ratio );
		
}

static bool sunny_wb_gain_set(void)
{
	USHORT R_GAIN, R_GAIN_H, R_GAIN_L;
	USHORT B_GAIN, B_GAIN_H, B_GAIN_L;
	USHORT Gr_GAIN;
	USHORT Gb_GAIN;
	USHORT G_GAIN, G_GAIN_H, G_GAIN_L;
		
   if(!sunny_r_ratio || !sunny_b_ratio)
   {
            SENSORDB("otp_sunny WB ratio Data Err!");
            return 0;
   }


	if(sunny_b_ratio < GOLDEN_BG_RATIO)                                    
	{                                                                                        
		if (sunny_r_ratio < GOLDEN_RG_RATIO)                                      
		{                                                                         
			G_GAIN = GAIN_DEFAULT;                                                  
			B_GAIN = GAIN_DEFAULT * GOLDEN_BG_RATIO / sunny_b_ratio;                
			R_GAIN = GAIN_DEFAULT * GOLDEN_RG_RATIO / sunny_r_ratio;                
		}                                                                         
		else                                                                      
		{                                                                         
			R_GAIN = GAIN_DEFAULT;                                                  
			G_GAIN = GAIN_DEFAULT * sunny_r_ratio / GOLDEN_RG_RATIO;                
			B_GAIN = G_GAIN * GOLDEN_BG_RATIO / sunny_b_ratio;                      
		}                                                                         
	}                                                                           
	else                                                                        
	{                                                                           
		if (sunny_r_ratio < GOLDEN_RG_RATIO)                                      
		{                                                                         
			B_GAIN = GAIN_DEFAULT;                                                  
			G_GAIN = GAIN_DEFAULT * sunny_b_ratio / GOLDEN_BG_RATIO;                
			R_GAIN = G_GAIN * GOLDEN_RG_RATIO / sunny_r_ratio;                      
		}                                                                         
		else                                                                      
		{                                                                         
			Gb_GAIN = GAIN_DEFAULT * sunny_b_ratio / GOLDEN_BG_RATIO;              
			Gr_GAIN = GAIN_DEFAULT * sunny_r_ratio / GOLDEN_RG_RATIO;              
			if(Gb_GAIN > Gr_GAIN )                                                  
			{                                                                       
				B_GAIN = GAIN_DEFAULT;                                                
				G_GAIN = Gb_GAIN;                                                     
				R_GAIN = G_GAIN * GOLDEN_RG_RATIO / sunny_r_ratio;                    
			}                                                                       
			else                                                                    
			{                                                                       
				R_GAIN = GAIN_DEFAULT;                                                
				G_GAIN = Gr_GAIN;                                                     
				B_GAIN = G_GAIN * GOLDEN_BG_RATIO / sunny_b_ratio;                    
			}                                                                       
		}                                                                         
	}                                                                           
	SENSORDB("otp_sunny_sunny_r_ratio=%d,sunny_b_ratio=%d \n",sunny_r_ratio,sunny_b_ratio);

	if(R_GAIN < GAIN_DEFAULT)
		R_GAIN = GAIN_DEFAULT;
	if(G_GAIN < GAIN_DEFAULT)
		G_GAIN = GAIN_DEFAULT;
	if(B_GAIN < GAIN_DEFAULT)
		B_GAIN = GAIN_DEFAULT;
		
	R_GAIN_H = (R_GAIN >> 8) & 0xFF;
	R_GAIN_L = R_GAIN & 0xFF;
	B_GAIN_H = (B_GAIN >> 8) & 0xFF;
	B_GAIN_L = B_GAIN & 0xFF;
	G_GAIN_H = (G_GAIN >> 8) & 0xFF;
	G_GAIN_L = G_GAIN & 0xFF;


	S5K3M2_bytewrite_cmos_sensor(GAIN_RED_ADDR, R_GAIN_H);		
	S5K3M2_bytewrite_cmos_sensor(GAIN_RED_ADDR+1, R_GAIN_L);
	S5K3M2_bytewrite_cmos_sensor(GAIN_BLUE_ADDR, B_GAIN_H);
	S5K3M2_bytewrite_cmos_sensor(GAIN_BLUE_ADDR+1, B_GAIN_L);     
	S5K3M2_bytewrite_cmos_sensor(GAIN_GREEN1_ADDR, G_GAIN_H);     
	S5K3M2_bytewrite_cmos_sensor(GAIN_GREEN1_ADDR+1, G_GAIN_L); //Green 1 default gain 1x		
	S5K3M2_bytewrite_cmos_sensor(GAIN_GREEN2_ADDR, G_GAIN_H); //Green 2 default gain 1x
	S5K3M2_bytewrite_cmos_sensor(GAIN_GREEN2_ADDR+1, G_GAIN_L);
	SENSORDB("otp_sunny WB Update Finished! \n");
	return 1;
}


BYTE S5K3M2_otp(void)
{ 
	
	
	u8 value[2]={0};
	u8 value1[2]={0};
	u8 value2[2]={0};
	//Ink 2016.07.26
	//read_eeprom_byte(0x0001, &value);
	read_eeprom_byte(0x0001, value);
	read_eeprom_byte(0x000A, value1);
	//end
	value2[0]= read_eeprom_byte_8(0x0001);
	value2[1]= read_eeprom_byte_8(0x000A);
	
	printk("===mlk===Module ID = 0x%x,SOFT ID = 0x%x\n",value[0],value1[0]);
	printk("===mlk1===Module ID = 0x%x,SOFT ID = 0x%x\n",value2[0],value2[1]);
	
	read_eeprom_awb();
	sunny_wb_gain_set();
	return 0;
}

#endif
