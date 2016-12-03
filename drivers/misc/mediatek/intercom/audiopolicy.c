#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>  
#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/irq.h>
#include <asm/scatterlist.h>
#include <mach/irqs.h>
#include <linux/slab.h>
#include "linux/delay.h"
#include <linux/input.h>

#define PTT_PIN_RELEASE                0
#define PTT_PIN_PRESS                  1
#define PTT_PIN_DEFAULT                PTT_PIN_RELEASE 
#define SQ_PIN_RELEASE                 0
#define SQ_PIN_PRESS                   1

#define HEADSET_MODE                   0
#define SPEAKER_MODE                   1
#define	POWER_OFF                      0
#define POWER_ON                       1
#define SPK_OFF_MODE                   0
#define SPK_SILENT_MODE                0
#define SPK_L_MODE                     1
#define SPK_M_MODE                     2
#define SPK_H_MODE                     3
#define INTERCOM_SPK_DEFAULT           SPK_H_MODE
#define SYSTEM_SPK_DEFAULT             SPK_M_MODE
#define SYSTEM_SPK_ON                  0x01 //0000,0001: 0x01 | spk_status
#define SYSTEM_SPK_OFF                 0xfe //1111,1110: 0xfe & spk_status
#define INTERCOM_SPK_ON                0x02 //0000,0010: 0x02 | spk_status
#define INTERCOM_SPK_OFF               0xfd //1111,1101: 0xfd & spk_status
#define INTERCOM_AND_SYS_SPK_OFF       0    //0000,0000

static struct pinctrl *audio_pinctrl;
//static struct pinctrl_state *audio_pins_default;
static struct pinctrl_state *mic_main,*mic_headset,*spk_enable,*spk_disable;
extern volatile int intercom_using_flag;
extern volatile int sq_status;
volatile unsigned char spk_status = 0;
volatile unsigned int intercom_audio_mode = SPEAKER_MODE;//switch status when headset plug in or plug out.
volatile unsigned int intercom_spk_mode = INTERCOM_SPK_DEFAULT;

//extern unsigned int Intercom_Ext_Speaker_Change(int power);
int audio_get_gpio_info(void *dev);
static void external_amplifier_mode(int amp_mode);
void set_intercom_mic_channel(int mic_switch_en);
void set_intercom_spk_mode(int spk_en);
unsigned int intercom_audio_switch(int audio_channel);
unsigned int switch_intercom_spk_mode(int switch_spk_mode);// silent/low/middle/high
void set_system_spk_mode(int spk_en);
int get_intercom_spk_mode(void);
extern void TurnOnIntercomMicPowerACC(bool enable, int audio_mode);

int audio_get_gpio_info(void *dev)
{
	int ret = 0;
	audio_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(audio_pinctrl)) {
        ret = PTR_ERR(audio_pinctrl);
        printk(KERN_ERR "Runbo Cannot find audio_pinctrl!\n");
        return ret;
    }
	
	mic_main = pinctrl_lookup_state(audio_pinctrl, "intercom_mic_main");
    if (IS_ERR(mic_main)) {
        ret = PTR_ERR(mic_main);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl mic_main %d!\n", ret);
    }
	mic_headset = pinctrl_lookup_state(audio_pinctrl, "intercom_mic_headset");
    if (IS_ERR(mic_headset)) {
        ret = PTR_ERR(mic_headset);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl mic_headset %d!\n", ret);
    }
	
	spk_enable = pinctrl_lookup_state(audio_pinctrl, "intercom_spk_enable");
    if (IS_ERR(spk_enable)) {
        ret = PTR_ERR(spk_enable);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl spk_enable %d!\n", ret);
    }
	spk_disable = pinctrl_lookup_state(audio_pinctrl, "intercom_spk_disable");
    if (IS_ERR(spk_disable)) {
        ret = PTR_ERR(spk_disable);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl spk_disable %d!\n", ret);
    }
	
	pinctrl_select_state(audio_pinctrl, mic_main);
	pinctrl_select_state(audio_pinctrl, spk_disable);
	
	return 0;
}
EXPORT_SYMBOL(audio_get_gpio_info);

static void external_amplifier_mode(int amp_mode)
{
	switch (amp_mode)
	{
		//SPK_OFF_MODE/SPK_SILENT_MODE
		case SPK_SILENT_MODE:
				pinctrl_select_state(audio_pinctrl, spk_disable);
				printk(KERN_ERR "Runbo SPK_SILENT_MODE!\n");
				break;
		case SPK_L_MODE:
				//Mode 1
				pinctrl_select_state(audio_pinctrl, spk_disable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_enable);
				printk(KERN_ERR "Runbo SPK_L_MODE!\n");
				break;
		case SPK_M_MODE:
				//Mode 3
				pinctrl_select_state(audio_pinctrl, spk_disable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_enable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_disable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_enable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_disable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_enable);
				printk(KERN_ERR "Runbo SPK_M_MODE!\n");
				break;
		case SPK_H_MODE:
				//Mode 4
				pinctrl_select_state(audio_pinctrl, spk_disable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_enable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_disable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_enable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_disable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_enable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_disable);
				udelay(2);
				pinctrl_select_state(audio_pinctrl, spk_enable);
				printk(KERN_ERR "Runbo SPK_H_MODE!\n");
				break;
		default:
				break;
	}
}

void set_intercom_mic_channel(int mic_switch_en)
{
	switch (mic_switch_en)
	{
		case POWER_OFF:
			pinctrl_select_state(audio_pinctrl, mic_main);
			TurnOnIntercomMicPowerACC(false, intercom_audio_mode);
			break;
		case POWER_ON:
			if (intercom_audio_mode == HEADSET_MODE)
			{
				pinctrl_select_state(audio_pinctrl, mic_headset);
				printk(KERN_ERR "Runbo:intercom headset mode.\n");
			}
			else if (intercom_audio_mode == SPEAKER_MODE)
			{
				pinctrl_select_state(audio_pinctrl, mic_main);
				TurnOnIntercomMicPowerACC(true, intercom_audio_mode);
				printk(KERN_ERR "Runbo:intercom speaker mode.\n");
			}
			break;
		default:
			break;
	}
}

void set_intercom_spk_mode(int spk_en)
{
	if (spk_en == 1)
		spk_status |= INTERCOM_SPK_ON;
	else
		spk_status &= INTERCOM_SPK_OFF;
		
	switch (spk_en)
	{
		case POWER_OFF:
			//both system and intercom spk turn off,so turn off speaker.
			if (spk_status == INTERCOM_AND_SYS_SPK_OFF)
			{
				external_amplifier_mode(SPK_OFF_MODE);
				printk(KERN_ERR "Runbo:set_intercom_spk_mode poweroff.\n");
			}
			//when turn off intercom spk,the system spk still running,so switch spk to system spk mode.
			else
			{
				set_system_spk_mode(POWER_ON);
				printk(KERN_ERR "Runbo:set_intercom_spk_mode resume as system spk mode.\n");
			}
			break;
		case POWER_ON:
			if (intercom_audio_mode == HEADSET_MODE)
			{
				external_amplifier_mode(SPK_SILENT_MODE);
				printk(KERN_ERR "Runbo:set_intercom_spk_mode slient mode.\n");
			}
			else if (intercom_audio_mode == SPEAKER_MODE)
			{
				external_amplifier_mode(intercom_spk_mode);
				printk(KERN_ERR "Runbo:set_intercom_spk_mode poweron as intercom_spk_mode.\n");
			}
			break;
		default:
			break;
	}
}
EXPORT_SYMBOL(set_intercom_spk_mode);

//switch status when headset plug in or plug out. spk_mode ro headset mode.
unsigned int intercom_audio_switch(int audio_channel)
{
	intercom_audio_mode = audio_channel;
	if (intercom_using_flag)
	{
		set_intercom_spk_mode(sq_status);
		set_intercom_mic_channel(intercom_using_flag);
	}
	printk(KERN_ERR "Runbo:intercom_audio_mode = %d.\n",audio_channel);
	return 0;
}
EXPORT_SYMBOL(intercom_audio_switch);

//switch intercom_spk_mode between silent/L/M/H mode.
unsigned int switch_intercom_spk_mode(int switch_spk_mode)
{
	intercom_spk_mode = switch_spk_mode;
	if (intercom_using_flag)
	{
		set_intercom_spk_mode(sq_status);
	}
	printk(KERN_ERR "Runbo:switch_intercom_spk_mode.\n");
	return 0;
}
EXPORT_SYMBOL(switch_intercom_spk_mode);

void set_system_spk_mode(int spk_en)
{
	if (spk_en == 1)
		spk_status |= SYSTEM_SPK_ON;
	else
		spk_status &= SYSTEM_SPK_OFF;
		
	switch (spk_en)
	{
		case POWER_OFF:
			//both system and intercom spk turn off,so turn off speaker.
			if (spk_status == INTERCOM_AND_SYS_SPK_OFF)
			{
				external_amplifier_mode(SPK_OFF_MODE);
				printk(KERN_ERR "Runbo:set_system_spk_mode poweroff.\n");
			}
			//when turn off system spk,the intercom spk still running,so switch spk to intercom spk mode.
			else
			{
				set_intercom_spk_mode(sq_status);
				printk(KERN_ERR "Runbo:set_system_spk_mode resume intercom spk mode.\n");
			}
			break;
		case POWER_ON:
			external_amplifier_mode(SYSTEM_SPK_DEFAULT);
			printk(KERN_ERR "Runbo:set_system_spk_mode poweron as SYSTEM_SPK_DEFAULT.\n");
			break;
		default:
			break;
	}
}
EXPORT_SYMBOL(set_system_spk_mode);

int get_intercom_spk_mode(void)
{
	return intercom_spk_mode;
}
EXPORT_SYMBOL(get_intercom_spk_mode);

MODULE_DESCRIPTION("Runbo audio policy driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL V2");

