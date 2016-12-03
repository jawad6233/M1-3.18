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
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>  
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/irq.h>
#include <asm/scatterlist.h>
#include <mach/irqs.h>
#include <linux/slab.h>
#include "linux/delay.h"
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#define GPIO_DIR_OUT                    1
#define PULL_UP                         1
#define PULL_DOWN                       0
#define GPIO_MODE_GPIO                  GPIO_MODE_00
#define INTERCOM_DEVICE_NAME	        "intercom_device"
#define PTT_PIN_RELEASE                 0
#define PTT_PIN_PRESS                   1
#define PTT_PIN_DEFAULT                 PTT_PIN_RELEASE 
#define SQ_PIN_RELEASE                  0
#define SQ_PIN_PRESS                    1

#define INTERCOM_SPK_OFF                0
#define INTERCOM_SPK_ON                 1
#define	POWER_OFF                       0
#define POWER_ON                        1
#define HEADSET_MODE                    0
#define SPEAKER_MODE                    1
#define GET_AUDIO_MODE                  2
#define INTERCOM_BT_PTT_PRESS           1
#define INTERCOM_BT_PTT_RELEASE         0
#define INTERCOM_BT_MODE_ON             1
#define INTERCOM_BT_MODE_OFF            0
#define INTERCOM_SPK_SILENT_MODE        0
#define INTERCOM_SPK_L_MODE             1
#define INTERCOM_SPK_M_MODE             2
#define INTERCOM_SPK_H_MODE             3
#define INTERCOM_SQ_EINT_PIN            7
#define INTERCOM_PTT_EINT_PIN           119

#define INTERCOM_IOC_MAGIC              'I'
#define INTERCOM_IOC_POWER_CONTRL       _IO(INTERCOM_IOC_MAGIC, 0)
#define INTERCOM_IOC_AUDIO_MODE         _IO(INTERCOM_IOC_MAGIC, 1)
#define INTERCOM_IOC_BT_PTT_MODE        _IO(INTERCOM_IOC_MAGIC, 2)
#define INTERCOM_IOC_GET_VERSION        _IO(INTERCOM_IOC_MAGIC, 3)
#define INTERCOM_IOC_LED_INDICATION     _IO(INTERCOM_IOC_MAGIC, 4)
#define INTERCOM_IOC_BT_MODE            _IO(INTERCOM_IOC_MAGIC, 5)
#define INTERCOM_IOC_SPK_MODE           _IO(INTERCOM_IOC_MAGIC, 6)
#define MAX_INTERCOM_IOCTL_CMD_NUM      20

#define NLED_OFF 0
#define NLED_ON 1
enum mt65xx_led_pmic {
    MT65XX_LED_PMIC_LCD_ISINK = 0,
    MT65XX_LED_PMIC_NLED_ISINK0,
    MT65XX_LED_PMIC_NLED_ISINK1,
    MT65XX_LED_PMIC_NLED_ISINK2,
    MT65XX_LED_PMIC_NLED_ISINK3
};
extern int mt_brightness_set_pmic(enum mt65xx_led_pmic pmic_type, u32 level, u32 div);

struct intercom_android_dev 
{  
	int used;
	struct mutex sem;  
	struct cdev dev;  
};
static struct intercom_android_dev* intercom_dev = NULL;
static struct class *intercom_class;
static int intercom_major = 0;
static int intercom_minor = 0;
volatile int intercom_using_flag = 0;
struct pinctrl *intercom_pinctrl;
struct pinctrl_state *intercom_pins_default;
struct pinctrl_state *uarttx,*uartrx,*pdn_output0,*pdn_output1,*bt_ptt_output0,*bt_ptt_output1,*sq_as_eint,*ptt_as_eint;

static int pttkey_status = PTT_PIN_RELEASE;
static struct workqueue_struct *pttkey_eint_workqueue = NULL;
static struct workqueue_struct *sq_eint_workqueue = NULL;
static struct workqueue_struct *timer_workqueue = NULL;
static struct timer_list ptt_tx_end_timer;
static unsigned int irq_ptt_num;
static unsigned int irq_sq_num;
volatile int sq_status = SQ_PIN_RELEASE;

extern struct input_dev *kpd_input_dev;
static int intercom_bt_mode_open = 0;
extern volatile unsigned int intercom_audio_mode; 
static int register_pttkey_eint(void);
static irqreturn_t pttkey_eint_func(int irq,void *desc);
static int register_sq_eint(void);
static irqreturn_t sq_eint_func(int irq,void *desc);

static int intercom_setup_dev(struct intercom_android_dev* dev);
static long intercom_ioctl(struct file *file,unsigned int cmd,unsigned long arg);
static int intercom_probe(struct platform_device *dev);
static int intercom_suspend(struct platform_device *dev, pm_message_t state);
static int intercom_resume(struct platform_device *dev);
static int intercom_remove(struct platform_device *dev);
static int intercom_open(struct inode* inode, struct file* filp);
static int intercom_release(struct inode* inode, struct file* filp);
static void sq_eint_work_callback(struct work_struct *work);
static void pttkey_eint_work_callback(struct work_struct *work);
static void timer_work_callback(struct work_struct *work);
static int intercom_get_gpio_info(struct platform_device *pdev);
extern int audio_get_gpio_info(void *pdev);
extern unsigned int intercom_audio_switch(int audio_channel);
extern unsigned int switch_intercom_spk_mode(int switch_spk_mode);
extern void set_intercom_spk_mode(int spk_en);
extern void set_intercom_mic_channel(int mic_switch_en);
void intercom_ptt_tx_start(void);
void intercom_ptt_tx_release(void);

struct of_device_id intercom_of_match[] = {
	{ .compatible = "mediatek,mt6735-intercom", },
	{},
};
struct of_device_id ptt_of_eint_match[] = {
	{ .compatible = "mediatek,INTERCOM_ptt-eint", },
	{},
};
struct of_device_id sq_of_eint_match[] = {
	{ .compatible = "mediatek,INTERCOM_sq-eint", },
	{},
};


DECLARE_WORK(pttkey_eint_work, pttkey_eint_work_callback);
DECLARE_WORK(sq_eint_work, sq_eint_work_callback);
DECLARE_WORK(timer_work, timer_work_callback);

static int intercom_get_gpio_info(struct platform_device *pdev)
{
	int ret = 0;
	intercom_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(intercom_pinctrl)) {
        ret = PTR_ERR(intercom_pinctrl);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl!\n");
        return ret;
    }

	intercom_pins_default = pinctrl_lookup_state(intercom_pinctrl, "intercom_default");
    if (IS_ERR(intercom_pins_default)) {
        ret = PTR_ERR(intercom_pins_default);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl default %d!\n", ret);
    }

	pdn_output0 = pinctrl_lookup_state(intercom_pinctrl, "intercom_pdn_output0");
    if (IS_ERR(pdn_output0)) {
        ret = PTR_ERR(pdn_output0);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl pdn_output0 %d!\n", ret);
    }
	pdn_output1 = pinctrl_lookup_state(intercom_pinctrl, "intercom_pdn_output1");
    if (IS_ERR(pdn_output1)) {
        ret = PTR_ERR(pdn_output1);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl pdn_output1 %d!\n", ret);
    }

	bt_ptt_output0 = pinctrl_lookup_state(intercom_pinctrl, "intercom_bt_ptt_output0");
    if (IS_ERR(bt_ptt_output0)) {
        ret = PTR_ERR(bt_ptt_output0);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl bt_ptt_output0 %d!\n", ret);
    }
	bt_ptt_output1 = pinctrl_lookup_state(intercom_pinctrl, "intercom_bt_ptt_output1");
    if (IS_ERR(bt_ptt_output1)) {
        ret = PTR_ERR(bt_ptt_output1);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl bt_ptt_output1 %d!\n", ret);
    }

	sq_as_eint = pinctrl_lookup_state(intercom_pinctrl, "intercom_sq_as_eint");
    if (IS_ERR(sq_as_eint)) {
        ret = PTR_ERR(sq_as_eint);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl sq_as_eint %d!\n", ret);
    }
	
	ptt_as_eint = pinctrl_lookup_state(intercom_pinctrl, "intercom_ptt_as_eint");
    if (IS_ERR(ptt_as_eint)) {
        ret = PTR_ERR(ptt_as_eint);
        printk(KERN_ERR "Runbo Cannot find intercom pinctrl ptt_as_eint %d!\n", ret);
    }
	
	pinctrl_select_state(intercom_pinctrl, pdn_output0);
	pinctrl_select_state(intercom_pinctrl, bt_ptt_output0);
	pinctrl_select_state(intercom_pinctrl, sq_as_eint);
	pinctrl_select_state(intercom_pinctrl, ptt_as_eint);
	
	return 0;
}


static void timer_work_callback(struct work_struct *work)
{
	//Disable Main_MICBIAS0
	set_intercom_mic_channel(POWER_OFF);

}

static void ptt_tx_end_function(unsigned long data)
{
	queue_work(timer_workqueue, &timer_work);
}

static inline int intercom_ptt_tx_end_setup_timer(void)
{
	init_timer(&ptt_tx_end_timer);
	ptt_tx_end_timer.function = ptt_tx_end_function;
	ptt_tx_end_timer.expires  = jiffies + 1*HZ;;
	add_timer(&ptt_tx_end_timer);
	return 0;
}

static int register_pttkey_eint(void)
{
	struct device_node *ptt_node;
	ptt_node = of_find_matching_node(NULL,ptt_of_eint_match);
	pinctrl_select_state(intercom_pinctrl, ptt_as_eint);
	msleep(50);
	if (ptt_node)
	{
		irq_ptt_num = irq_of_parse_and_map(ptt_node,0);
		if (!irq_ptt_num)
		{
			printk(KERN_ERR "irq_of_parse_and_map for can't get irq_ptt_num!\n");
			return -EINVAL;
		}
		//request irq for eint
		if (request_irq(irq_ptt_num,pttkey_eint_func,IRQ_TYPE_EDGE_BOTH,"INTERCOM_ptt-eint",NULL))
		{
			printk(KERN_ERR "EINT PTT IRQ line is not avaliable!\n");
			return -EINVAL;
		}
	}
	enable_irq(irq_ptt_num);
	printk("Init GPIO_INTERCOM_PTT_EINT_PIN: register_pttkey_eint().\n");
	return 0;
}

static irqreturn_t pttkey_eint_func(int irq,void *desc)
{
	queue_work(pttkey_eint_workqueue, &pttkey_eint_work);
	return IRQ_HANDLED;
}


static void pttkey_eint_work_callback(struct work_struct *work)
{
	int pttkey_eint_status = 0;
	disable_irq(irq_ptt_num);
	if (pttkey_status == PTT_PIN_RELEASE)
	{
		msleep(5);
		pttkey_eint_status = gpio_get_value(INTERCOM_PTT_EINT_PIN);
		printk("pttkey gpio_get_value(INTERCOM_PTT_EINT_PIN) = %d.\n", pttkey_eint_status);
		if (!pttkey_eint_status)
		{
			pttkey_status = PTT_PIN_PRESS;
			printk("Runbo:Run pttkey is Pressed.\n");
			mt_brightness_set_pmic(MT65XX_LED_PMIC_NLED_ISINK1,NLED_OFF,0);
            input_report_key(kpd_input_dev, KEY_PTT, 1);
            input_sync(kpd_input_dev);
			if (intercom_using_flag == 1)
			{
				del_timer(&ptt_tx_end_timer);
				//Enable Main_MICBIAS0
				if (!intercom_bt_mode_open)
					set_intercom_mic_channel(POWER_ON);
			}
			//KEY_PTT is pressed,change EINT to Rising mode.
			enable_irq(irq_ptt_num);
			return;
		}
		//If this eint was ignored,resume PTT EINT configuration.
		pttkey_status = PTT_PIN_RELEASE;
		enable_irq(irq_ptt_num);
	}
	else if (pttkey_status == PTT_PIN_PRESS)
	{
		msleep(5);
		pttkey_eint_status = gpio_get_value(INTERCOM_PTT_EINT_PIN);
		printk("pttkey gpio_get_value(INTERCOM_PTT_EINT_PIN) = %d.\n", pttkey_eint_status);
		if (pttkey_eint_status)
		{
			pttkey_status = PTT_PIN_RELEASE;
			printk("Runbo:Run pttkey is Released.\n");
            input_report_key(kpd_input_dev, KEY_PTT, 0);
            input_sync(kpd_input_dev);
			if (intercom_using_flag == 1)
			{
				mt_brightness_set_pmic(MT65XX_LED_PMIC_NLED_ISINK1,NLED_ON,0);
				intercom_ptt_tx_end_setup_timer();
			}
			//KEY_PTT is released,change EINT to Falling mode.
			enable_irq(irq_ptt_num);
			return;
		}
		//If this eint was ignored,resume PTT EINT configuration.
		pttkey_status = PTT_PIN_PRESS;
		enable_irq(irq_ptt_num);
	}
}


static int register_sq_eint(void)
{
	struct device_node *sq_node;
	sq_node = of_find_matching_node(NULL,sq_of_eint_match);
	if (sq_node)
	{
		irq_sq_num = irq_of_parse_and_map(sq_node,0);
		if (!irq_sq_num)
		{
			printk(KERN_ERR "irq_of_parse_and_map for can't get irq_sq_num!\n");
			return -EINVAL;
		}
		//request irq for eint
		if (request_irq(irq_sq_num,sq_eint_func,IRQ_TYPE_EDGE_BOTH,"INTERCOM_sq-eint",NULL))
		{
			printk(KERN_ERR "EINT SQ IRQ line is not avaliable!\n");
			return -EINVAL;
		}
	}
	//enable_irq(irq_sq_num);
	disable_irq(irq_sq_num);
	printk("Init GPIO_INTERCOM_SQ_EINT_PIN: register_sq_eint().\n");
	return 0;
}

static irqreturn_t sq_eint_func(int irq,void *desc)
{
	printk("sq_eint_func().\n");
	queue_work(sq_eint_workqueue, &sq_eint_work);
	return IRQ_HANDLED;
}

static void sq_eint_work_callback(struct work_struct *work)
{
	int sq_eint_status = 0;
	disable_irq(irq_sq_num);
	if (sq_status == SQ_PIN_RELEASE)
	{
		msleep(5);
		sq_eint_status = gpio_get_value(INTERCOM_SQ_EINT_PIN);
		printk("sq_pin gpio_get_value(INTERCOM_SQ_EINT_PIN) = %d.\n", sq_eint_status);
		if (!sq_eint_status)
		{
			sq_status = SQ_PIN_PRESS;
			printk("Runbo:sq is Pressed.\n");
			mt_brightness_set_pmic(MT65XX_LED_PMIC_NLED_ISINK1,NLED_OFF,0);
            input_report_key(kpd_input_dev, KEY_SQ, 1);
            input_sync(kpd_input_dev);
	        udelay(500);
			//enable speaker.
			if (!intercom_bt_mode_open)
				set_intercom_spk_mode(sq_status);
			//KEY_SQ is pressed,change EINT to Rising mode.
			enable_irq(irq_sq_num);
			return;
		}
		//If this eint was ignored,resume SQ EINT configuration.
		sq_status = SQ_PIN_RELEASE;
		enable_irq(irq_sq_num);
	}
	else if (sq_status == SQ_PIN_PRESS)
	{
		msleep(5);
		sq_eint_status = gpio_get_value(INTERCOM_SQ_EINT_PIN);
		printk("sq_pin gpio_get_value(INTERCOM_SQ_EINT_PIN) = %d.\n", sq_eint_status);
		if (sq_eint_status)
		{
			sq_status = SQ_PIN_RELEASE;
			printk("Runbo:sq is Released.\n");
			mt_brightness_set_pmic(MT65XX_LED_PMIC_NLED_ISINK1,NLED_ON,0);
            input_report_key(kpd_input_dev, KEY_SQ, 0);
            input_sync(kpd_input_dev);
	        udelay(500);
			set_intercom_spk_mode(sq_status);
			//KEY_SQ is pressed,change EINT to Falling mode.
			enable_irq(irq_sq_num);
			return;
		}
		//If this eint was ignored,resume SQ EINT configuration.
		sq_status = SQ_PIN_PRESS;
		enable_irq(irq_sq_num);
	}
}

//zougx add 20160913 start
int get_intercom_sq_status(void)
{
	return sq_status;
}
EXPORT_SYMBOL(get_intercom_sq_status);
int get_intercom_using_flag(void)
{
	return intercom_using_flag;
}
EXPORT_SYMBOL(get_intercom_using_flag);
//zougx add 20160913 end
void intercom_ptt_tx_start(void)
{
	pinctrl_select_state(intercom_pinctrl, bt_ptt_output1);
}

void intercom_ptt_tx_release(void)
{
	pinctrl_select_state(intercom_pinctrl, bt_ptt_output0);
}

int get_intercom_bt_mode(void)
{
	return intercom_bt_mode_open;
}
EXPORT_SYMBOL(get_intercom_bt_mode);


static struct file_operations intercom_fops =
{
	.owner = THIS_MODULE,
	.open = intercom_open,
	.release = intercom_release,
	.unlocked_ioctl = intercom_ioctl,
};



static struct platform_driver intercom_driver =
{
	.probe = intercom_probe,
	.suspend = intercom_suspend,
	.resume = intercom_resume,
	.remove = intercom_remove,
	.driver = 
	{
		.name = "intercom_driver",
		.of_match_table = intercom_of_match,
	},
};

static struct platform_device intercom_device =
{
	.name ="intercom_driver",
	.id = -1,
};

static long intercom_ioctl(struct file *file,unsigned int cmd,unsigned long arg)
{
	long err = 0;
	int arg_value = 0;
	void __user *ptr;

	mutex_lock(&intercom_dev->sem);
	ptr = (void __user*) arg;
	get_user(arg_value,(int *)(&ptr));
	mutex_unlock(&intercom_dev->sem);
	printk(KERN_ERR "Runbo:call intercom_ioctl!cmd = %d,arg = %d ,arg_value = %d. \n",cmd,(unsigned int)arg,arg_value);
	if (_IOC_TYPE(cmd) != INTERCOM_IOC_MAGIC)
	{
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > MAX_INTERCOM_IOCTL_CMD_NUM )
	{
		return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	}
   	else if (_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err =  !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	}
	if (err)
	{
		printk(KERN_ERR "Runbo:intercom_ioctl arg error!!\n");
		return -EFAULT;
	}	
	switch (cmd)
	{
		case INTERCOM_IOC_POWER_CONTRL:
			if (arg == POWER_OFF)
			{
				printk(KERN_ERR "Runbo:Power_Down intercom!\n");
				disable_irq(irq_sq_num);
				sq_status = SQ_PIN_RELEASE;
				intercom_audio_switch(intercom_audio_mode);
				pinctrl_select_state(intercom_pinctrl, pdn_output0);
				intercom_using_flag = 0;
				mt_brightness_set_pmic(MT65XX_LED_PMIC_NLED_ISINK1,NLED_OFF,0);
			}
			else if (arg == POWER_ON)
			{
				printk(KERN_ERR "Runbo:Power_Up intercom!\n");
				intercom_using_flag = 1;
				sq_status = SQ_PIN_RELEASE;
				pinctrl_select_state(intercom_pinctrl, pdn_output1);
				intercom_audio_switch(intercom_audio_mode);
				//register_sq_eint();
				enable_irq(irq_sq_num);
				mt_brightness_set_pmic(MT65XX_LED_PMIC_NLED_ISINK1,NLED_ON,0);
			}
			break;
		
		case INTERCOM_IOC_AUDIO_MODE:
			//arg = SPEAKER_MODE / HEADSET_MODE
			intercom_audio_mode = arg;
			intercom_audio_switch(intercom_audio_mode);
			/*
			else if (arg == GET_AUDIO_MODE)
			{
				return intercom_audio_mode;
			}
			*/
			break;

		case INTERCOM_IOC_BT_PTT_MODE:
			if (arg == INTERCOM_BT_PTT_PRESS)
			{
				intercom_ptt_tx_start();
				printk(KERN_ERR "Runbo:Intercom PTT TX start!\n");
			}
			else if (arg == INTERCOM_BT_PTT_RELEASE)
			{
				intercom_ptt_tx_release();
				printk(KERN_ERR "Runbo:Intercom PTT TX release!\n");
			}
			break;
		case INTERCOM_IOC_BT_MODE:
			if (arg == INTERCOM_BT_MODE_ON)
			{
				intercom_bt_mode_open = 1;
				set_intercom_spk_mode(POWER_OFF); //disable speaker.
				printk(KERN_ERR "Runbo:Intercom enable BT mode!\n");
			}
			else if (arg == INTERCOM_BT_MODE_OFF)
			{
				intercom_bt_mode_open = 0;
				if((sq_status == SQ_PIN_PRESS) && (intercom_using_flag == 1))
				{
					set_intercom_spk_mode(POWER_ON);
				}
				printk(KERN_ERR "Runbo:Intercom disable BT mode!\n");
			}
			break;
		case INTERCOM_IOC_SPK_MODE:
			switch (arg)
			{
				case INTERCOM_SPK_SILENT_MODE:
				case INTERCOM_SPK_L_MODE:
				case INTERCOM_SPK_M_MODE:
				case INTERCOM_SPK_H_MODE:
				switch_intercom_spk_mode(arg);
				break;
			}
			
			break;

		default:
			break;
	}
	return 0;
}

static int intercom_setup_dev(struct intercom_android_dev* dev)
{
	int err;
	dev_t devno = MKDEV(intercom_major, intercom_minor);
	memset(dev, 0, sizeof(struct intercom_android_dev));
	cdev_init(&(dev->dev), &intercom_fops);
	dev->dev.owner = THIS_MODULE;
	dev->dev.ops = &intercom_fops;
	err = cdev_add(&(dev->dev),devno, 1);
	if (err)
	{
		return err;
	}
	dev->used = 0;
	mutex_init(&dev->sem);
	return 0;
}

static int intercom_probe(struct platform_device *pdev)
{
	int err = -1;
	dev_t tmp_dev = 0;
	struct device* temp = NULL;
	printk(KERN_ERR "Runbo:running probe function.\n");
	if (intercom_major)
	{
		tmp_dev = MKDEV(intercom_major, intercom_minor);
		err = register_chrdev_region(tmp_dev, 1, INTERCOM_DEVICE_NAME);
	}
	else
    {
		err = alloc_chrdev_region(&tmp_dev, 0, 1, INTERCOM_DEVICE_NAME);
		intercom_major = MAJOR(tmp_dev);
		intercom_minor = MINOR(tmp_dev);
	}
	if(err < 0)
	{
		printk(KERN_ERR "Runbo: register_chrdev err=%d\n", err);
 		return -EIO;
	}
	intercom_dev = kmalloc(sizeof(struct intercom_android_dev), GFP_KERNEL);
	if(!intercom_dev)
	{
		err = -ENOMEM;
		printk(KERN_ERR "Runbo:Failed to alloc intercom_dev./n");
		goto unregister;
	}
	err = intercom_setup_dev(intercom_dev);
	if(err)
	{
		printk(KERN_ERR "Runbo:Failed to setup dev: %d./n", err);
		goto cleanup;
	}
	intercom_class = class_create(THIS_MODULE, INTERCOM_DEVICE_NAME);
	if(IS_ERR(intercom_class))
	{
		err = PTR_ERR(intercom_class);
		printk(KERN_ERR "Runbo:Failed to create intercom class./n");
		goto destroy_cdev;
	}
	temp = device_create(intercom_class, NULL, tmp_dev, "%s", INTERCOM_DEVICE_NAME);
	if(IS_ERR(temp))
	{
		err = PTR_ERR(temp);
		printk(KERN_ERR "Runbo:Failed to create intercom device.");
		goto destroy_class;
	}
	dev_set_drvdata(temp, intercom_dev);
	printk(KERN_ERR "Runbo:Succedded to initialize intercom device./n");
	
	intercom_get_gpio_info(pdev);
	audio_get_gpio_info(&pdev->dev);
	
    pttkey_eint_workqueue = create_singlethread_workqueue("pttkey_eint");
	if (!pttkey_eint_workqueue)
	{
		printk(KERN_ERR "Runbo:Fail to create pttkey_eint_workqueue./n");
	}
    sq_eint_workqueue = create_singlethread_workqueue("sq_eint");
	if (!sq_eint_workqueue)
	{
		printk(KERN_ERR "Runbo:Fail to create sq_eint_workqueue./n");
	}
    timer_workqueue = create_singlethread_workqueue("ptt_tx_end_timer");
	if (!timer_workqueue)
	{
		printk(KERN_ERR "Runbo:Fail to create timer_workqueue./n");
	}
	register_pttkey_eint();
	register_sq_eint();
	return 0;

destroy_class:
	class_destroy(intercom_class);
destroy_cdev:
	cdev_del(&(intercom_dev->dev));
cleanup:
	kfree(intercom_dev);
unregister:
	unregister_chrdev_region(MKDEV(intercom_major, intercom_minor), 1);
	return 0;
}

static int intercom_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int intercom_resume(struct platform_device *dev)
{
	return 0;
}

static int intercom_remove(struct platform_device *dev)
{
	dev_t devno = MKDEV(intercom_major, intercom_minor);
	printk(KERN_ERR "Runbo:Destroy intercom device./n");
	if(intercom_class)
	{
		device_destroy(intercom_class, MKDEV(intercom_major, intercom_minor));
		class_destroy(intercom_class);
	}
	if(intercom_dev)
	{
		cdev_del(&intercom_dev->dev);
		kfree(intercom_dev);
	}
	unregister_chrdev_region(devno, 1);
	destroy_workqueue(pttkey_eint_workqueue);
	destroy_workqueue(sq_eint_workqueue);
	destroy_workqueue(timer_workqueue);
	return 0;
}

static int intercom_open(struct inode* inode, struct file* filp)
{
	mutex_lock(&intercom_dev->sem);
	intercom_dev->used++;
	mutex_unlock(&intercom_dev->sem);
	printk(KERN_ERR "Runbo:intercom_open contuer intercom_dev->used = %d\n",intercom_dev->used);
	return 0;
}

static int intercom_release(struct inode* inode, struct file* filp)
{
	mutex_lock(&intercom_dev->sem);
	if(intercom_dev->used != 0)
	intercom_dev->used--;
	mutex_unlock(&intercom_dev->sem);
	printk(KERN_ERR "Runbo:intercom_release contuer intercom_dev->used = %d\n",intercom_dev->used);
	return 0;
}


static int __init intercom_init(void)
{
	int ret = 0;
	ret = platform_device_register(&intercom_device);
	if (ret != 0)
	{
		printk(KERN_ERR "Runbo:platform_device_register error:(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&intercom_driver);
	if (ret)
	{
		printk(KERN_ERR "Runbo:platform_driver_register error:(%d)\n", ret);
		return ret;
	}
	printk(INTERCOM_DEVICE_NAME"initialized.\n");
	return 0;
}

static void __exit intercom_exit(void)
{
	platform_driver_unregister(&intercom_driver);
	platform_device_unregister(&intercom_device);
	printk(INTERCOM_DEVICE_NAME"release.\n");
}

module_init(intercom_init);
module_exit(intercom_exit);
MODULE_DESCRIPTION("Runbo intercom driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL V2");

