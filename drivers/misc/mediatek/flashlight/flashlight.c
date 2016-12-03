//
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <mach/upmu_sw.h>
#include "kd_flashlight.h"
#include <mach/mt_pbm.h>

#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>



/******************************************************************************
 * Definition
******************************************************************************/

/* device name and major number */
#define FLASHLIGHT_DEVNAME            "kd_camera_flashlight"

/***************************************************/
static int strobe_Res;
static int g_timeOutTimeMs;
static struct hrtimer g_timeOutTimer;
static struct work_struct workTimeOut;
static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */
struct pinctrl *flashlight_pinctrl = NULL;
struct pinctrl_state *flashlight_default= NULL;
struct pinctrl_state *flash_mode_h= NULL;
struct pinctrl_state *flash_mode_l= NULL;
struct pinctrl_state *torch_mode_h= NULL;
struct pinctrl_state *torch_mode_l= NULL;
struct of_device_id flashlight_of_match[] = {
    { .compatible = "mediatek,mt6735-flashlight", },
    {},
};

static void work_timeOutFunc(struct work_struct *data);

/* ============================== */
/* functions */
/* ============================== */
enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static int flashlight_get_gpio_info(struct platform_device *pdev)
{
    int ret = 0;
	printk("flashlight_get_gpio_info Start ^^ \n");
    flashlight_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flashlight_pinctrl )) {
		ret = PTR_ERR(flashlight_pinctrl );
		printk("%s : pinctrl err, flashlight_pinctrl \n", __func__);
		return ret;
	}

	flashlight_default= pinctrl_lookup_state(flashlight_pinctrl, "default");
	if (IS_ERR(flashlight_default)) {
		ret = PTR_ERR(flashlight_default);
		printk("%s : pinctrl err, flashlight_default\n", __func__);
	}

	flash_mode_l= pinctrl_lookup_state(flashlight_pinctrl, "flash_mode0");
	if (IS_ERR(flash_mode_l)) {
		ret = PTR_ERR(flash_mode_l);
		printk("%s : pinctrl err, flashlight_mode_l\n", __func__);
//		return ret;
	}

	flash_mode_h= pinctrl_lookup_state(flashlight_pinctrl, "flash_mode1");
	if (IS_ERR(flash_mode_h)) {
		ret = PTR_ERR(flash_mode_h);
		printk("%s : pinctrl err, flashlight_mode_h\n", __func__);
//		return ret;
	}

	torch_mode_l= pinctrl_lookup_state(flashlight_pinctrl, "torch_mode0");
	if (IS_ERR(torch_mode_l)) {
		ret = PTR_ERR(torch_mode_l);
		printk("%s : pinctrl err, torch_mode_l\n", __func__);
//		return ret;
	}

	torch_mode_h= pinctrl_lookup_state(flashlight_pinctrl, "torch_mode1");
	if (IS_ERR(torch_mode_h)) {
		ret = PTR_ERR(torch_mode_h);
		printk("%s : pinctrl err, torch_mode_h\n", __func__);
//		return ret;
	}

	printk("flashlight_get_gpio_info Done\n");
	return ret;
}

int FL_Init(void) {
	//INIT_WORK(&workTimeOut, work_timeOutFunc);
    return 0;
}

static void FL_Enable(void)
{
	pinctrl_select_state(flashlight_pinctrl, torch_mode_h);
//	pinctrl_select_state(flashlight_pinctrl, flash_mode_l);
	printk ("flashlight turn on \n");
}

static void FL_Disable(void)
{
	pinctrl_select_state(flashlight_pinctrl, torch_mode_l);
	pinctrl_select_state(flashlight_pinctrl, flash_mode_l);
	printk ("flashlight turn off \n");
}

int FL_dim_duty(int duty)
{
	printk ("flashlight FL_dim_duty %d\n",duty);
	if(duty ){
		pinctrl_select_state(flashlight_pinctrl, flash_mode_h);
	}
	else{
		pinctrl_select_state(flashlight_pinctrl, flash_mode_l);
		pinctrl_select_state(flashlight_pinctrl, torch_mode_h);
	}
	return 0;
}


static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	printk ("ledTimeOut_callback\n");
}

void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}
static int constant_flashlight_open(void)
{
    int i4RetValue = 0;
    printk("constant_flashlight_open line=%d\n", __LINE__);
	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	printk("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);

    if(strobe_Res)
    {
        printk(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }

    spin_unlock_irq(&g_strobeSMPLock);
    printk("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;
}

static int constant_flashlight_release(void)
 {
     printk(" constant_flashlight_release\n");
     if (strobe_Res)
     {
         spin_lock_irq(&g_strobeSMPLock);
 
         strobe_Res = 0;

         spin_unlock_irq(&g_strobeSMPLock);
 
		 FL_Disable();
         //FL_Uninit();
     }
 
     printk(" Done\n");
     return 0;
 
 }



/***************************************/

static long flashlight_ioctl_core(struct file *file, unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	kdStrobeDrvArg kdArg;
	unsigned long copyRet;

	printk ("flashlight flashlight_ioctl_core cmd: 0x%x ,arg:0x%lx\n",cmd,arg);

	copyRet = copy_from_user(&kdArg, (void *)arg, sizeof(kdStrobeDrvArg));
	printk("flashlight_ioctl cmd=0x%x(nr=%d), senorDev=0x%x ledId=0x%x arg=0x%lx\n", cmd,
	     _IOC_NR(cmd), kdArg.sensorDev, kdArg.strobeId, (unsigned long)kdArg.arg);


	switch (cmd) {
	case FLASH_IOC_GET_PROTOCOL_VERSION:
		i4RetValue = 1;
		break;
	case FLASH_IOC_IS_LOW_POWER: //182
		printk("FLASH_IOC_IS_LOW_POWER");
		break;

	case FLASH_IOC_LOW_POWER_DETECT_START:
		printk("FLASH_IOC_LOW_POWER_DETECT_START");
		break;

	case FLASH_IOC_LOW_POWER_DETECT_END:
		printk("FLASH_IOC_LOW_POWER_DETECT_END");
		break;
	case FLASHLIGHTIOC_X_SET_DRIVER:
		constant_flashlight_open();
		break;
	case FLASH_IOC_GET_PART_ID:
	case FLASH_IOC_GET_MAIN_PART_ID:
	case FLASH_IOC_GET_SUB_PART_ID:
	case FLASH_IOC_GET_MAIN2_PART_ID:
		{
			int partId;
			partId = 1; 
			kdArg.arg = partId;
			if (copy_to_user
			    ((void __user *)arg, (void *)&kdArg, sizeof(kdStrobeDrvArg))) {
				printk("[FLASH_IOC_GET_PART_ID] ioctl copy to user failed ~\n");
				return -EFAULT;
			}
			printk("FLASH_IOC_GET_PART_ID line=%d partId=%d\n", __LINE__, partId);
		}
		break;
	case FLASH_IOC_SET_TIME_OUT_TIME_MS: //100
		printk ("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d ms\n", kdArg.arg);
		g_timeOutTimeMs = kdArg.arg;
		break;
	case FLASH_IOC_SET_DUTY: //110
		printk ("FLASHLIGHT_DUTY: 0x%x\n", kdArg.arg);
		FL_dim_duty(kdArg.arg);
		break;
	case FLASH_IOC_SET_ONOFF:  //115
		printk ("FLASH_IOC_SET_ONOFF 115 arg=0x%lx\n",(unsigned long)kdArg.arg);

		if (1 ==  (unsigned long )kdArg.arg ) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	case FLASH_IOC_UNINIT: //120
		constant_flashlight_release();
		break;
	case FLASH_IOC_GET_PRE_ON_TIME_MS:   //130
		printk ("FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY 130\n");
		break;
	case FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY: //131
		printk ("FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY 131\n");
		break;
	default:
		{
				printk("[default] function pointer is wrong ~\n");
		}
		break;
	}
	return i4RetValue;
}

#ifdef CONFIG_COMPAT
static long my_ioctl_compat(struct file *filep, unsigned int cmd, unsigned long arg)
{
    int err;
    /* int copyRet; */
    kdStrobeDrvArg *pUObj;

    printk("flash my_ioctl_compat2 line=%d cmd=0x%x arg=0x%lx\n", __LINE__, cmd, arg);
    pUObj = compat_ptr(arg);
    err = flashlight_ioctl_core(filep, cmd, (unsigned long)pUObj);

    return err;

}
#endif



static long flashlight_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err;
	printk ("flashlight flashlight_ioctl cmd: 0x%x ,arg:0x%lx\n",cmd,arg);
	err = flashlight_ioctl_core(file, cmd, arg);
	return err;
}

static int flashlight_open(struct inode *inode, struct file *file)
{
	int i4RetValue = 0;
	printk("[flashlight_open] E ~\n");
	//timerInit();
	printk("[flashlight_open] E ~\n");
	return i4RetValue;
}

static int flashlight_release(struct inode *inode, struct file *file)
{
	printk("[flashlight_release] E ~\n");
	return 0;
}

/* ======================================================================== */
/* ======================================================================== */
/* ======================================================================== */
/* Kernel interface */
static const struct file_operations flashlight_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = flashlight_ioctl,
	.open = flashlight_open,
	.release = flashlight_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = my_ioctl_compat,
#endif
};

/* ======================================================================== */
/* Driver interface */
/* ======================================================================== */
struct flashlight_data {
	spinlock_t lock;
	wait_queue_head_t read_wait;
	struct semaphore sem;
};

static struct class *flashlight_class;
static struct device *flashlight_device;
static struct flashlight_data flashlight_private;
static dev_t flashlight_devno;
static struct cdev flashlight_cdev;
/* ======================================================================== */

static int flashlight_probe(struct platform_device *pdev)
{
	int ret = 0, err = 0;

	early_printk("[flashlight_probe] start ~\n");

	ret = alloc_chrdev_region(&flashlight_devno, 0, 1, FLASHLIGHT_DEVNAME);
	if (ret) {
		printk("[flashlight_probe  iii ] alloc_chrdev_region fail: %d ~\n", ret);
		goto flashlight_probe_error;
	} else {
		printk("[flashlight_probe iii ] major: %d, minor: %d ~\n", MAJOR(flashlight_devno),
		     MINOR(flashlight_devno));
	}
	cdev_init(&flashlight_cdev, &flashlight_fops);
	flashlight_cdev.owner = THIS_MODULE;
	err = cdev_add(&flashlight_cdev, flashlight_devno, 1);
	if (err) {
		printk("[flashlight_probe] cdev_add fail: %d ~\n", err);
		goto flashlight_probe_error;
	}

	flashlight_class = class_create(THIS_MODULE, "flashlightdrv");
	if (IS_ERR(flashlight_class)) {
		printk("[flashlight_probe] Unable to create class, err = %d ~\n",
		     (int)PTR_ERR(flashlight_class));
		goto flashlight_probe_error;
	}

	flashlight_device =
	    device_create(flashlight_class, NULL, flashlight_devno, NULL, FLASHLIGHT_DEVNAME);
	if (NULL == flashlight_device) {
		printk("[flashlight_probe] device_create fail ~\n");
		goto flashlight_probe_error;
	}

	flashlight_get_gpio_info(pdev);
	/* initialize members */
	spin_lock_init(&flashlight_private.lock);
	init_waitqueue_head(&flashlight_private.read_wait);
	/* init_MUTEX(&flashlight_private.sem); */
	sema_init(&flashlight_private.sem, 1);


	printk("[flashlight_probe] Done ~\n");
	return 0;

flashlight_probe_error:
	if (err == 0)
		cdev_del(&flashlight_cdev);
	if (ret == 0)
		unregister_chrdev_region(flashlight_devno, 1);
	return -1;
}

static int flashlight_remove(struct platform_device *dev)
{

	printk("[flashlight_remove] start\n");
	cdev_del(&flashlight_cdev);
	unregister_chrdev_region(flashlight_devno, 1);
	device_destroy(flashlight_class, flashlight_devno);
	class_destroy(flashlight_class);

	printk("[flashlight_remove Done ~");
	return 0;
}

static void flashlight_shutdown(struct platform_device *dev)
{

	printk("[flashlight_shutdown] start\n");
	printk("[flashlight_shutdown] Done ~");
}

static struct platform_driver flashlight_platform_driver = {
	.probe = flashlight_probe,
	.remove = flashlight_remove,
	.shutdown = flashlight_shutdown,
	.driver = {
		   .name = FLASHLIGHT_DEVNAME,
		   .owner = THIS_MODULE,
		   .of_match_table = flashlight_of_match,
		   },
};

//static struct platform_device flashlight_platform_device = {
//	.name = FLASHLIGHT_DEVNAME,
//	.id = 0,
//	.dev = {
//		}
//};

static int __init flashlight_init(void)
{
	int ret = 0;

	printk("[flashlight_init iii ] start ~\n");

//	ret = platform_device_register(&flashlight_platform_device);
//	if (ret) {
//		printk("[flashlight_init iii ] platform_device_register fail ~\n");
//		return ret;
//	}

	ret = platform_driver_register(&flashlight_platform_driver);
	if (ret) {
		printk("[flashlight_init] platform_driver_register fail ~\n");
		return ret;
	}

	printk("[flashlight_init iii ] done! ~\n");
	return ret;
}

static void __exit flashlight_exit(void)
{
	printk("[flashlight_exit] start ~\n");
	platform_driver_unregister(&flashlight_platform_driver);
	/* to flush work queue */
	/* flush_scheduled_work(); */
	printk("[flashlight_exit] done! ~\n");
}

/* ======================================================== */
module_init(flashlight_init);
module_exit(flashlight_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ink <cwj@tintele.com>");
MODULE_DESCRIPTION("Flashlight control Driver");

