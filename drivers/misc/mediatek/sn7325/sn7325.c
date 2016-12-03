#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/irqreturn.h>
#include <linux/err.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <mach/mt_gpt.h>
#include <mach/irqs.h>
#include <linux/kthread.h>
#include <linux/of_irq.h>
#include <linux/timer.h>
#include <linux/platform_device.h>




extern struct input_dev *kpd_input_dev;

#define CONFIG_TINTELE_PROJECT_DT600
#define SN_RESET_PIN 72
#define SN_INTERUPT_PIN 20

struct pinctrl *sn_pinctrl;
struct pinctrl_state *sn_pins_default;
struct pinctrl_state *sn_eint_as_int, *sn_eint_output0, *sn_eint_output1, *sn_rst_output0, *sn_rst_output1;
//static int get_pin_flag = 0;


static int sn7325_set_rst_output(int level);
static int sn7325_set_interuput(int i);
int read_thread_func(void *arg);


struct sn7325_data sn7325;
struct workqueue_struct *sn7325_wq;
struct task_struct * my_thread = NULL;
struct task_struct * read_thread = NULL;


struct of_device_id sn7325_of_interruput_match[] = {
	{ .compatible = "mediatek,mt6735-sn7325", },
};


enum sn7325_cmd 
{
	SN7325_INPUT_A	= 0,
	SN7325_INPUT_B 	= 1,
	SN7325_OUTPUT_A = 2,
	SN7325_OUTPUT_B = 3,
	SN7325_CONFIG_A = 4,
	SN7325_CONFIG_B = 5,
	SN7325_INT_CONTROL_A = 6,
	SN7325_INT_CONTROL_B = 7,
};
enum sn7325_cmd mycmd;
struct sn7325_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	int enable;

};

static	void  sn7325_poll_work_func(struct work_struct *work)
{
   enable_irq(SN_INTERUPT_PIN);
   read_thread_func(NULL);
   disable_irq(SN_INTERUPT_PIN);
}

DECLARE_WORK(sn7325_work, sn7325_poll_work_func);

static irqreturn_t sn7325_interrupt(int irq, void * dev_id)
{
	enable_irq(SN_INTERUPT_PIN);
	queue_work(sn7325_wq, &sn7325_work);
	return IRQ_HANDLED;
}

u8 read_data ;
struct read_num
{
	u8 num;
	u8 repeat;
};

static u8 translateNum(int num)
{
	int a[] = {0xf0,0xb0,0x30,0x70,0x50,0x10,0x90,0xd0,0xc0,0x80,0x0,0x40,0x60,0x20,0xa0,0xe0};
	int i ;
	for(i = 0;i<16;i++)
	{
		if(a[i] == num )  
		return (i+1);			  
	}
	return (i+1);
}

static u8 vol_old = 0  ;
static u8 intercom_vol_old = 0  ;
static u8 chan_old = 0 ;

static DEFINE_MUTEX(sn7325_mutex);
//zougx add 20160904 start
#ifdef CONFIG_TINTELE_DEVICE_MODEL_M1
extern int get_intercom_using_flag(void);
extern int get_intercom_sq_status(void);
#endif
//zougx add 20160904 end

#if 1
int read_thread_func(void *arg)
{
    u8 data = 0;
	u8 tran_data = 0;
	
	u8 vol_new=0;
	#ifdef CONFIG_TINTELE_DEVICE_MODEL_M1
	u8 intercom_vol_new = 0;
	u8 chan_new=0;
	#endif
	mutex_lock(&sn7325_mutex);
	
	mycmd = SN7325_INPUT_B;
	data =  i2c_smbus_read_byte_data(sn7325.client, mycmd);
	tran_data  = translateNum(data & 0xF0); //DT600 
	#ifdef CONFIG_TINTELE_DEVICE_MODEL_M1
	if(get_intercom_using_flag()){
		if(get_intercom_sq_status()){
			// intercom vol ++
			intercom_vol_new = tran_data;
			printk("guixing -- intercom_vol_new=%d,intercom_vol_old=%d --\n",intercom_vol_new,intercom_vol_old);
			if(intercom_vol_new != intercom_vol_old){
				if(((intercom_vol_new >intercom_vol_old)  &&  ((intercom_vol_new >1) && (intercom_vol_new< 16)))  || ((intercom_vol_old == 16) &&  (intercom_vol_new == 1))  || ((intercom_vol_old == 15) &&  (intercom_vol_new == 16))){
					// intercom volume ++
					input_report_abs(sn7325.input_dev, ABS_BRAKE,   2 );
					input_report_abs(sn7325.input_dev, ABS_BRAKE, 0 );
					input_sync(sn7325.input_dev);
				}
				if(((intercom_vol_new < intercom_vol_old) && ((intercom_vol_old >1) && (intercom_vol_old< 16))) ||   ((intercom_vol_old == 1) &&(intercom_vol_new==16)) || ((intercom_vol_old == 16) &&(intercom_vol_new==15))){
					//intercom volume --
					input_report_abs(sn7325.input_dev, ABS_BRAKE,   1 );
				    input_report_abs(sn7325.input_dev, ABS_BRAKE, 0 );
		            input_sync(sn7325.input_dev);
				}
				intercom_vol_old = intercom_vol_new;
			}
		}else{
			//chan ++
			chan_new=tran_data;
			if(chan_new != chan_old )
			{
		    	input_report_abs(sn7325.input_dev, ABS_GAS, chan_new);
				chan_old = chan_new;
		    }
		}
		input_sync(sn7325.input_dev);
	}else{
	#endif
		vol_new = tran_data;
		if(vol_new != vol_old){
			printk("guixing -- vol_new=%d,vol_old=%d --\n",vol_new,vol_old);
			if(((vol_new >vol_old)  &&  ((vol_new >1) && (vol_new< 16)))  || ((vol_old == 16) &&  (vol_new == 1))  || ((vol_old == 15) &&  (vol_new == 16))){
				// system volume ++
				input_report_key(kpd_input_dev, KEY_VOLUMEUP, 1);
                input_sync(kpd_input_dev);
			    input_report_key(kpd_input_dev, KEY_VOLUMEUP, 0);
                input_sync(kpd_input_dev);
			}
			if(((vol_new < vol_old) && ((vol_old >1) && (vol_old< 16))) ||   ((vol_old == 1) &&(vol_new==16)) || ((vol_old == 16) &&(vol_new==15))){
				//system volume --
				input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 1);
                input_sync(kpd_input_dev);
			    input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 0);
                input_sync(kpd_input_dev);
			}
			vol_old = vol_new;
		}
	#ifdef CONFIG_TINTELE_DEVICE_MODEL_M1
	}
	#endif
	//printk("guixing report chan_new=%d, chan_old=%d \n",chan_new,chan_old);
	//chan_old = chan_new;
	mutex_unlock(&sn7325_mutex);
	printk(KERN_ERR "guixing sn7325 PORTA data = 0x%x \n",data);
	return 0 ;
}
	

#else
int read_thread_func(void *arg)
{
    u8 data = 0;
	int err =1;
	
	u8 vol_new=0,chan_new=0;
	mutex_lock(&sn7325_mutex);
	get_pin_flag++;
	mycmd = SN7325_INPUT_B;
	err = 1;
	data =  i2c_smbus_read_byte_data(sn7325.client, mycmd);
	//get_pin_flag++;
	read_data = data;//for  read_thread for test 
	#ifdef CONFIG_TINTELE_PROJECT_DT600
	vol_new  = translateNum(data & 0xF0); //DT600 
	chan_new = translateNum(data & 0xF0);
    #endif
	if(vol_new != vol_old)
		{         
			printk("guixing intercom status is %d\n",get_intercom_using_flag());
          if(  (  (vol_new >vol_old)  &&  ( (vol_new >1) && (vol_new< 16)   ) )  || ( (vol_old == 16 ) &&  (vol_new == 1) )  || ( (vol_old == 15 ) &&  (vol_new == 16) ) )
		    {
			  if( get_intercom_using_flag()  )
			  {	  
				input_report_abs(sn7325.input_dev, ABS_BRAKE,   2 );
				printk("vol_add++++++++\n");

				input_report_abs(sn7325.input_dev, ABS_BRAKE, 0 );
				input_sync(sn7325.input_dev);
			  }
			  else
			  {
			    input_report_key(kpd_input_dev, KEY_VOLUMEUP, 1);
                input_sync(kpd_input_dev);
			    input_report_key(kpd_input_dev, KEY_VOLUMEUP, 0);
                input_sync(kpd_input_dev);
			  }
		    }
          if( ( (vol_new < vol_old) && ( (vol_old >1) && (vol_old< 16) ) ) ||   ((vol_old == 1) &&(vol_new==16)) ||   ((vol_old == 16) &&(vol_new==15)) )
		  {
			  if( get_intercom_using_flag() )
			  {
                input_report_abs(sn7325.input_dev, ABS_BRAKE,   1 );
			    printk("vol_add----\n");
			    input_report_abs(sn7325.input_dev, ABS_BRAKE, 0 );
	            input_sync(sn7325.input_dev);
			  }
			  else
			  {
			    input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 1);
                input_sync(kpd_input_dev);
			    input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 0);
                input_sync(kpd_input_dev);
			  }
		    }
	    }
	printk("guixing report chan_new=%d, chan_old=%d \n",chan_new,chan_old);
	if(get_intercom_using_flag()){
	    if(chan_new != chan_old )
		{
	    	input_report_abs(sn7325.input_dev, ABS_GAS, chan_new);
	    }
		chan_old = chan_new;
	}else{
    	vol_old = vol_new;
	}
	
	input_sync(sn7325.input_dev);
	mutex_unlock(&sn7325_mutex);
	printk(KERN_ERR "guixing sn7325 PORTA data = 0x%x and count = %d \n",data,get_pin_flag);
	return 0 ;
}
#endif
	
static int sn7325_register_interrupt(void)
{
	int ret;
	u8 data ;
	int err =1;
	int sn7325_irq = 0;
	struct device_node *sn7325_node;
	mycmd = SN7325_INT_CONTROL_B;
	data = 0x00;// SN7325_INT_CONTROL_B  on	
	err = i2c_smbus_write_byte_data(sn7325.client, mycmd, data);	
	if (err < 0)  {			   
		printk(KERN_ERR"sn7325_config_col()-->write sn7325 int_control_A register failed!   \n");
		return err;
	}   
	msleep(50);
	sn7325_set_interuput(1);
	msleep(50);
	sn7325_node = of_find_matching_node(NULL,sn7325_of_interruput_match);
	if(sn7325_node){
		
		sn7325_irq = irq_of_parse_and_map(sn7325_node,0);
		if(!sn7325_irq){
			printk("can't irq_of_parse_and_map for abc!!\n");
			return 0;
		}
		ret = request_irq(sn7325_irq,sn7325_interrupt,IRQF_TRIGGER_FALLING,"SN7325-eint",NULL);
	}
	
    enable_irq(sn7325_irq);
	return 0;
}
// zougx add 20160904 start
static void init_vol_chan(void)
{
	u8 data ;
	mycmd = SN7325_INPUT_B;
	data = -1;		
	data = i2c_smbus_read_byte_data(sn7325.client, mycmd); //读出porta的output
    vol_old  = translateNum(data & 0xF0); //DT900 
    chan_old  = translateNum(data & 0xF0);
	intercom_vol_old  = translateNum(data & 0xF0);
}
// zougx add 20160904 end 

void sn7325_reset(void)
{
	sn7325_set_rst_output(1);
	mdelay(5);
	sn7325_set_rst_output(0);
	mdelay(100);
	sn7325_set_rst_output(1);
	mdelay(100);
}

static int sn7325_init_chip(int i)
{
    int err;
	int data;
	struct i2c_client *client = sn7325.client;
	
	sn7325_reset();//reset the chip sn7325
	
	 //关闭中断A
	mycmd = SN7325_INT_CONTROL_A;
	err = 1; 
	data = 0xff;//SN7325_INT_CONTROL_A OFF
	err = i2c_smbus_write_byte_data(client, mycmd, data);
	if (err < 0)  {			 
		printk(KERN_ERR"sn7325_config_col()-->write sn7325 int_control_A register failed!	 \n");
		return err;
	 }	 
	 msleep(50);

	//关闭中断B
	mycmd = SN7325_INT_CONTROL_B;
	err = 1; 
	data = 0xff;// SN7325_INT_CONTROL_B  OFF	
	err = i2c_smbus_write_byte_data(client, mycmd, data);
	if (err < 0)  {				
		printk(KERN_ERR"sn7325_config_col()-->write sn7325 int_control_A register failed!	\n");
		return err;
	}	
	msleep(50);		

	//设置为输出
    mycmd = SN7325_CONFIG_A;
	err = 1; 
	data = 0x00;//
	err = i2c_smbus_write_byte_data(client, mycmd, data);
	if (err < 0)  {			
		printk(KERN_ERR"sn7325_config_col()-->write sn7325 int_control_A register failed!	\n");
		return err;
 	}	
	msleep(50);
	//设置为输出0xff
	mycmd = SN7325_OUTPUT_A;
	err = 1; 
	data = 0xff;//output 0xff
	err = i2c_smbus_write_byte_data(client, mycmd, data);
	if (err < 0)  {
		printk(KERN_ERR"sn7325_config_col()-->write sn7325 int_control_A register failed!	\n");
		return err;
	}	
	msleep(50);	
	/* Begin [chenxw, 2015-04-23 Add for aclohol */
	mycmd = SN7325_OUTPUT_A;
	err = 1;
	data = -1;
	data = i2c_smbus_read_byte_data(client, mycmd); //读出porta的output
	printk("chenxw off SN7325_OUTPUT_A*********** data = %d\n", data);
	if (data < 0) {
		printk(KERN_ERR"Read sn7325 OUTPUT_A register failed!\n");
		return data;
	}
	data &= 0xef;
	printk("chenxw off SN7325_OUTPUT_A*********** data = %d\n", data);
	err = i2c_smbus_write_byte_data(client, mycmd, data); //设置D4为低
	if (err < 0) {
		printk(KERN_ERR"write sn7325 OUTPUT_A register failed!	\n");
		return err;
	}
	/* end */

	/* Begin [chenxw, 2016-01-08 config pp0-pp3输入，pp3-pp7输出 */

	//配置为输入
	mycmd = SN7325_CONFIG_B;
	err = 1; 
	data = 0xff;//
	err = i2c_smbus_write_byte_data(client, mycmd, data);	
	if (err < 0)  {		
		printk(KERN_ERR"sn7325_config_col()-->write sn7325 int_control_A register failed!	\n");
		return err;
	}	
//end
	msleep(50);
	mycmd = SN7325_INPUT_B;
	data =  i2c_smbus_read_byte_data(client, mycmd);
	if (data < 0) {	
		printk(KERN_ERR"xiaopei sn7325 read PortB failed!	\n");	
		return 0;
	}	
	msleep(50);
    sn7325_register_interrupt();//register interrrupt
    init_vol_chan();
    //init_vol_chan();//init the value of vol and channel 
	return err;
}

static int sn7325_get_gpio_info(struct platform_device *pdev)
{
	int ret;
	printk("=== find sn7325 pinctrl ===\n");
	sn_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(sn_pinctrl)) {
		ret = PTR_ERR(sn_pinctrl);
		//printk("=== can't find sn7325 pinctrl ===\n");
		dev_err(&pdev->dev, "sn7325 Cannot find touch pinctrl1!\n");
		return ret;
	}
	sn_pins_default = pinctrl_lookup_state(sn_pinctrl, "sn_default");
	
	if (IS_ERR(sn_pins_default)) {
		ret = PTR_ERR(sn_pins_default);
		dev_err(&pdev->dev, "sn7325 Cannot find touch pinctrl default %d!\n", ret);
	}
	sn_eint_as_int = pinctrl_lookup_state(sn_pinctrl, "sn_state_eint_as_int");
	if (IS_ERR(sn_eint_as_int)) {
		ret = PTR_ERR(sn_eint_as_int);
		dev_err(&pdev->dev, "sn7325 Cannot find touch pinctrl state_eint_as_int!\n");
		return ret;
	}
	sn_eint_output0 = pinctrl_lookup_state(sn_pinctrl, "sn_state_eint_output0");
	if (IS_ERR(sn_eint_output0)) {
		ret = PTR_ERR(sn_eint_output0);
		dev_err(&pdev->dev, "sn7325 Cannot find touch pinctrl state_eint_output0!\n");
		return ret;
	}
	sn_eint_output1 = pinctrl_lookup_state(sn_pinctrl, "sn_state_eint_output1");
	if (IS_ERR(sn_eint_output1)) {
		ret = PTR_ERR(sn_eint_output1);
		dev_err(&pdev->dev, "sn7325 Cannot find touch pinctrl state_eint_output1!\n");
		return ret;
	}
	sn_rst_output0 = pinctrl_lookup_state(sn_pinctrl, "sn_state_rst_output0");
	if (IS_ERR(sn_rst_output0)) {
		ret = PTR_ERR(sn_rst_output0);
		dev_err(&pdev->dev, "sn7325 Cannot find touch pinctrl state_rst_output0!\n");
		return ret;
	}
	sn_rst_output1 = pinctrl_lookup_state(sn_pinctrl, "sn_state_rst_output1");
	if (IS_ERR(sn_rst_output1)) {
		ret = PTR_ERR(sn_rst_output1);
		dev_err(&pdev->dev, "sn7325 Cannot find touch pinctrl state_rst_output1!\n");
		return ret;
	}
	return 0;
}


static int sn7325_set_rst_output(int level)
{
	if (level)
		pinctrl_select_state(sn_pinctrl, sn_rst_output1);
	else
		pinctrl_select_state(sn_pinctrl, sn_rst_output0);
	return 0;
}

static int sn7325_set_interuput(int i)
{
	//return 0;
	if (i== 1)
		pinctrl_select_state(sn_pinctrl, sn_eint_as_int);
	return 0;
}

static int sn7325_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{			
	int err;
	sn7325.client = client;
    printk(KERN_DEBUG"Enter sn7325_probe...\n");
    msleep(10);    
	
	//sn7325_get_gpio_info(client);

    sn7325.input_dev= input_allocate_device();
	if (!sn7325.input_dev) {
		err = -ENODEV;
		printk(KERN_ERR
			"sn7325_probe: Failed to allocate input device\n");
		goto exit_gpioReset_request;
	}
	
	//sn7325_rst_output(1);
	sn7325_init_chip(1);	//SN7325 init function 
	
	set_bit(EV_ABS, sn7325.input_dev->evbit );
	set_bit(EV_SYN, sn7325.input_dev->evbit );
	input_set_abs_params( sn7325.input_dev,ABS_GAS,-1,20,0,0);
	input_set_abs_params( sn7325.input_dev,ABS_BRAKE,-1,20,0,0);
	sn7325.input_dev->name = "sn7325_event";
	err = input_register_device(sn7325.input_dev);
	if (err) {
		printk(KERN_ERR
			"sn7325_probe(): Failed to register a input device\n");
		goto exit_input_register_device_failed;
	}	
 return 0;

exit_input_register_device_failed:
	  
		 input_free_device(sn7325.input_dev);	//xiaopei add	
exit_gpioReset_request:

		return -1;
}

static int sn7325_remove(struct i2c_client *client)
{	
	input_unregister_device(sn7325.input_dev);
	return 0;
}

static	const struct i2c_device_id sn7325_id[] = {
	{"sn7325", 0},
	{},
};

static const struct of_device_id sn7325_match[]={
	{.compatible = "mediatek,sn7325"},
	{},
};

static struct i2c_driver sn7325_driver = {
	.driver = {
	    .name = "sn7325",
		.of_match_table = sn7325_match,
		.owner = THIS_MODULE,
	    
	},
	.id_table = sn7325_id,
	.probe	= sn7325_probe,
	.remove	= sn7325_remove,
	
};
static int sn7325_plat_probe(struct platform_device *pdev)
{
	int ret = 0;
	sn7325_get_gpio_info(pdev);

	mdelay(5);
	//return 0;
	sn7325_wq = create_singlethread_workqueue("sn7325");
	if (!sn7325_wq) {
		printk(KERN_ERR"failed to create sn7325_wq.\n");
		return -1;
	}  
	ret = i2c_add_driver(&sn7325_driver);
	if(ret !=0){
		printk(KERN_ERR"failed to i2c_add_sn7325_driver.\n");
		return -1;
	}
	return 0;
}
static int sn7325_plat_remove(struct platform_device *dev)
{
	printk("== sn7325_remove start==\n");
	i2c_del_driver(&sn7325_driver);	
	printk(KERN_DEBUG"sn7325_remove(): destroy_workqueue");
	destroy_workqueue(sn7325_wq);
	return 0;
}

struct of_device_id sn7325_of_match[] = {
	{ .compatible = "mediatek,common-sn7325", },
};

static struct platform_driver sn7325_plat_driver =
{
	.probe = sn7325_plat_probe,
	.remove = sn7325_plat_remove,
	.driver = 
	{
		.name = "sn7325_plat_driver",
		.of_match_table = sn7325_of_match,
	},
};

static int __init sn7325_init(void)
{
	int ret = 0;
	ret = platform_driver_register(&sn7325_plat_driver);
	if (ret)
	{
		printk(KERN_ERR "Runbo:platform_driver_register error:(%d)\n", ret);
		return ret;
	}
	return  0;
}

static	void __exit sn7325_exit(void)
{
	platform_driver_unregister(&sn7325_plat_driver);
}

module_init(sn7325_init);
module_exit(sn7325_exit);
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL V2");


#if 0
static int __init sn7325_init(void)
{
	int ret = 0;
	sn7325_wq = create_singlethread_workqueue("sn7325");
	if (!sn7325_wq) {
		printk(KERN_ERR"failed to create sn7325_wq.\n");
		return -1;
		}  
	ret = i2c_add_driver(&sn7325_driver);
	if(ret !=0){
		printk(KERN_ERR"failed to i2c_add_sn7325_driver.\n");
		return -1;
	}
	return  0;
}

static	void __exit sn7325_exit(void)
{
	i2c_del_driver(&sn7325_driver);	
	printk(KERN_DEBUG"sn7325_remove(): destroy_workqueue");
	destroy_workqueue(sn7325_wq);
}

#endif
