#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/charging.h>
#include "bq24296.h"

/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define bq28z610_SLAVE_ADDR_WRITE   0xAB
#define bq28z610_SLAVE_ADDR_READ    0xAA
#define bq28z610_REG_NUM			0x3e

static struct i2c_client *new_client;
static const struct i2c_device_id bq28z610_i2c_id[] = { {"bq28z610", 0}, {} };

kal_bool external_fg_hw_init_done = KAL_FALSE;
static int bq28z610_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id bq28z610_of_match[] = {
	{.compatible = "mediatek,swithing_charger",},
	{},
};

MODULE_DEVICE_TABLE(of, bq28z610_of_match);
#endif

static struct i2c_driver bq28z610_driver = {
	.driver = {
		   .name = "bq28z610",
#ifdef CONFIG_OF
		   .of_match_table = bq28z610_of_match,
#endif
		   },
	.probe = bq28z610_driver_probe,
	.id_table = bq28z610_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char bq28z610_reg[bq28z610_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq28z610_i2c_access);

int g_bq28z610_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write bq28z610]
  *
  *********************************************************/
int bq28z610_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&bq28z610_i2c_access);

	/* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG; */
	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
		new_client->ext_flag = 0;

		mutex_unlock(&bq28z610_i2c_access);
		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
	new_client->ext_flag = 0;

	mutex_unlock(&bq28z610_i2c_access);
	return 1;
}

int bq28z610_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&bq28z610_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&bq28z610_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&bq28z610_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq28z610_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char bq28z610_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq28z610_read_byte(RegNum, &bq28z610_reg);

	battery_log(BAT_LOG_FULL, "[bq28z610_read_interface] Reg[%x]=0x%x\n", RegNum, bq28z610_reg);

	bq28z610_reg &= (MASK << SHIFT);
	*val = (bq28z610_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[bq28z610_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int bq28z610_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq28z610_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq28z610_read_byte(RegNum, &bq28z610_reg);
	battery_log(BAT_LOG_FULL, "[bq28z610_config_interface] Reg[%x]=0x%x\n", RegNum, bq28z610_reg);

	bq28z610_reg &= ~(MASK << SHIFT);
	bq28z610_reg |= (val << SHIFT);

	ret = bq28z610_write_byte(RegNum, bq28z610_reg);
	battery_log(BAT_LOG_FULL, "[bq28z610_config_interface] write Reg[%x]=0x%x\n", RegNum, bq28z610_reg);

	/* Check */
	/* bq28z610_read_byte(RegNum, &bq28z610_reg); */
	/* battery_log(BAT_LOG_FULL, "[bq28z610_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq28z610_reg); */

	return ret;
}

/* write one register directly */
unsigned int bq28z610_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned int ret = 0;

	ret = bq28z610_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
signed int bq28z610_read_current(void)
{
	signed int external_fg_current= 0;
	bq28z610_read_byte(0x02, &bq28z610_reg[0x02]);
	bq28z610_read_byte(0x03, &bq28z610_reg[0x03]);
	external_fg_current= bq28z610_reg[0x03] << 8 | bq28z610_reg[0x02];
	battery_log(BAT_LOG_CRTI, "bq28z610 current:%u ma\n",external_fg_current);
	return external_fg_current;
}

unsigned int bq28z610_read_temperature(void)
{
	unsigned int temperature= 0;
	bq28z610_read_byte(0x06, &bq28z610_reg[0x06]);
	bq28z610_read_byte(0x07, &bq28z610_reg[0x07]);
	temperature= bq28z610_reg[0x07] << 8 | bq28z610_reg[0x06];
	battery_log(BAT_LOG_CRTI, "bq28z610 temperature:%d c\n",(temperature-2721)/10);
	temperature = (temperature - 2721)/10;
	return temperature;
}

unsigned int bq28z610_read_valtage(void)
{
	unsigned int valtage= 0;
	bq28z610_read_byte(0x08, &bq28z610_reg[0x08]);
	bq28z610_read_byte(0x09, &bq28z610_reg[0x09]);
	valtage = bq28z610_reg[0x09] << 8 | bq28z610_reg[0x08];
	battery_log(BAT_LOG_CRTI, "bq28z610 valtage:%d mv\n",valtage);
	return valtage;
}

unsigned int bq28z610_read_status(void)
{
	unsigned int status= 0;
	bq28z610_read_byte(0x0a, &bq28z610_reg[0x0a]);
	bq28z610_read_byte(0x0b, &bq28z610_reg[0x0b]);
	status= bq28z610_reg[0x0b] << 8 | bq28z610_reg[0x0a];
	battery_log(BAT_LOG_CRTI, "bq28z610 status:0x%x \n",status);
	return status;
}

signed int bq28z610_read_instantaneous_current(void)
{
	signed int external_fg_instantaneous_current= 0;
	bq28z610_read_byte(0x0c, &bq28z610_reg[0x0c]);
	bq28z610_read_byte(0x0d, &bq28z610_reg[0x0d]);
	external_fg_instantaneous_current= bq28z610_reg[0x0d] << 8 | bq28z610_reg[0x0c];
	if(external_fg_instantaneous_current > 0xE000)
		external_fg_instantaneous_current = 0- (0xffff - external_fg_instantaneous_current);  //dis-charge
	battery_log(BAT_LOG_CRTI, "bq28z610 instantaneous current:%u ma\n",external_fg_instantaneous_current);
	return external_fg_instantaneous_current;
}

unsigned int bq28z610_read_remaining_capacity(void)
{
	unsigned int external_fg_remaining_capacity= 2000;
	bq28z610_read_byte(0x10, &bq28z610_reg[0x10]);
	bq28z610_read_byte(0x11, &bq28z610_reg[0x11]);
	external_fg_remaining_capacity= bq28z610_reg[0x11] << 8 | bq28z610_reg[0x10];
	battery_log(BAT_LOG_CRTI, "bq28z610 remaining capacity:%d amh\n",external_fg_remaining_capacity);
	return external_fg_remaining_capacity;
}

unsigned int bq28z610_read_capacity(void)
{
	unsigned int external_fg_capacity= 4000;
	bq28z610_read_byte(0x12, &bq28z610_reg[0x12]);
	bq28z610_read_byte(0x13, &bq28z610_reg[0x13]);
	external_fg_capacity= bq28z610_reg[0x13] << 8 | bq28z610_reg[0x12];
	battery_log(BAT_LOG_CRTI, "bq28z610 capacity:%d amh\n",external_fg_capacity);
	return external_fg_capacity;
}

unsigned int bq28z610_read_soc(void)
{
	unsigned int external_fg_soc= 50;
	bq28z610_read_byte(0x2c, &bq28z610_reg[0x2c]);
	bq28z610_read_byte(0x2d, &bq28z610_reg[0x2d]);
	external_fg_soc= bq28z610_reg[0x2d] << 8 | bq28z610_reg[0x2c];
	battery_log(BAT_LOG_CRTI, "bq28z610 soc:%d %%\n",external_fg_soc);
	return external_fg_soc;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/

void bq28z610_dump_register(void)
{
	int i = 0;

	battery_log(BAT_LOG_FULL, "[bq28z610] ");
	for (i = 0; i < bq28z610_REG_NUM; i++) {
		bq28z610_read_byte(i, &bq28z610_reg[i]);
	//	printk("bq28z610 [0x%x]=0x%x \n", i, bq28z610_reg[i]);
	}
	bq28z610_read_current();
	bq28z610_read_temperature();
	bq28z610_read_valtage();
	bq28z610_read_status();
	bq28z610_read_status();
	bq28z610_read_instantaneous_current();
	bq28z610_read_remaining_capacity();
	bq28z610_read_capacity();
	bq28z610_read_soc();
	battery_log(BAT_LOG_FULL, "bq28z610 \n");
}

static int bq28z610_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	battery_log(BAT_LOG_CRTI, "[bq28z610_driver_probe]\n");

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

	if (!new_client) {
		err = -ENOMEM;
		goto exit;
	}
	memset(new_client, 0, sizeof(struct i2c_client));

	new_client = client;

	/* --------------------- */
	//bq28z610_hw_component_detect();
	bq28z610_dump_register();
	external_fg_hw_init_done = KAL_TRUE;

	return 0;

exit:
	return err;

}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_bq28z610 = 0;
static ssize_t show_bq28z610_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_bq28z610_access] 0x%x\n", g_reg_value_bq28z610);
	return sprintf(buf, "%u\n", g_reg_value_bq28z610);
}

static ssize_t store_bq28z610_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	battery_log(BAT_LOG_CRTI, "[store_bq28z610_access]\n");

	if (buf != NULL && size != 0) {
		battery_log(BAT_LOG_CRTI, "[store_bq28z610_access] buf is %s and size is %zu\n", buf, size);
		/*reg_address = kstrtoul(buf, 16, &pvalue);*/

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			battery_log(BAT_LOG_CRTI,
			    "[store_bq28z610_access] write bq28z610 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = bq28z610_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = bq28z610_read_interface(reg_address, &g_reg_value_bq28z610, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq28z610_access] read bq28z610 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_bq28z610);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq28z610_access] Please use \"cat bq28z610_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(bq28z610_access, 0664, show_bq28z610_access, store_bq28z610_access);	/* 664 */

static int bq28z610_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	battery_log(BAT_LOG_CRTI, "******** bq28z610_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq28z610_access);

	return 0;
}

struct platform_device bq28z610_user_space_device = {
	.name = "bq28z610-user",
	.id = -1,
};

static struct platform_driver bq28z610_user_space_driver = {
	.probe = bq28z610_user_space_probe,
	.driver = {
		   .name = "bq28z610-user",
		   },
};

static int __init bq28z610_subsys_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&bq28z610_driver) != 0)
		battery_log(BAT_LOG_CRTI, "[bq24261_init] failed to register bq24261 i2c driver.\n");
	else
		battery_log(BAT_LOG_CRTI, "[bq24261_init] Success to register bq24261 i2c driver.\n");

	/* bq28z610 user space access interface */
	ret = platform_device_register(&bq28z610_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq28z610_init] Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&bq28z610_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq28z610_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit bq28z610_exit(void)
{
	i2c_del_driver(&bq28z610_driver);
}

/* module_init(bq28z610_init); */
/* module_exit(bq28z610_exit); */
subsys_initcall(bq28z610_subsys_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq28z610 Driver");
MODULE_AUTHOR("YT Lee<yt.lee@mediatek.com>");
