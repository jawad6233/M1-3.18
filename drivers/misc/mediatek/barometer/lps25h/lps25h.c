/* LPS25H Pressure Sensor Driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * History: V1.0 --- [2015.11.23]Driver creation
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <cust_baro.h>
#include "barometer.h"
#include "lps25h.h"
#define POWER_NONE_MACRO	MT65XX_POWER_NONE
#define CONFIG_ID_TEMPERATURE

#define	I2C_AUTO_INCREMENT	0x80
#define	SENSITIVITY_T		48	/* =	480 LSB/degrC	*/
#define	TEMPERATURE_OFFSET	4250	/* =	42.5 degrC	*/
#define	SENSITIVITY_P		4096	/* =	LSB/mbar	*/

#define RES_CONF 		0x10
#define CTRL_REG1 		0x20
#define CTRL_REG2 		0x21
#define	PRESS_OUT_XL		0x28	/* press output (3 regs) */
#define TEMP_OUT_L		0x2B	/* temp output (2 regs) */
#define FIFO_CTRL 		0x2E

static DEFINE_MUTEX(lps25h_i2c_mutex);
static DEFINE_MUTEX(lps25h_op_mutex);

/* sensor type */
enum SENSOR_TYPE_ENUM {
	LPS25H_TYPE = 0x0,

	INVALID_TYPE = 0xff
};

/* power mode */
enum SENSOR_POWERMODE_ENUM {
	SUSPEND_POWERMODE = 0x0,
	NORMAL_POWERMODE,

	UNDEFINED_POWERMODE = 0xff
};

/* trace */
enum BAR_TRC {
	BAR_TRC_READ	= 0x01,
	BAR_TRC_RAWDATA	= 0x02,
	BAR_TRC_IOCTL	= 0x04,
	BAR_TRC_FILTER	= 0x08,
};

/* s/w filter */
struct data_filter {
	u32 raw[C_MAX_FIR_LENGTH][LPS25H_DATA_NUM];
	int sum[LPS25H_DATA_NUM];
	int num;
	int idx;
};

/* lps25h i2c client data */
struct lps25h_i2c_data {
	struct i2c_client *client;
	struct baro_hw *hw;

	/* sensor info */
	u8 sensor_name[MAX_SENSOR_NAME];
	enum SENSOR_TYPE_ENUM sensor_type;
	enum SENSOR_POWERMODE_ENUM power_mode;

	/*misc*/
	atomic_t trace;
	atomic_t suspend;
	atomic_t filter;

#if defined(CONFIG_LPS25H_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
};

#define BAR_TAG                  "[barometer] "
#define BAR_FUN(f)               printk(KERN_INFO BAR_TAG"%s\n", __func__)
#define BAR_ERR(fmt, args...) \
	printk(KERN_ERR BAR_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define BAR_LOG(fmt, args...)    printk(KERN_ERR BAR_TAG fmt, ##args)

//static struct i2c_driver lps25h_i2c_driver;
static struct lps25h_i2c_data *obj_i2c_data;
static const struct i2c_device_id lps25h_i2c_id[] = {
	{LPS25H_DEV_NAME, 0},
	{}
};

/*static struct i2c_board_info __initdata lps25h_i2c_info = {
	I2C_BOARD_INFO(LPS25H_DEV_NAME, LPS25H_I2C_ADDRESS)
};*/

static int lps25h_local_init(void);
static int lps25h_remove(void);
static int lps25h_init_flag =-1; // 0<==>OK -1 <==> fail

static struct baro_init_info lps25h_init_info = {
		.name = "lps25h",
		.init = lps25h_local_init,
		.uninit = lps25h_remove,
};
struct baro_hw baro_cust;
static struct baro_hw *hw = &baro_cust;
/* I2C operation functions */
static int lps25h_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
	int err;
	struct i2c_msg msgs[2]={{0},{0}};
	
	mutex_lock(&lps25h_i2c_mutex);
	
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len =1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len =len;
	msgs[1].buf = data;
	printk("LPS25msgs[0].addr=%x msgs[0].buf=%x\n",(int)msgs[0].addr,beg);
	if (!client)
	{
	    mutex_unlock(&lps25h_i2c_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) 
	{
		BAR_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lps25h_i2c_mutex);
		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) 
	{
		BAR_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
		err = -EIO;
	} 
	else 
	{
		err = 0;
	}
	printk("i2c_transfer error: (add%d data%d len%d) err%d\n",addr, *data, len, err);
	mutex_unlock(&lps25h_i2c_mutex);
	return err;

}

static int lps25h_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{	/*because address also occupies one byte, the maximum length for write is 7 bytes*/
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];
	err =0;
	
	mutex_lock(&lps25h_i2c_mutex);
	
	if (!client)
	{
		mutex_unlock(&lps25h_i2c_mutex);
		return -EINVAL;
	}
	else if (len >= C_I2C_FIFO_SIZE) 
	{		 
		BAR_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lps25h_i2c_mutex);
		return -EINVAL;
	}	 

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
	{
		buf[num++] = data[idx];
	}

	err = i2c_master_send(client, buf, num);
	if (err < 0)
	{
		BAR_ERR("send command error!!\n");
		mutex_unlock(&lps25h_i2c_mutex);
		return -EFAULT;
	} 
	mutex_unlock(&lps25h_i2c_mutex);
	return err;
}

static void lps25h_power(struct baro_hw *hw, unsigned int on)
{
#if 0
	static unsigned int power_on;

	if (hw->power_id != POWER_NONE_MACRO) {/* have externel LDO */
		BAR_LOG("power %s\n", on ? "on" : "off");
		if (power_on == on) {/* power status not change */
			BAR_LOG("ignore power control: %d\n", on);
		} else if (on) {/* power on */
			if (!hwPowerOn(hw->power_id, hw->power_vol, LPS25H_DEV_NAME))
				BAR_ERR("power on failed\n");
		} else {/* power off */
			if (!hwPowerDown(hw->power_id, LPS25H_DEV_NAME))
				BAR_ERR("power off failed\n");
		}
	}
	power_on = on;
	#endif
}

/* get chip type */
static int lps25h_get_chip_type(struct i2c_client *client)
{
	int err = 0;
	u8 chip_id = 0;
	struct lps25h_i2c_data *obj = i2c_get_clientdata(client);
	BAR_FUN(f);

	err = lps25h_i2c_read_block(client, LPS25H_WHO_AM_I, &chip_id, 0x01);
	if (err != 0)
		return err;

	switch (chip_id) {
	case LPS25H_CHIP_ID:
		obj->sensor_type = LPS25H_TYPE;
		strcpy(obj->sensor_name, "lps25h");
		break;
	default:
		obj->sensor_type = INVALID_TYPE;
		strcpy(obj->sensor_name, "unknown sensor");
		break;
	}

	BAR_LOG("[%s]chip id = %#x, sensor name = %s\n", __func__, chip_id, obj->sensor_name);

	if (obj->sensor_type == INVALID_TYPE) {
		BAR_ERR("unknown pressure sensor\n");
		return -1;
	}
	return 0;
}

static int lps25h_set_powermode(struct i2c_client *client, enum SENSOR_POWERMODE_ENUM power_mode)
{
	struct lps25h_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	u8 data;

	BAR_LOG("[%s] power_mode = %d, old power_mode = %d\n", __func__,
		power_mode, obj->power_mode);

	if (power_mode == obj->power_mode)
		return 0;

	if (obj->sensor_type == LPS25H_TYPE) {/* LPS25H */
		if (NORMAL_POWERMODE == power_mode) {
			data = 0xC4; // activity mode, 25Hz, BDU enabled
		}
		else if (SUSPEND_POWERMODE == power_mode) {
			data = 0x00; // power down mode
		}

		err = lps25h_i2c_write_block(client, CTRL_REG1, &data, 1);
		if (err < 0) {
			BAR_ERR("%s: set ctrl_reg1 register failed, err = %d\n", __func__, err);
		}
	}

	if (err < 0) {
		BAR_ERR("set power mode failed, err = %d, sensor name = %s\n",
			err, obj->sensor_name);
	}
	else {
		obj->power_mode = power_mode;
	}

	return err;
}

static int lps25h_read_raw_temperature(struct i2c_client *client, s16 *temperature)
{
	struct lps25h_i2c_data *priv = i2c_get_clientdata(client);
	s32 err = 0;
	u8 data[8];
	int regToRead = 2;

	if (NULL == client) {
		err = -EINVAL;
		return err;
	}
	mutex_lock(&lps25h_op_mutex);
	err = lps25h_i2c_read_block(client,0X27, data, 1);
	printk("LPS251111111 status LPS25 tempL1=%d err=%d\n",data[0],err);
	err = lps25h_i2c_read_block(client,LPS25H_WHO_AM_I, data, 1);
	printk("LPS251111111 WHO_AM_I=%d err=%d\n",data[0],err);
	err = lps25h_i2c_read_block(client,CTRL_REG1, data, 1);
	printk("LPS251111111 CTRL_REG1=%d err=%d\n",data[0],err);
	err = lps25h_i2c_read_block(client,CTRL_REG2, data, 1);
	printk("LPS251111111 CTRL_REG2=%d err=%d\n",data[0],err);
	if (priv->sensor_type == LPS25H_TYPE) {/* LPS25H/B */
		/* Data bytes from hardware	PRESS_OUT_XL,PRESS_OUT_L,PRESS_OUT_H, */
		/*				TEMP_OUT_L, TEMP_OUT_H */
		err = lps25h_i2c_read_block(client, I2C_AUTO_INCREMENT|TEMP_OUT_L, data, regToRead);
		printk("LPS25 tempH=%d LPS25 tempL1=%d err=%d\n",data[1],data[0],err);
		if (err < 0)
			return err;

		*temperature = (s16) ((((s8) data[1]) << 8) | (data[0]));
	}
	err = lps25h_i2c_read_block(client,0X27, data, 1);
	printk("LPS25222222222 status LPS25 tempL1=%d err=%d\n",data[0],err);
	mutex_unlock(&lps25h_op_mutex);

	return err;
}

static int lps25h_read_raw_pressure(struct i2c_client *client, s32 *pressure)
{
	struct lps25h_i2c_data *priv = i2c_get_clientdata(client);
	s32 err = 0;
	u8 prs_data[8];
	int regToRead = 3;

	if (NULL == client) {
		err = -EINVAL;
		return err;
	}
	mutex_lock(&lps25h_op_mutex);
	if (priv->sensor_type == LPS25H_TYPE) {/* LPS25H */
		/* Data bytes from hardware	PRESS_OUT_XL,PRESS_OUT_L,PRESS_OUT_H, */
		/*				TEMP_OUT_L, TEMP_OUT_H */
		err = lps25h_i2c_read_block(client, I2C_AUTO_INCREMENT|PRESS_OUT_XL, prs_data, regToRead);
		if (err < 0)
			return err;

		*pressure = (s32)((((s8) prs_data[2]) << 16) | (prs_data[1] << 8) | (prs_data[0]));
	}
	mutex_unlock(&lps25h_op_mutex);
#ifdef CONFIG_LPS25H_LOWPASS
/*
*Example: firlen = 16, filter buffer = [0] ... [15],
*when 17th data come, replace [0] with this new data.
*Then, average this filter buffer and report average value to upper layer.
*/
	if (atomic_read(&priv->filter)) {
		if (atomic_read(&priv->fir_en) &&
			!atomic_read(&priv->suspend)) {
			int idx, firlen = atomic_read(&priv->firlen);
			if (priv->fir.num < firlen) {
				priv->fir.raw[priv->fir.num][LPS25H_PRESSURE] = *pressure;
				priv->fir.sum[LPS25H_PRESSURE] += *pressure;
				if (atomic_read(&priv->trace) &
					BAR_TRC_FILTER) {
					BAR_LOG("add [%2d] [%5d] => [%5d]\n",
					priv->fir.num,
					priv->fir.raw
					[priv->fir.num][LPS25H_PRESSURE],
					priv->fir.sum[LPS25H_PRESSURE]);
				}
				priv->fir.num++;
				priv->fir.idx++;
			} else {
				idx = priv->fir.idx % firlen;
				priv->fir.sum[LPS25H_PRESSURE] -= priv->fir.raw[idx][LPS25H_PRESSURE];
				priv->fir.raw[idx][LPS25H_PRESSURE] = *pressure;
				priv->fir.sum[LPS25H_PRESSURE] += *pressure;
				priv->fir.idx++;
				*pressure = priv->fir.sum[LPS25H_PRESSURE]/firlen;
				if (atomic_read(&priv->trace) &
					BAR_TRC_FILTER) {
					BAR_LOG("add [%2d][%5d]=>[%5d]:[%5d]\n", idx,
					priv->fir.raw[idx][LPS25H_PRESSURE],
					priv->fir.sum[LPS25H_PRESSURE],
					*pressure);
				}
			}
		}
	}
#endif

	return err;
}

/*
*get compensated temperature
*unit:100 degrees centigrade
*/
static int lps25h_get_temperature(struct i2c_client *client,
		char *buf, int bufsize)
{
	struct lps25h_i2c_data *obj = i2c_get_clientdata(client);
	int status;
	s16 utemp = 0;/* uncompensated temperature */
	s32 temperature = 0;

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	status = lps25h_read_raw_temperature(client, &utemp);
	if (status != 0)
		return status;

	if (obj->sensor_type == LPS25H_TYPE) {/* LPS25H */
		temperature = TEMPERATURE_OFFSET + ((10*utemp)/SENSITIVITY_T);
	}

	sprintf(buf, "%08x", temperature);
	if (atomic_read(&obj->trace) & BAR_TRC_IOCTL)
		BAR_LOG("compensated temperature value: %s\n", buf);

	return status;
}

/*
*get compensated pressure
*unit: hectopascal(hPa)
*/
static int lps25h_get_pressure(struct i2c_client *client, char *buf, int bufsize)
{
	struct lps25h_i2c_data *obj = i2c_get_clientdata(client);
	int status;
	s32 upressure = 0, pressure = 0;

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}
	status = lps25h_read_raw_pressure(client, &upressure);
	if (status != 0)
		goto exit;
	if (obj->sensor_type == LPS25H_TYPE) {/* LPS25H */
		pressure = upressure;
	}
	sprintf(buf, "%08x", pressure);
	
	if (atomic_read(&obj->trace) & BAR_TRC_IOCTL){
		BAR_LOG("compensated pressure value: %s\n", buf);
		printk("LPS2H buf=%s\n",buf);
		}
exit:
	return status;
}

/* lps25h setting initialization */
static int lps25h_init_client(struct i2c_client *client)
{
	int err = 0;
	u8 data = 0;
	BAR_FUN();

	err = lps25h_get_chip_type(client);
	if (err < 0) {
		BAR_ERR("get chip type failed, err = %d\n", err);
		return err;
	}

	data = 0x0F; // press:512 avg, temp:64 avg
	err = lps25h_i2c_write_block(client, RES_CONF, &data, 1);
	if (err < 0) {
		BAR_ERR("set res_conf register failed, err = %d\n", err);
		return err;
	}

	data = 0x00; // power down mode
	err = lps25h_i2c_write_block(client, CTRL_REG1, &data, 1);
	if (err < 0) {
		BAR_ERR("set ctrl_reg1 register failed, err = %d\n", err);
		return err;
	}

	data = 0x40; // FIFO mean mode enabled
	err = lps25h_i2c_write_block(client, CTRL_REG2, &data, 1);
	if (err < 0) {
		BAR_ERR("set ctrl_reg2 register failed, err = %d\n", err);
		return err;
	}

	data = 0xDF; // 32 samples average
	err = lps25h_i2c_write_block(client, FIFO_CTRL, &data, 1);
	if (err < 0) {
		BAR_ERR("set fifo_ctrl register failed, err = %d\n", err);
		return err;
	}

	err = lps25h_set_powermode(client, NORMAL_POWERMODE);//modify by zy
	if (err < 0) {
		BAR_ERR("set power mode failed, err = %d\n", err);
		return err;
	}

	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct lps25h_i2c_data *obj = obj_i2c_data;

	if (NULL == obj) {
		BAR_ERR("lps25h i2c data pointer is null\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", obj->sensor_name);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct lps25h_i2c_data *obj = obj_i2c_data;
	char strbuf[LPS25H_BUFSIZE] = "";

	if (NULL == obj) {
		BAR_ERR("lps25h i2c data pointer is null\n");
		return 0;
	}

	lps25h_get_pressure(obj->client, strbuf, LPS25H_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lps25h_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		BAR_ERR("lps25h i2c data pointer is null\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf,
		size_t count)
{
	struct lps25h_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		BAR_ERR("i2c_data obj is null\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		BAR_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct lps25h_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		BAR_ERR("lps25h i2c data pointer is null\n");
		return 0;
	}

	if (obj->hw)
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			obj->hw->i2c_num,
			obj->hw->direction,
			obj->hw->power_id,
			obj->hw->power_vol);
	else
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

	len += snprintf(buf+len, PAGE_SIZE-len, "i2c addr:%#x,ver:%s\n",
			obj->client->addr, LPS25H_DRIVER_VERSION);

	return len;
}

static ssize_t show_power_mode_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct lps25h_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		BAR_ERR("lps25h i2c data pointer is null\n");
		return 0;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "%s mode\n",
		obj->power_mode == NORMAL_POWERMODE ? "normal" : "suspend");

	return len;
}

static ssize_t store_power_mode_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct lps25h_i2c_data *obj = obj_i2c_data;
	unsigned long power_mode;
	int err;

	if (obj == NULL) {
		BAR_ERR("lps25h i2c data pointer is null\n");
		return 0;
	}

	err = kstrtoul(buf, 10, &power_mode);

	if (err == 0) {
		err = lps25h_set_powermode(obj->client,
			(enum SENSOR_POWERMODE_ENUM)(!!(power_mode)));
		if (err)
			return err;
		return count;
	}
	return err;
}

static DRIVER_ATTR(chipinfo,	S_IRUGO,	show_chipinfo_value,	NULL);
static DRIVER_ATTR(sensordata,	S_IRUGO,	show_sensordata_value,	NULL);
static DRIVER_ATTR(trace,	S_IWUSR | S_IRUGO,
		show_trace_value,	store_trace_value);
static DRIVER_ATTR(status,	S_IRUGO,	show_status_value,	NULL);
static DRIVER_ATTR(powermode,	S_IWUSR | S_IRUGO,
		show_power_mode_value,	store_power_mode_value);

static struct driver_attribute *lps25h_attr_list[] = {
	&driver_attr_chipinfo,	/* chip information*/
	&driver_attr_sensordata,/* dump sensor data*/
	&driver_attr_trace,	/* trace log*/
	&driver_attr_status,	/* cust setting */
	&driver_attr_powermode,	/* power mode */
};

static int lps25h_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(lps25h_attr_list)/sizeof(lps25h_attr_list[0]));

	if (NULL == driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, lps25h_attr_list[idx]);
		if (err) {
			BAR_ERR("driver_create_file (%s) = %d\n",
			lps25h_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int lps25h_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(lps25h_attr_list)/sizeof(lps25h_attr_list[0]));

	if (NULL == driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, lps25h_attr_list[idx]);

	return err;
}

int barometer_operate(void *self, uint32_t command, void *buff_in, int size_in,
		void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct lps25h_i2c_data *priv = (struct lps25h_i2c_data *)self;
	struct hwm_sensor_data *barometer_data;
	char buff[LPS25H_BUFSIZE];

	switch (command) {
	case SENSOR_DELAY:
		/* under construction */
	break;

	case SENSOR_ENABLE:
	if ((buff_in == NULL) || (size_in < sizeof(int))) {
		BAR_ERR("enable sensor parameter error\n");
		err = -EINVAL;
	} else {
		mutex_lock(&lps25h_op_mutex);
		/* value:[0--->suspend, 1--->normal] */
		value = *(int *)buff_in;
		BAR_LOG("sensor enable/disable command: %s\n",
			value ? "enable" : "disable");

		err = lps25h_set_powermode(priv->client, (enum SENSOR_POWERMODE_ENUM)(!!value));
		if (err)
			BAR_ERR("set power mode failed, err = %d\n", err);
#ifdef CONFIG_LPS25H_LOWPASS
		/* clear filter buffer */
		if (value == 0) {
			memset(&(priv->fir), 0, sizeof(struct data_filter));
		}
#endif
		mutex_unlock(&lps25h_op_mutex);
	}
	break;

	case SENSOR_GET_DATA:
	if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
		BAR_ERR("get sensor data parameter error\n");
		err = -EINVAL;
	} else {
		mutex_lock(&lps25h_op_mutex);
		barometer_data = (struct hwm_sensor_data *)buff_out;
		err = lps25h_get_pressure(priv->client, buff, LPS25H_BUFSIZE);
		if (err) {
			BAR_ERR("get compensated pressure value failed,"
				"err = %d\n", err);
			return -1;
		}
		sscanf(buff, "%x", &barometer_data->values[0]);
		barometer_data->values[1] = barometer_data->values[2] = 0;
		barometer_data->status = SENSOR_STATUS_ACCURACY_HIGH;
		barometer_data->value_divide = SENSITIVITY_P;
		mutex_unlock(&lps25h_op_mutex);
	}
	break;

	default:
		BAR_ERR("barometer operate function no this parameter %d\n",
			command);
		err = -1;
	break;
	}

	return err;
}

#ifdef CONFIG_ID_TEMPERATURE
int temperature_operate(void *self, uint32_t command, void *buff_in,
		int size_in, void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct lps25h_i2c_data *priv = (struct lps25h_i2c_data *)self;
	struct hwm_sensor_data *temperature_data;
	char buff[LPS25H_BUFSIZE];

	switch (command) {
	case SENSOR_DELAY:
	/* under construction */
	break;

	case SENSOR_ENABLE:
	if ((buff_in == NULL) || (size_in < sizeof(int))) {
		BAR_ERR("enable sensor parameter error\n");
		err = -EINVAL;
	} else {
		mutex_lock(&lps25h_op_mutex);
		/* value:[0--->suspend, 1--->normal] */
		value = *(int *)buff_in;
		BAR_LOG("sensor enable/disable command: %s\n",
			value ? "enable" : "disable");

		err = lps25h_set_powermode(priv->client,
			(enum SENSOR_POWERMODE_ENUM)(!!value));
		if (err)
			BAR_ERR("set power mode failed, err = %d\n", err);

		mutex_unlock(&lps25h_op_mutex);
	}
	break;

	case SENSOR_GET_DATA:
	if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
		BAR_ERR("get sensor data parameter error\n");
		err = -EINVAL;
	} else {
		mutex_lock(&lps25h_op_mutex);
		temperature_data = (struct hwm_sensor_data *)buff_out;
		err = lps25h_get_temperature(priv->client, buff, LPS25H_BUFSIZE);
		if (err) {
			BAR_ERR("get compensated temperature value failed,"
				"err = %d\n", err);
			return -1;
		}
		sscanf(buff, "%x", &temperature_data->values[0]);
		temperature_data->values[1] = temperature_data->values[2] = 0;
		temperature_data->status = SENSOR_STATUS_ACCURACY_HIGH;
		temperature_data->value_divide = 100;
		mutex_unlock(&lps25h_op_mutex);
	}
	break;

	default:
		BAR_ERR("temperature operate function no this parameter %d\n",
			command);
		err = -1;
	break;
	}

	return err;
}
#endif/* CONFIG_ID_TEMPERATURE */

static int lps25h_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_i2c_data;

	if (file->private_data == NULL) {
		BAR_ERR("null pointer\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int lps25h_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long lps25h_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct lps25h_i2c_data *obj = (struct lps25h_i2c_data *)file->private_data;
	struct i2c_client *client = obj->client;
	char strbuf[LPS25H_BUFSIZE];
	u32 dat = 0;
	void __user *data;
	int err = 0;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
			(void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
			(void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		BAR_ERR("access error: %08X, (%2d, %2d)\n",
			cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case BAROMETER_IOCTL_INIT:
		lps25h_init_client(client);
		err = lps25h_set_powermode(client, NORMAL_POWERMODE);
		if (err) {
			err = -EFAULT;
			break;
		}
	break;

	case BAROMETER_IOCTL_READ_CHIPINFO:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		strcpy(strbuf, obj->sensor_name);
		if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
			err = -EFAULT;
			break;
		}
	break;

	case BAROMETER_GET_PRESS_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		printk("LPS2H BAROMETER_GET_PRESS_DATA\n");
		lps25h_get_pressure(client, strbuf, LPS25H_BUFSIZE);
				printk("LPS2H strbuf=%s\n",strbuf);

		sscanf(strbuf, "%x", &dat);
		printk("LPS2H dat=%x\n",dat);
		if (copy_to_user(data, &dat, sizeof(dat))) {
			printk("LPS2H ERROR\n");
			err = -EFAULT;
			break;
		}
			//printk("LPS2H ERRORdata=%ld\n",(int)data);

	break;

	case BAROMETER_GET_TEMP_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		lps25h_get_temperature(client, strbuf, LPS25H_BUFSIZE);
		sscanf(strbuf, "%x", &dat);
		printk("LPS2H temp=%x\n",dat);
		if (copy_to_user(data, &dat, sizeof(dat))) {
			err = -EFAULT;
			break;
		}
	break;

	default:
		BAR_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
	break;
	}

	return err;
}

static const struct file_operations lps25h_fops = {
	.owner = THIS_MODULE,
	.open = lps25h_open,
	.release = lps25h_release,
	.unlocked_ioctl = lps25h_unlocked_ioctl,
};

static struct miscdevice lps25h_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "barometer",
	.fops = &lps25h_fops,
};

static int lps25h_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct lps25h_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	BAR_FUN();
	mutex_lock(&lps25h_op_mutex);
	if (msg.event == PM_EVENT_SUSPEND) {
		if (NULL == obj) {
			BAR_ERR("null pointer\n");
			mutex_unlock(&lps25h_op_mutex);
			return -EINVAL;
		}

		atomic_set(&obj->suspend, 1);
		err = lps25h_set_powermode(obj->client, SUSPEND_POWERMODE);
		if (err) {
			BAR_ERR("lps25h set suspend mode failed, err = %d\n", err);
			mutex_unlock(&lps25h_op_mutex);
			return err;
		}
		lps25h_power(obj->hw, 0);
	}
	mutex_unlock(&lps25h_op_mutex);
	return err;
}

static int lps25h_resume(struct i2c_client *client)
{
	struct lps25h_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	BAR_FUN();
	mutex_lock(&lps25h_op_mutex);
	if (NULL == obj) {
		BAR_ERR("null pointer\n");
		mutex_unlock(&lps25h_op_mutex);
		return -EINVAL;
	}

	lps25h_power(obj->hw, 1);

	err = lps25h_init_client(obj->client);
	if (err) {
		BAR_ERR("initialize client fail\n");
		mutex_unlock(&lps25h_op_mutex);
		return err;
	}

#ifdef CONFIG_LPS25H_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	atomic_set(&obj->suspend, 0);
	mutex_unlock(&lps25h_op_mutex);
	return 0;
}

static int lps25h_i2c_detect(struct i2c_client *client,
		struct i2c_board_info *info)
{
	strcpy(info->type, LPS25H_DEV_NAME);
	return 0;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int lps25h_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int lps25h_enable_nodata(int en)
{
	int res =0;
	int retry = 0;
	bool power=false;
	
	if(1==en)
	{
		power=true;
	}
	if(0==en)
	{
		power =false;
	}

	for(retry = 0; retry < 3; retry++){
		res = lps25h_set_powermode(obj_i2c_data->client,(enum SENSOR_POWERMODE_ENUM)(!!power));//modify
		if(res == 0)
		{
			BAR_LOG("lps25h_set_powermode done\n");
			break;
		}
		BAR_ERR("lps25h_set_powermode fail\n");
	}

	
	if(res != 0)
	{
		BAR_ERR("lps25h_set_powermode fail!\n");
		return -1;
	}
	BAR_LOG("lps25h_set_powermode OK!\n");
	return 0;
}

static int lps25h_set_delay(u64 ns)
{
	return 0;
}

static int lps25h_get_data(int* value, int* status)
{
	char buff[LPS25H_BUFSIZE];
	int err = 0;
	
	err = lps25h_get_pressure(obj_i2c_data->client, buff, LPS25H_BUFSIZE);
	if (err) {
		BAR_ERR("get compensated pressure value failed,"
			"err = %d\n", err);
		return -1;
	}
	sscanf(buff, "%x", value);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int lps25h_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
#ifdef CONFIG_ID_TEMPERATURE
	struct hwmsen_object sobj_t;
#endif
struct lps25h_i2c_data *obj;
	struct baro_control_path ctl = { 0 };
	struct baro_data_path data = { 0 };
	int err = 0;

	BAR_FUN();
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	obj->hw = hw;
	obj_i2c_data = obj;
	obj->client = client;
	i2c_set_clientdata(client, obj);
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	obj->power_mode = UNDEFINED_POWERMODE;
	//obj->last_temp_measurement = 0;
	//obj->temp_measurement_period = 1 * HZ;	/* temperature update period:1s */
	//mutex_init(&obj->lock);

#ifdef CONFIG_LPS25H_LOWPASS
	if (obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);

	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
#endif
	err = lps25h_init_client(client);
	if (err)
		goto exit_init_client_failed;
	err = misc_register(&lps25h_device);
	if (err) {
		BAR_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}
	ctl.is_use_common_factory = false;
	err = lps25h_create_attr(&(lps25h_init_info.platform_diver_addr->driver));
	if (err) {
		BAR_ERR("create attribute failed, err = %d\n", err);
		goto exit_create_attr_failed;
	}
	ctl.open_report_data = lps25h_open_report_data;
	ctl.enable_nodata = lps25h_enable_nodata;
	ctl.set_delay = lps25h_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw->is_batch_supported;
	err = baro_register_control_path(&ctl);
	if(err)
	{
	 	BAR_ERR("register baro control path err\n");
		goto exit_hwmsen_attach_pressure_failed;
	}
	data.get_data = lps25h_get_data;
	data.vender_div = SENSITIVITY_P;//100;
	err = baro_register_data_path(&data);
	if (err) {
		BAR_ERR("baro_register_data_path failed, err = %d\n", err);
		goto exit_hwmsen_attach_pressure_failed;
	}
	err = batch_register_support_info(ID_PRESSURE,obj->hw->is_batch_supported, data.vender_div, 0);
	if(err)
	{
		BAR_ERR("register baro batch support err = %d\n", err);
		goto exit_hwmsen_attach_pressure_failed;
	}

#ifdef CONFIG_ID_TEMPERATURE
	sobj_t.self = obj;
	sobj_t.polling = 1;
	sobj_t.sensor_operate = temperature_operate;
	err = hwmsen_attach(ID_TEMPRERATURE, &sobj_t);
	if (err) {
		BAR_ERR("hwmsen attach failed, err = %d\n", err);
		goto exit_hwmsen_attach_temperature_failed;
	}
#endif/* CONFIG_ID_TEMPERATURE */
	lps25h_init_flag =0;
	BAR_LOG("%s: OK\n", __func__);
	return 0;

#ifdef CONFIG_ID_TEMPERATURE
exit_hwmsen_attach_temperature_failed:
	hwmsen_detach(ID_TEMPRERATURE);
#endif/* CONFIG_ID_TEMPERATURE */
exit_hwmsen_attach_pressure_failed:
	lps25h_delete_attr(&(lps25h_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
	misc_deregister(&lps25h_device);
exit_misc_device_register_failed:
exit_init_client_failed:
	kfree(obj);
exit:
	BAR_ERR("err = %d\n", err);
	lps25h_init_flag =-1;
	return err;
}

static int lps25h_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = hwmsen_detach(ID_PRESSURE);
	if (err)
		BAR_ERR("hwmsen_detach ID_PRESSURE failed, err = %d\n", err);

#ifdef CONFIG_ID_TEMPERATURE
	err = hwmsen_detach(ID_TEMPRERATURE);
	if (err)
		BAR_ERR("hwmsen_detach ID_TEMPRERATURE failed, err = %d\n", err);
#endif

	err = lps25h_delete_attr(&(lps25h_init_info.platform_diver_addr->driver));
	if (err)
		BAR_ERR("lps25h_delete_attr failed, err = %d\n", err);

	err = misc_deregister(&lps25h_device);
	if (err)
		BAR_ERR("misc_deregister failed, err = %d\n", err);

	obj_i2c_data = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
static const struct of_device_id lps25_of_match[] = {
	{.compatible = "mediatek,pressure"},
	{},
};
static struct i2c_driver lps25h_i2c_driver = {
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	LPS25H_DEV_NAME,
		.of_match_table = lps25_of_match,
	},
	
	.probe = lps25h_i2c_probe,
	.remove = lps25h_i2c_remove,
	.detect = lps25h_i2c_detect,
	.suspend = lps25h_suspend,
	.resume = lps25h_resume,
	.id_table = lps25h_i2c_id,
};

static int lps25h_remove(void)
{
	struct baro_hw *hw =hw;
	BAR_FUN();
	lps25h_power(hw, 0);
	i2c_del_driver(&lps25h_i2c_driver);
	return 0;
}

static int  lps25h_local_init(void)
{
 //  struct baro_hw *hw = hw;
	printk("lps25h loccal init+++\n");
 printk("BAROMETER_GET_PRESS_DATA=%ld\n",BAROMETER_GET_PRESS_DATA);
		printk("BAROMETER_GET_TEMP_DATA=%ld\n",BAROMETER_GET_TEMP_DATA);
	lps25h_power(hw, 1);
	if(i2c_add_driver(&lps25h_i2c_driver))
	{
		BAR_ERR("add driver error\n");
		return -1;
	}
	if(-1 == lps25h_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}

static int __init lps25h_init(void)
{
	//struct baro_hw *hw = get_cust_baro_hw();
	const char *name = "mediatek,lps25h";
		printk("BAROMETER_GET_PRESS_DATA=%ld\n",BAROMETER_GET_PRESS_DATA);
		printk("BAROMETER_GET_TEMP_DATA=%ld\n",BAROMETER_GET_TEMP_DATA);


	hw =   get_baro_dts_func(name, hw);
	if (!hw)
		BAR_ERR("get cust_baro dts info fail\n");
	BAR_FUN();
	printk("lps25h_initi2Cadd0=%d\n",hw->i2c_addr[0]);
	printk("lps25h_initi2Cadd1=%d\n",hw->i2c_addr[1]);
		printk("lps25h_initi2i2c_num=%d\n",hw->i2c_num);
				printk("lps25h_initi2direction=%d\n",hw->direction);


	//i2c_register_board_info(hw->i2c_num, &lps25h_i2c_info, 1);
	baro_driver_add(&lps25h_init_info);
	return 0;
}

static void __exit lps25h_exit(void)
{
	BAR_FUN();
}

module_init(lps25h_init);
module_exit(lps25h_exit);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("LPS25H/B I2C Driver");
MODULE_AUTHOR("jay.huangfu@st.com");
MODULE_VERSION(LPS25H_DRIVER_VERSION);
