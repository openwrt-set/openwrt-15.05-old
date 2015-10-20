#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/vermagic.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>

extern u64 uevent_next_seqnum(void);

static DEFINE_IDR(maxim59xx_id);
static DEFINE_MUTEX(maxim59xx_id_mutex);
static DEFINE_MUTEX(maxim59xx_timer_mutex);
static DEFINE_MUTEX(maxim59xx_i2c_mutex);

enum maxim59xx_chip {
	MAXIM_UNKNOWN = 0,
	MAXIM_5971B = 0x80,
};

enum maxim59xx_registers {
	MAXIM_5971B_INT 				= 0x0,
	MAXIM_5971B_INT_MASK			= 0x1,
	MAXIM_5971B_POWER_EVENT			= 0x2,
	MAXIM_5971B_POWER_EVENT_COR		= 0x3,
	MAXIM_5971B_DETECT_EVENT		= 0x4,
	MAXIM_5971B_DETECT_EVENT_COR	= 0x5,
	MAXIM_5971B_FAULT_EVENT			= 0x6,
	MAXIM_5971B_FAULT_EVENT_COR		= 0x7,
	MAXIM_5971B_STARTUP_EVENT		= 0x8,
	MAXIM_5971B_STARTUP_EVENT_COR 	= 0x9,
	MAXIM_5971B_SUPPLY_EVENT		= 0xA,
	MAXIM_5971B_SUPPLY_EVENT_COR	= 0xB,
	MAXIM_5971B_PORT_STATUS			= 0xC,
	MAXIM_5971B_POWER_STATUS		= 0x10,
	MAXIM_5971B_PIN_STATUS			= 0x11,
	MAXIM_5971B_OP_MODE				= 0x12,
	MAXIM_5971B_DISCONNECT_ENABLE	= 0x13,
	MAXIM_5971B_DET_CLASS_ENABLE	= 0x14,
	MAXIM_5971B_BACKOFF_ENABLE		= 0x15,
	MAXIM_5971B_TIMING_CFG			= 0x16,
	MAXIM_5971B_MISC_CONF_1			= 0x17,
	MAXIM_5971B_POWER_ENABLE		= 0x19,
	MAXIM_5971B_GLOBAL				= 0x1A,
	MAXIM_5971B_ID					= 0x1B,
	MAXIM_5971B_SMODE				= 0x1C,
	MAXIM_5971B_WATCHDOG			= 0x1E,
	MAXIM_5971B_SWITCH_MODE			= 0x1F,
	MAXIM_5971B_PROGRAM				= 0x23,
	MAXIM_5971B_PWM					= 0x24,
	MAXIM_5971B_MISC_CONF_2			= 0x29,
	MAXIM_5971B_ICUT				= 0x2A,
	MAXIM_5971B_PORT_CURRENT		= 0x30,
	MAXIM_5971B_PORT_CURRENT_LSB	= 0x31,
};
/*
struct maxim59xx_device_info {
	char* name;
	enum maxim59xx_chip id;
	uint8_t ports;
};

static struct maxim59xx_device_info[] = {
	{ "MAXIM_5971B", MAXIM_5971B, 1 },
};
*/
struct maxim59xx_platform_data {
	int current_limit;
	int current_value;
	int voltage;
};

struct maxim59xx_device {
	struct device *dev;
	struct maxim59xx_platform_data pdata;
	int id;
	char* name;
	struct work_struct work;
	uint32_t irq_parent;
	uint32_t irq_gpio;
	unsigned irq_enable;
	enum maxim59xx_chip chip;
	struct power_supply ps;
};

static uint8_t __read_byte(const struct i2c_client *client, u8 command) {
	union i2c_smbus_data data;
	int status = 0;
	int count = 0;
	do { 
		status = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data
		);
		count++;
	} while( status != 0 && count < 10 );
	
	if( count >= 10 ) {
		dev_err(&client->dev, "read failed with status: %d\n", status);
		return 0xFF;
	}

	return data.byte;
}

static uint8_t __dev_get_value( struct device *dev, enum maxim59xx_registers reg) {
	struct power_supply* ps;
	struct maxim59xx_device* d;
	struct i2c_client *client; 
	
	ps = dev_get_drvdata(dev);
	d = container_of(ps,struct maxim59xx_device, ps);
	client = to_i2c_client(d->dev);

	return __read_byte(client, reg);
}

static uint8_t __dev_set_value( struct device *dev, enum maxim59xx_registers reg, uint8_t value) {
	struct power_supply* ps;
	struct maxim59xx_device* d;
	struct i2c_client *client; 
	
	ps = dev_get_drvdata(dev);
	d = container_of(ps,struct maxim59xx_device, ps);
	client = to_i2c_client(d->dev);

	return __read_byte(client, reg);
}

static irqreturn_t maxim59xx_irq_handler( int irq, void* data){
	struct maxim59xx_device* d = data;
	
	schedule_work(&d->work);

	return IRQ_HANDLED;
}

static int maxim59xx_detect(struct maxim59xx_device *dev) {
	struct i2c_client *client = to_i2c_client(dev->dev);
	int ret = i2c_smbus_read_byte_data(client, MAXIM_5971B_ID);

	return ret;
}

/* set current and voltage limit entries */
static ssize_t maxim59xx_sysfs_set_limit(struct device *dev,
								struct device_attribute *attr,
								const char *buf,
								size_t count) {
	uint8_t r = buf[0] - 48;

	if( r > 7  || r == 4 )
		return -EINVAL;
	
	dev_dbg(dev, "%d limit set\n", r);

	__dev_set_value(dev, MAXIM_5971B_ICUT, r);

	return count;
}


/* show current and voltage limit entries (in mA or mV) */
static ssize_t maxim59xx_sysfs_show_limit(struct device *dev,
							struct device_attribute *attr,
												char *buf)
{
	char nums[8][4];
	int i;
	uint8_t icut;
	icut = __dev_get_value(dev, MAXIM_5971B_ICUT);
	

	for( i=0; i < 8; i++ ) {
		nums[i][1]=('0'+i);
		if (i == icut) {
			nums[i][0]='[';
			nums[i][2]=']';
		} else {
			nums[i][0]=' ';
			nums[i][2]=' ';
		}
		nums[i][3]='\0';
	}

	return snprintf(buf, PAGE_SIZE, "NUMBER  LIMIT  THRESHOLD\n"\
									" %s     420      370   \n"\
									" %s     720      634   \n"\
									" %s     126      111   \n"\
									" %s     223      196   \n"\
									" %s     N/A      N/A   \n"\
									" %s     850      748   \n"\
									" %s     900      792   \n"\
									" %s     950      836   \n"
		, nums[0],nums[1],nums[2],nums[3],nums[4],nums[5],nums[6],nums[7]);
}
/* Automatic current limit */
static ssize_t maxim59xx_sysfs_set_autolimit(struct device *dev,
								struct device_attribute *attr,
								const char *buf,
								size_t count) {
	uint8_t r, conf;

	r = buf[0] - 48;
	
	if( r > 1 )
		return -EINVAL;

	conf = __dev_get_value(dev, MAXIM_5971B_MISC_CONF_1);

	if( !r ) {
		conf |= (1 << 2);
	} else {
		conf &= ~(1 <<  2);
	}

	__dev_set_value(dev, MAXIM_5971B_MISC_CONF_1, conf);

	return count;
}

/* show current and voltage limit entries (in mA or mV) */
static ssize_t maxim59xx_sysfs_show_autolimit(struct device *dev,
							struct device_attribute *attr,
												char *buf)
{
	uint8_t misc;

	misc = __dev_get_value(dev, MAXIM_5971B_MISC_CONF_1);

	return snprintf(buf, PAGE_SIZE, "%d\n", ((misc & (1 << 2)) ? 0 : 1));
}
static ssize_t maxim59xx_sysfs_show_voltage(struct device *dev,
							struct device_attribute *attr,
												char *buf)
{
	return snprintf(buf, PAGE_SIZE, "54000000\n");
}

static ssize_t maxim59xx_sysfs_show_current(struct device *dev,
							struct device_attribute *attr,
												char *buf)
{
	uint8_t status, ret;
	uint16_t ct, value;
	
	status = __dev_get_value(dev, MAXIM_5971B_PORT_STATUS);
	ret = __dev_get_value(dev, MAXIM_5971B_PORT_CURRENT);
	
	ct = ret << 1;
	ret = __dev_get_value(dev, MAXIM_5971B_PORT_CURRENT_LSB);

	ct = ct + (ret & 0x01);
	if( status & 0x04 ) {
		value = ct * 1950;
	} else {
		value = ct * 98;
	}
	
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t maxim59xx_sysfs_show_class(struct device *dev,
							struct device_attribute *attr,
												char *buf)
{
	uint8_t status;
	uint8_t class;
	char* str_class;

	status = __dev_get_value(dev, MAXIM_5971B_PORT_STATUS);

	class = (status & 0x70) >> 4;

	switch( class ) {
		case 0:
			str_class = "Unknown";
			break;
		case 1:
			str_class = "1";
			break;
		case 2:
			str_class = "2";
			break;
		case 3:
			str_class = "3";
			break;
		case 4:
			str_class = "4";
			break;
		case 5:
			str_class = "5";
			break;
		case 6:
			str_class = "0";
			break;
		case 7:
		default:
			str_class = "FAIL!";
			break;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", str_class );
}

static ssize_t maxim59xx_sysfs_show_det(struct device *dev,
							struct device_attribute *attr,
												char *buf)
{

	uint8_t status;
	uint8_t det;
	char* str_det;

	status = __dev_get_value(dev, MAXIM_5971B_PORT_STATUS);

	det = status & 0x7;

	switch ( det ) {
		case 1:
			str_det = "DCP";
			break;
		case 2:
			str_det = "HIGH CAP";
			break;
		case 3:
			str_det = "RLOW";
			break;
		case 4:
			str_det = "OK";
			break;
		case 5 :
			str_det = "RHIGH";
			break;
		case 6 :
			str_det = "OPEN";
			break;
		case 7 :
			str_det = "DCN";
			break;
		default:
		case 0:
			str_det = "None";
			break;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", str_det );
}

/* Reset port */
static ssize_t maxim59xx_sysfs_set_port_reset(struct device *dev,
								struct device_attribute *attr,
								const char *buf,
								size_t count) {
	int8_t ret;
	uint8_t r;

	r = buf[0] - 48;
	
	if( r != 1 )
		return -EINVAL;

	ret = __dev_set_value(dev, MAXIM_5971B_GLOBAL, 0x1);

	if( ret ) {
		dev_err(dev,"Port reset failed with errno %d\n", ret);
		return ret;
	}

	return count;
}

static ssize_t maxim59xx_sysfs_shutdown(struct device *dev,
								struct device_attribute *attr,
								const char *buf,
								size_t count) {
	int8_t ret;
	uint8_t r;

	r = buf[0] - 48;
	
	if( r ) {
		ret = __dev_set_value(dev, MAXIM_5971B_OP_MODE, 0x3 );
	} else {
		ret = __dev_set_value(dev, MAXIM_5971B_OP_MODE, 0x0 );
	}

	if( ret ) {
		dev_err(dev,"Shutdown failed with errno %d\n", ret);
		return ret;
	}

	return count;
}
#define __current current
#undef current
static DEVICE_ATTR(current_limit, S_IWUSR | S_IRUGO, maxim59xx_sysfs_show_limit, maxim59xx_sysfs_set_limit);
static DEVICE_ATTR(voltage, S_IRUGO, maxim59xx_sysfs_show_voltage, NULL);
static DEVICE_ATTR(current, S_IRUGO, maxim59xx_sysfs_show_current, NULL);
static DEVICE_ATTR(class, S_IRUGO, maxim59xx_sysfs_show_class, NULL);
static DEVICE_ATTR(detection, S_IRUGO, maxim59xx_sysfs_show_det, NULL);
static DEVICE_ATTR(autolimit, S_IRUGO|S_IWUSR, maxim59xx_sysfs_show_autolimit, maxim59xx_sysfs_set_autolimit);
static DEVICE_ATTR(port_reset,S_IWUSR, NULL, maxim59xx_sysfs_set_port_reset);
static DEVICE_ATTR(shutdown,S_IWUSR, NULL, maxim59xx_sysfs_shutdown);

static struct attribute *maxim59xx_sysfs_attributes[] = {
	&dev_attr_current_limit.attr,
	&dev_attr_current.attr,
	&dev_attr_voltage.attr,
	&dev_attr_class.attr,
	&dev_attr_detection.attr,
	&dev_attr_autolimit.attr,
	&dev_attr_port_reset.attr,
	&dev_attr_shutdown.attr,
	NULL
};

#define current __current

static const struct attribute_group maxim59xx_sysfs_attr_group = {
	.attrs = maxim59xx_sysfs_attributes,
};

static enum power_supply_property maxim59xx_power_supply_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int maxim59xx_power_supply_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val) {
	struct maxim59xx_device *dev = container_of(psy, struct maxim59xx_device, ps);
	struct i2c_client* client = to_i2c_client(dev->dev);
	switch (psp) {
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval=54000000;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		{
			uint8_t status = __read_byte(client, MAXIM_5971B_PORT_STATUS);
			uint8_t ret = __read_byte(client, MAXIM_5971B_PORT_CURRENT);
			uint16_t ct;
			ct = ret << 1;
			ret = __read_byte(client, MAXIM_5971B_PORT_CURRENT_LSB);

			ct = ct + (ret & 0x01);
			if( status & 0x04 ) {
				val->intval = ct * 1950;
			} else {
				val->intval = ct * 98;
			}
			break;
		}
		default:
			return -EINVAL;
	}

	return 0;
}

static int maxim59xx_power_supply_init( struct maxim59xx_device *dev ) {
	int ret;
	dev_dbg(dev->dev, "power supply init in\n");
	dev->ps.name = dev->name;
	dev->ps.type = POWER_SUPPLY_TYPE_MAINS;
	dev->ps.properties = maxim59xx_power_supply_props;
	dev->ps.num_properties = ARRAY_SIZE(maxim59xx_power_supply_props);
	dev->ps.get_property = maxim59xx_power_supply_get_property;
//	dev->ps.set_property = maxim59xx_power_supply_set_property;
//	dev->ps.property_is_writeable = maxim59xx_power_supply_property_is_writeable;

	ret = maxim59xx_detect(dev);

	if( ret < 0 ) {
		dev->chip = MAXIM_UNKNOWN;
	} else {
		dev->chip = ret & 0xF8;
	}
	
	ret = power_supply_register(dev->dev, &dev->ps);

	if (ret) {
		dev_dbg(dev->dev, "power supply init error\n");
		return ret;
	}

	dev_dbg(dev->dev, "power supply init out\n");
	return 0;
}

static int maxim59xx_power_supply_exit( struct maxim59xx_device *dev ) {
	power_supply_unregister(&dev->ps);
	return 0;
}

static int maxim59xx_sysfs_init(struct maxim59xx_device *dev){
	dev_dbg(dev->dev, "sysfs init\n");
	return sysfs_create_group(&dev->ps.dev->kobj, &maxim59xx_sysfs_attr_group);
}

static int maxim59xx_sysfs_exit(struct maxim59xx_device *dev){
	sysfs_remove_group(&dev->ps.dev->kobj, &maxim59xx_sysfs_attr_group);

	return 0;
}

static int maxim59xx_event_add_var(struct sk_buff *b, const char *format, ...)
{
	static char buf[128];
	char *s;
	va_list args;
	int len;

	va_start(args, format);
	len = vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	if (len >= sizeof(buf)) {
		return -ENOMEM;
	}

	s = skb_put(b, len + 1);
	strcpy(s, buf);

	return 0;
}

static int maxim59xx_fill_event(struct sk_buff* b) {
	int ret;

	ret = maxim59xx_event_add_var(b, "HOME=%s","/");
	if( ret ) return ret;
	
	ret = maxim59xx_event_add_var(b, "PATH=%s","/sbin:/bin:/usr/sbin:/usr/bin");
	if( ret ) return ret;
	
	ret = maxim59xx_event_add_var(b, "SUBSYSTEM=%s","power");
	if( ret ) return ret;
	
	ret = maxim59xx_event_add_var(b, "SEQNUM=%llu", uevent_next_seqnum());

	return ret;
}

static void maxim59xx_timer_work(struct work_struct *work){
	struct maxim59xx_device *dev = container_of(work, struct maxim59xx_device, work);
	struct i2c_client* client = to_i2c_client(dev->dev);
	int8_t ret;
	uint8_t cnt = 0;
	struct sk_buff *b;
	uint8_t status = __read_byte(client, MAXIM_5971B_INT);

	b = alloc_skb(2048, GFP_KERNEL);
	ret = maxim59xx_fill_event(b);

	if( ret != 0 ) {
		goto out_skb;
	}

	if( status & 0x1 ) {
		ret = maxim59xx_event_add_var(b, "ACTION=%s","P_EN");
		if( ret ) { 
			dev_dbg(dev->dev, "Variable P_EN addition failed %d\n", ret);
			goto out_skb;
		}
	}
	if( status & 0x2 ) {
		ret = maxim59xx_event_add_var(b, "ACTION=%s","PGOOD");
		if( ret ) { 
			dev_dbg(dev->dev, "Variable PGOOD addition failed %d\n", ret);
			goto out_skb;
		}
	}
	if( status & 0x4 ) {
		ret = maxim59xx_event_add_var(b, "ACTION=%s","LD");
		if( ret ) { 
			dev_dbg(dev->dev, "Variable LD addition failed %d\n", ret);
			goto out_skb;
		}
	}
	if( status & 0x8 ) {
		ret = maxim59xx_event_add_var(b, "ACTION=%s","DET");
		if( ret ) { 
			dev_dbg(dev->dev, "Variable DET addition failed %d\n", ret);
			goto out_skb;
		}
	}
	if( status & 0x10 ) {
		ret = maxim59xx_event_add_var(b, "ACTION=%s","CLASS");
		if( ret ) { 
			dev_dbg(dev->dev, "Variable CLASS addition failed %d\n", ret);
			goto out_skb;
		}
	}
	if( status & 0x20 ) {
		ret = maxim59xx_event_add_var(b, "ACTION=%s","OVERCURRENT");
		if( ret ) { 
			dev_dbg(dev->dev, "Variable OVERCURRENT addition failed %d\n", ret);
			goto out_skb;
		}
	}
	if( status & 0x40 ) {
		ret = maxim59xx_event_add_var(b, "ACTION=%s","IVC");
		if( ret ) { 
			dev_dbg(dev->dev, "Variable IVC addition failed %d\n", ret);
			goto out_skb;
		}
	}
	if( status & 0x80 ) {
		ret = maxim59xx_event_add_var(b, "ACTION=%s","SUP");
		if( ret ) { 
			dev_dbg(dev->dev, "Variable SUP addition failed %d\n", ret);
			goto out_skb;
		}
	}
	

	NETLINK_CB(b).dst_group = 1;
	broadcast_uevent(b, 0, 1, GFP_KERNEL);
	do{
		ret = i2c_smbus_write_byte_data(client, MAXIM_5971B_GLOBAL, 0x80);
		cnt++;
	} while( ret < 0 && cnt < 5 );
	
	if( ret ) {	
		dev_dbg(dev->dev, "Interrupt clear failed. Errno: %d\n", ret);
	}

	return;
out_skb:
	kfree_skb(b);
	return;
}

static int maxim59xx_exit( struct maxim59xx_device *dev) {
	if( dev->irq_parent ) {
		free_irq(dev->irq_parent, dev);
		gpio_free(dev->irq_gpio);
	}
	
	cancel_work_sync(&dev->work);
	
	return 0;
}

static int maxim59xx_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {
	int ret;
	int num;
	char *name;
	struct maxim59xx_device *dev;
	struct device_node *np = client->dev.of_node;
	struct maxim59xx_platform_data *pdata = client->dev.platform_data;
	int int_gpio = 0; 

	dev_dbg(&client->dev, "maxim59xx probe\n");
	if (!np && !pdata) {
		dev_err(&client->dev, "platform data missing\n");
		return -ENODEV;
	}

	mutex_lock(&maxim59xx_id_mutex);
	num = idr_alloc(&maxim59xx_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&maxim59xx_id_mutex);

	if (num < 0)
		return num;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		ret = -ENOMEM;
		goto err;
	}

	dev = devm_kzalloc(&client->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "failed to allocate device data\n");
		ret = -ENOMEM;
		goto err_1;
	}

	dev_dbg(&client->dev, "maxim 5971 allocated mem\n");
	i2c_set_clientdata(client, dev);

	// probe chip on bus
	ret = __read_byte(client, MAXIM_5971B_ID);

	switch( ret & 0xF8 ) {
		case MAXIM_5971B:
			//do something
			break;
		default:
			dev_err(&client->dev, "Get unknown chip ID 0x%X\n", ret);
			ret = -ENODEV;
			goto err_1;
			break;
	}
	dev_info(&client->dev, "Found device with ID 0x%X rev. 0x%X\n", ret & 0xF8, ret & 0x07);
	INIT_WORK(&dev->work, maxim59xx_timer_work);

	dev->id = num;
	dev->dev = &client->dev;
	dev->chip = id->driver_data;
	dev->name = name;


	int_gpio = of_get_named_gpio(np, "int", 0);
	dev_info(&client->dev, "int_gpio %d\n", int_gpio);
	if( int_gpio > 0 ) {
		int irq;
		ret = gpio_is_valid(int_gpio);
		if( !ret ) {
			dev_err(&client->dev, "invalid gpio %d for interrupt\n", int_gpio);
			goto err_2;
		}
		
		dev_info(&client->dev, "gpio for interrupt: %d", int_gpio);
		ret = gpio_request(int_gpio, "poe_int");
		if( ret ) {
			dev_err(&client->dev, "gpio %d request failed\n", int_gpio);
			goto err_2;
		}

		dev_info(&client->dev, "gpio %d requested \n", int_gpio);
		dev->irq_gpio = int_gpio;
		
		gpio_direction_input(int_gpio);

		irq = gpio_to_irq(int_gpio);

		if( irq < 0 ) {
			ret = irq;
			dev_err(&client->dev, "Can't get irq for %d gpio\n", int_gpio);
			goto err_2;
		}

		dev_info(&client->dev, "IRQ: %d", irq);

		ret = request_irq( irq, maxim59xx_irq_handler, IRQF_TRIGGER_FALLING, name, dev);

		if( ret ) {
			dev_err(&client->dev, "Can't request irq %d\n", int_gpio);
			goto err_2;
		}

		dev_info(&client->dev, "IRQ: %d requested\n", irq);

		dev->irq_parent = irq;
		ret = __read_byte(client, MAXIM_5971B_INT_MASK);

		dev_info(&client->dev, "Interrupt mask: 0x%02X\n", ret);
		i2c_smbus_write_byte_data(client, MAXIM_5971B_INT_MASK, 0xFF);
		i2c_smbus_write_byte_data(client, MAXIM_5971B_GLOBAL, 0x80);
	} else if( int_gpio == 0 ) {
		ret = -EINVAL;
		goto err_1;
	} else {
		ret = int_gpio;
		goto err_1;
	}

	// if in devtree set interrupt pin, use interrupt. 
	// If not, then setup delayed work
	
	ret = maxim59xx_power_supply_init(dev);

	if (ret) {
		dev_err(dev->dev, "failed to register power supply: %d\n", ret);
		goto err_3;
	}

	ret = maxim59xx_sysfs_init(dev);
	if (ret) {
		dev_err(dev->dev, "failed to create sysfs entries: %d\n", ret);
		goto err_4;
	}

	return 0;
err_4:
	maxim59xx_exit(dev);
err_3:
	maxim59xx_power_supply_exit(dev);
err_2:
	if( int_gpio > 0 ) { gpio_free(int_gpio); }
err_1:
	kfree(name);
err:
	mutex_lock(&maxim59xx_id_mutex);
	idr_remove(&maxim59xx_id, num);
	mutex_unlock(&maxim59xx_id_mutex);
	return ret;
}

static int maxim59xx_remove(struct i2c_client *client)
{
	struct maxim59xx_device *dev = i2c_get_clientdata(client);

	maxim59xx_sysfs_exit(dev);
	maxim59xx_exit(dev);
	maxim59xx_power_supply_exit(dev);

	mutex_lock(&maxim59xx_id_mutex);
	idr_remove(&maxim59xx_id, dev->id);
	mutex_unlock(&maxim59xx_id_mutex);

	dev_info(dev->dev, "driver unregistered\n");

	kfree(dev->name);

	return 0;
}

static const struct i2c_device_id maxim59xx_ids[] = {
	{ "5971b", MAXIM_5971B },
	{},
};
MODULE_DEVICE_TABLE(i2c, maxim59xx_ids );

static const struct of_device_id maxim59xx_of_match[] = {
	{ .compatible = "maxim,5971b" },
	{},
};
MODULE_DEVICE_TABLE(of, maxim59xx_of_match);

static struct i2c_driver maxim59xx_driver = {
	.driver = {
		.name = "MAX59xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(maxim59xx_of_match),
	},
	.probe = maxim59xx_probe,
	.remove = maxim59xx_remove,
	.id_table = maxim59xx_ids,
};
module_i2c_driver(maxim59xx_driver);

MODULE_AUTHOR("Nikita Nazarenko <nnazarenko@radiofid.ru>");
MODULE_DESCRIPTION("MAXIM 59XX Power over Ethernet PSE driver");
MODULE_LICENSE("GPL");
