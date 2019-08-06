/*
 * Linux Device Driver for communication with 1-Wire DS18B20 thermal sensor
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/kobject.h>

#define W1_CONVERT_T				0X44
#define W1_READ_SCRATCHPAD			0xBE
#define W1_SKIP_ROM				0xCC
#define W1_GPIO_DQ				17
#define LED_GREEN_GPIO				23
#define LED_YELLOW_GPIO				22
#define LED_RED_GPIO				27
#define TEMP_DECIMAL_STEPS_12BIT		625
#define TEMP_NORMAL				24
#define TEMP_WARM				27
#define TEMP_HOT				30

static struct temperature {
	int digit;
	int decimal;
	int range;
};

static struct temperature cur_temp;
static struct temperature prev_temp;

/* Invoked on read of sysfs file */
static ssize_t temp_show(struct kobject *kobj, struct attribute *attr, char *buf);

static ssize_t temp_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "Current temperature is %d.%04d C.\n",
					cur_temp.digit, cur_temp.decimal);
}

static struct kobj_attribute temp_attr = __ATTR_RO(temp);
static struct attribute *temp_attrs[] = {
	&temp_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name = "gpio17",
	.attrs = temp_attrs,
};

static struct kobject *temp_kobj;

/* States of state machine used with the LEDs */
typedef enum {
	INIT,
	NORMAL,
	WARM,
	HOT
}state_t;

/* Events of state machine used with the LEDs */
typedef enum {
	TEMP_DROPS_TO_NORMAL,
	TEMP_DROPS_TO_WARM,
	TEMP_RISES_TO_NORMAL,
	TEMP_RISES_TO_WARM,
	TEMP_RISES_TO_HOT,
	TEMP_SAME
}event_t;

static struct state_trans_t {
	state_t state;
	event_t event;
	void (*funcptr)(void);
};

static int gpio_setup(void);
static event_t get_event(void);
static event_t event;
static state_t cur_state;
static void light_green(void);
static void light_yellow(void);
static void light_red(void);
static struct work_struct *work;
static void work_handler(void);
static int reschedule;

static struct state_trans_t state_trans[] = {
	{INIT, TEMP_RISES_TO_NORMAL, light_green},
	{INIT, TEMP_RISES_TO_WARM, light_yellow},
	{INIT, TEMP_RISES_TO_HOT, light_red},
	{NORMAL, TEMP_RISES_TO_WARM, light_yellow},
	{NORMAL, TEMP_RISES_TO_HOT, light_red},
	{WARM, TEMP_DROPS_TO_NORMAL, light_green},
	{WARM, TEMP_RISES_TO_HOT, light_red},
	{HOT, TEMP_DROPS_TO_WARM, light_yellow},
	{HOT, TEMP_DROPS_TO_NORMAL, light_green}
};

struct w1_device {
	void (*write_bit)(char value);
	char (*read_bit)(void);
	void (*write_byte)(char value);
	char (*read_byte)(void);
	unsigned char (*reset)(void);
	void (*read_temperature)(void);
};

static struct w1_device *w1_ds18b20;

/* Device specific operations functions */
static void w1_write_bit(char value);
static char w1_read_bit(void);
static void w1_write_byte(char value);
static char w1_read_byte(void);
static unsigned char w1_reset(void);
static void w1_read_temperature(void);

static void w1_write_bit(char value)
{
	unsigned long write_timeslot_useconds = 60;
	gpio_direction_output(W1_GPIO_DQ, 0);
	udelay(1);
	if (1 == value) {
		gpio_direction_input(W1_GPIO_DQ);
	}
	udelay(write_timeslot_useconds);
	gpio_direction_input(W1_GPIO_DQ);
}

static char w1_read_bit(void)
{
	unsigned char bit;
	unsigned long read_timeslot_useconds = 14;
	gpio_direction_output(W1_GPIO_DQ, 0);
	udelay(1);
	gpio_direction_input(W1_GPIO_DQ);
	udelay(read_timeslot_useconds);
	bit = gpio_get_value(W1_GPIO_DQ);
	udelay(45);
	return bit;
}

static void w1_write_byte(char value)
{
	unsigned char i;
	for (i = 0; i < 8; i++)
	{
		w1_ds18b20->write_bit((value >> i) & 0x01);
	}
}

static char w1_read_byte(void)
{
	unsigned char i;
	unsigned char value = 0;
	for (i = 0; i < 8; i++)
	{
		value |= (w1_ds18b20->read_bit() << i);
	}

	return value;
}

/* 
 * To wake up the DS18B20 a reset signal should be sent to the slave.
 * The bus is pulled low for at least 480 us and then released. 
 * The slave then issues a presence signal(pulling the bus low) if present.
 */
static unsigned char w1_reset(void)
{
	unsigned char presence = 0;
	unsigned long reset_pulse_useconds = 480;
	unsigned long presence_pulse_useconds = 60;
	gpio_direction_output(W1_GPIO_DQ, 0); // Pull line low and wait for 480 us
	udelay(reset_pulse_useconds);
	gpio_direction_input(W1_GPIO_DQ); // Release line and wait for 60 us
	udelay(presence_pulse_useconds);

	presence = gpio_get_value(W1_GPIO_DQ); // Read line value (0 = OK, 1 = NOT PRESENT)
	udelay(420);

	return presence;
}

static void w1_read_temperature(void)
{
	char w1_scratchpad[2];
	int i;

	/* Issue Convert T command in order to start converting the temperature */
	w1_ds18b20->reset();
	w1_ds18b20->write_byte(W1_SKIP_ROM);
	w1_ds18b20->write_byte(W1_CONVERT_T);

	while(!w1_ds18b20->read_bit());	// Wait untill conversion is done

	/* Issue Read Scratchpad command in order to read the scratchpad, where data is located */
	w1_ds18b20->reset();
	w1_ds18b20->write_byte(W1_SKIP_ROM);
	w1_ds18b20->write_byte(W1_READ_SCRATCHPAD);

	/* Read only the first two bytes (Temperature bytes) */
	for (i = 0; i < 2; i++) {
		w1_scratchpad[i] = w1_ds18b20->read_byte();
	}

	cur_temp.digit = w1_scratchpad[0] >> 4;
	cur_temp.digit |= (w1_scratchpad[1] & 0x7) << 4; 

	if (cur_temp.digit >= TEMP_NORMAL && cur_temp.digit < TEMP_WARM) {
		cur_temp.range = TEMP_NORMAL;
	} else if (cur_temp.digit >= TEMP_WARM && cur_temp.digit < TEMP_HOT) {
		cur_temp.range = TEMP_WARM;
	} else {
		cur_temp.range = TEMP_HOT;
	}

	cur_temp.decimal = (w1_scratchpad[0]&0xf) * TEMP_DECIMAL_STEPS_12BIT;
}

static event_t get_event(void)
{
	if (cur_temp.range > prev_temp.range) {
		prev_temp.range = cur_temp.range;
		switch (cur_temp.range) {
			case TEMP_NORMAL:
				return TEMP_RISES_TO_NORMAL;
			case TEMP_WARM:
				return TEMP_RISES_TO_WARM;
			default:
				return TEMP_RISES_TO_HOT;
		}
	} else if (cur_temp.range < prev_temp.range) {
		prev_temp.range = cur_temp.range;
		switch (cur_temp.range) {
			case TEMP_NORMAL:
				return TEMP_DROPS_TO_NORMAL;
			default:
				return TEMP_DROPS_TO_WARM;
		}
    } else {
		return TEMP_SAME;
	}
}

static int gpio_setup(void)
{
	int ret;

	ret = gpio_request(W1_GPIO_DQ, "gpio_ds18b20");
	if (ret < 0) {
		return ret;
	}
	ret = gpio_request(LED_GREEN_GPIO, "green_led_gpio");
	if (ret < 0) {
		gpio_free(W1_GPIO_DQ);
		return ret;
	}
	ret = gpio_request(LED_YELLOW_GPIO, "yellow_led_gpio");
	if (ret < 0) {
		gpio_free(W1_GPIO_DQ);
		gpio_free(LED_GREEN_GPIO);
		return ret;
	}
	ret = gpio_request(LED_RED_GPIO, "red_led_gpio");
	if (ret < 0) {
		gpio_free(W1_GPIO_DQ);
		gpio_free(LED_GREEN_GPIO);
		gpio_free(LED_YELLOW_GPIO);
		return ret;
	}

	gpio_direction_output(LED_GREEN_GPIO, 0);
	gpio_direction_output(LED_YELLOW_GPIO, 0);
	gpio_direction_output(LED_RED_GPIO, 0);

	return ret;
}

static void light_green(void)
{
	gpio_set_value(LED_RED_GPIO, 0);
	gpio_set_value(LED_YELLOW_GPIO, 0);

	gpio_set_value(LED_GREEN_GPIO, 1);

	cur_state = NORMAL;
}

static void light_yellow(void)
{
	gpio_set_value(LED_RED_GPIO, 0);
	gpio_set_value(LED_GREEN_GPIO, 0);

	gpio_set_value(LED_YELLOW_GPIO, 1);

	cur_state = WARM;
}

static void light_red(void)
{
	gpio_set_value(LED_GREEN_GPIO, 0);
	gpio_set_value(LED_YELLOW_GPIO, 0);

	gpio_set_value(LED_RED_GPIO, 1);

	cur_state = HOT;
}

static void work_handler(void)
{
	int i, ret;
	int struct_size = sizeof(state_trans) / sizeof(state_trans[0]);
	w1_ds18b20->read_temperature();
	event = get_event();
	for (i = 0; i < struct_size; i++) {
		if (cur_state == state_trans[i].state) {
			if (event == state_trans[i].event) {
				(*state_trans[i].funcptr)();
				break;
			}
		}
	}

	mdelay(2000);

	if (reschedule) {
		ret = schedule_work((struct work_struct*) work);
	}
}

static int __init w1_ds18b20_init(void)
{
	pr_err("%s Initializing the module.\n", __func__);

	int ret;

	w1_ds18b20 = (struct w1_device*) kmalloc(sizeof(w1_ds18b20), GFP_KERNEL);
	if(!w1_ds18b20) {
		pr_err("%s Could not allocate memory.\n", __func__);
		return -ENOMEM;
	}

	w1_ds18b20->read_bit         = w1_read_bit;
	w1_ds18b20->read_byte        = w1_read_byte;
	w1_ds18b20->write_bit        = w1_write_bit;
	w1_ds18b20->write_byte       = w1_write_byte;
	w1_ds18b20->reset            = w1_reset;
	w1_ds18b20->read_temperature = w1_read_temperature;

	if (!gpio_is_valid(W1_GPIO_DQ) || !gpio_is_valid(LED_GREEN_GPIO) ||
			!gpio_is_valid(LED_YELLOW_GPIO) || !gpio_is_valid(LED_RED_GPIO)) {
		pr_err("%s GPIOs not valid.\n", __func__);
		kfree(w1_ds18b20);
		return -ENODEV;
	}

	ret = gpio_setup();
	if (ret < 0) {
		pr_err("%s GPIO request failed.\n", __func__);
		kfree(w1_ds18b20);
		return ret;
	}

	cur_state = INIT;
	cur_temp.range  = 0;
	prev_temp.range = 0;

	temp_kobj = kobject_create_and_add("ds18b20", kernel_kobj->parent);
	if (!temp_kobj) {
		pr_err("%s Failed to create kobject mapping.\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_group(temp_kobj, &attr_group);
	if (ret) {
		pr_err("%s Failed to create sysfs group.\n", __func__);
		kobject_put(temp_kobj);
		kfree(w1_ds18b20);
		return ret;
	}

	work = (struct work_struct*) kmalloc(sizeof(work), GFP_KERNEL);
	if (work) {
		reschedule = 1;
		INIT_WORK((struct work_struct*) work, work_handler);
		ret = schedule_work((struct work_struct*) work);
	}

	return 0;
}

static void __exit w1_ds18b20_exit(void)
{
	reschedule = 0;
	cancel_work_sync(work);
	kobject_put(temp_kobj);
	kfree((void *) work);
	kfree(w1_ds18b20);
	gpio_free(W1_GPIO_DQ);
	gpio_free(LED_GREEN_GPIO);
	gpio_free(LED_YELLOW_GPIO);
	gpio_free(LED_RED_GPIO);
	pr_err("%s Bye Bye.\n", __func__);
}

MODULE_AUTHOR("Martin Ponev");
MODULE_LICENSE("GPL");

module_init(w1_ds18b20_init);
module_exit(w1_ds18b20_exit);
