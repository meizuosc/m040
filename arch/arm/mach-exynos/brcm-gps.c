/*
 * Broadcom gps driver, Author: heljoy liu [heljoy@meizu.com]
 * This is based UBLOX gps driver of jerrymo@meizu.com
 * Copyright (c) 2010 meizu Corporation
 * 
 */
 
#define pr_fmt(fmt)	"BRCM_GPS: " fmt

#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/gpio.h>
#include <mach/gpio-common.h>
#include <mach/gpio-m040.h>
#include <plat/gpio-cfg.h>

/*
 * gps_is_runnig shows uart channel state
 * value sets to 0 when channel shutdown, 1 when startup
 * this is used when cpu enters LPA mode
 */
int gps_is_running;

/*gps driver private data struct*/
struct gps_data {
	int gps_power;
	int gps_reset;
};

static ssize_t gps_running_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",  gps_is_running);
}

static ssize_t gps_power_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int value;
	struct gps_data *data = dev_get_drvdata(dev);
	
	value = gpio_get_value(data->gps_power);
	pr_debug("%s():power %d\n", __func__, value);
	return sprintf(buf, "%d\n",  value);
}

static ssize_t gps_power_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
	struct gps_data *data = dev_get_drvdata(dev);

	pr_debug("%s():power %ld.\n", __func__, value);

	gpio_set_value(data->gps_power, !!value);
		
	return count;
}

static ssize_t gps_reset_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int value;
	struct gps_data *data = dev_get_drvdata(dev);
	
	value = gpio_get_value(data->gps_reset);
	pr_debug("%s():reset %d\n", __func__, value);
	return sprintf(buf, "%d\n",  value);
}


static ssize_t gps_reset_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
	struct gps_data *data = dev_get_drvdata(dev);

	pr_debug("%s():reset %ld.\n", __func__, value);

	gpio_set_value(data->gps_reset, !!value);
		
	return count;
}
static DEVICE_ATTR(running, S_IRUGO | S_IWUSR,
		   gps_running_show, NULL);

static DEVICE_ATTR(pwr, S_IRUGO | S_IWUSR,
		   gps_power_show, gps_power_store);

static DEVICE_ATTR(rst, S_IRUGO | S_IWUSR,
		   gps_reset_show, gps_reset_store);

static struct attribute *gps_attributes[] = {
	&dev_attr_pwr.attr,
	&dev_attr_rst.attr,
	&dev_attr_running.attr,
	NULL
};

static struct attribute_group gps_attribute_group = {
	.attrs = gps_attributes
};


static int __devinit gps_probe(struct platform_device *pdev)
{
	int ret;
	struct gps_data *data;

	data = kzalloc(sizeof(struct gps_data), GFP_KERNEL);
	if(!data) {
		pr_err("%s():kzalloc fail !!\n", __func__);
		return -ENOMEM;
	}
	/* config gpio pins as uart port */
	s3c_gpio_setpull(M040_GPS_RTS, GPIO_PULL_NONE);
	s3c_gpio_setpull(M040_GPS_CTS, GPIO_PULL_NONE);
	s3c_gpio_setpull(M040_GPS_TXD, GPIO_PULL_NONE);
	s3c_gpio_setpull(M040_GPS_RXD, GPIO_PULL_UP);
	s3c_gpio_cfgpin(M040_GPS_RTS, S3C_GPIO_SFN(2));
	s3c_gpio_cfgpin(M040_GPS_CTS, S3C_GPIO_SFN(2));
	s3c_gpio_cfgpin(M040_GPS_RXD, S3C_GPIO_SFN(2));
	s3c_gpio_cfgpin(M040_GPS_TXD, S3C_GPIO_SFN(2));

	data->gps_power = M040_GPS_PWRON;
	data->gps_reset = M040_GPS_RST;
	dev_set_drvdata(&pdev->dev, data);

	ret = sysfs_create_group(&pdev->dev.kobj, &gps_attribute_group);
	if (ret < 0) {
		pr_err("%s():sys create group fail !!\n", __func__);
		goto driver_free;
	}

	ret = gpio_request(data->gps_reset, "GPS_nRST");
	if (ret) {
		pr_err("%s():fail to request gpio (GPS_nRST)\n", __func__);
		goto sysfs_exit;
	}

	s3c_gpio_setpull(data->gps_reset, GPIO_PULL_NONE);
	gpio_direction_output(data->gps_reset, 1);

	ret = gpio_request(data->gps_power, "GPS_PWR");
	if (ret) {
		pr_err("%s():fail to request gpio (GPS_nRST)\n", __func__);
		goto gpio_req;
	}

	s3c_gpio_setpull(data->gps_power, GPIO_PULL_NONE);
	gpio_direction_output(data->gps_power, 0);

	if (mx_is_factory_test_mode(MX_FACTORY_TEST_BT)) {
		s3c_gpio_cfgpin(M040_GPS_RTS, S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(M040_GPS_CTS, S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(M040_GPS_RXD, S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(M040_GPS_TXD, S3C_GPIO_INPUT);
		s3c_gpio_setpull(M040_GPS_RTS, GPIO_PULL_DOWN);
		s3c_gpio_setpull(M040_GPS_CTS, GPIO_PULL_DOWN);
		s3c_gpio_setpull(M040_GPS_RXD, GPIO_PULL_DOWN);
		gpio_direction_output(data->gps_power, 1);
		printk("GPS in test mode!\n");
	}

	pr_info("gps successfully probed!\n");
	
	return 0;

gpio_req:
	gpio_free(data->gps_reset);
sysfs_exit:
	sysfs_remove_group(&pdev->dev.kobj, &gps_attribute_group);
driver_free:
	kfree(data);
	return ret;
}


static int __devexit gps_remove(struct platform_device *pdev)
{
	struct gps_data *data = dev_get_drvdata(&pdev->dev);

	sysfs_remove_group(&pdev->dev.kobj, &gps_attribute_group);
	gpio_free(data->gps_power);
	gpio_free(data->gps_reset);
	kfree(data);
	
	return 0;
}


static void gps_shutdown(struct platform_device *pdev)
{
	struct gps_data *data = dev_get_drvdata(&pdev->dev);

	if (gpio_get_value(data->gps_power))
		gpio_set_value(data->gps_power, 0);
}
/*platform driver data*/
static struct platform_driver gps_driver = {
	.driver = {
		.name = "brcm-gps",
		.owner = THIS_MODULE,
	},
	.probe =   gps_probe,
	.remove = __devexit_p(gps_remove),
	.shutdown = gps_shutdown,
};

	
static int __init gps_init(void)
{
	return platform_driver_register(&gps_driver);
}


static void __exit gps_exit(void)
{
	platform_driver_unregister(&gps_driver);
}

module_init(gps_init);
module_exit(gps_exit);

MODULE_AUTHOR("heljoy liu");
MODULE_DESCRIPTION("broadcom gps driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
