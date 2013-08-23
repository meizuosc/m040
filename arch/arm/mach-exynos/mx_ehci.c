#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <plat/ehci.h>
#include <plat/cpu.h>
#include <plat/usb-phy.h>
#include <asm/mach-types.h>

/* USB EHCI */
#ifdef CONFIG_USB_EHCI_S5P
static int m030_usb_dev_phy_power(struct platform_device *pdev,int on)
{
	int ret;
	struct regulator_bulk_data supplies[2];
	int num_consumers = ARRAY_SIZE(supplies);

	dev_dbg(&pdev->dev, "%s() : [%s]\n", __FUNCTION__, on?"ON":"OFF");

	supplies[0].supply = "pd_io";//
	supplies[1].supply = "pd_core";//


	ret = regulator_bulk_get(&pdev->dev, num_consumers, supplies);
	if (ret) {
		dev_err(&pdev->dev, "%s():regulator_bulk_get failed\n", __func__);
		return ret;
	}

	if (on) {
		ret = regulator_bulk_enable(num_consumers, supplies);
	}
	else {
		ret = regulator_bulk_disable(num_consumers, supplies);
	}
	regulator_bulk_free(num_consumers, supplies);
	
	return 0;
}

static int m032_usb_dev_phy_power(struct platform_device *pdev,int on)
{
	int ret;
	struct regulator_bulk_data supplies[3];
	int num_consumers = ARRAY_SIZE(supplies);

	dev_dbg(&pdev->dev, "%s() : [%s]\n", __FUNCTION__, on?"ON":"OFF");

	supplies[0].supply = "pd_hsic";//
	supplies[1].supply = "vdd_ldo16";//
	supplies[2].supply = "vdd_ldo15";//

	ret = regulator_bulk_get(&pdev->dev, num_consumers, supplies);
	if (ret) {
		dev_err(&pdev->dev, "%s():regulator_bulk_get failed\n", __func__);
		return ret;
	}

	if (on) {
		ret = regulator_bulk_enable(num_consumers, supplies);
	}
	else {
		ret = regulator_bulk_disable(num_consumers, supplies);
	}
	regulator_bulk_free(num_consumers, supplies);
	
	return 0;	
}

static int m030_usb_host_phy_power(struct platform_device *pdev, int on)
{
	int ret;
	struct regulator_bulk_data supplies[3];
	int num_consumers = ARRAY_SIZE(supplies);

	dev_dbg(&pdev->dev, "%s() : [%s]\n", __FUNCTION__, on?"ON":"OFF");

	supplies[0].supply = "pd_io";//
	supplies[1].supply = "pd_core";//
	supplies[2].supply = "pd_hsic";//


	ret = regulator_bulk_get(&pdev->dev, num_consumers, supplies);
	if (ret) {
		dev_err(&pdev->dev, "%s():regulator_bulk_get failed\n", __func__);
		return ret;
	}

	if (on) {
		ret = regulator_bulk_enable(num_consumers, supplies);
	}
	else {
		ret = regulator_bulk_disable(num_consumers, supplies);
	}
	regulator_bulk_free(num_consumers, supplies);
	
	return 0;	
}

static int m032_usb_host_phy_power(struct platform_device *pdev, int on)
{
	int ret;
	struct regulator_bulk_data supplies[3];
	int num_consumers = ARRAY_SIZE(supplies);

	dev_dbg(&pdev->dev, "%s() : [%s]\n", __FUNCTION__, on?"ON":"OFF");

	supplies[0].supply = "pd_hsic";//
	supplies[1].supply = "vdd_ldo16";//
	supplies[2].supply = "vdd_ldo15";//

	ret = regulator_bulk_get(&pdev->dev, num_consumers, supplies);
	if (ret) {
		dev_err(&pdev->dev, "%s():regulator_bulk_get failed\n", __func__);
		return ret;
	}

	if (on) {
		ret = regulator_bulk_enable(num_consumers, supplies);
	}
	else {
		ret = regulator_bulk_disable(num_consumers, supplies);
	}
	regulator_bulk_free(num_consumers, supplies);
	
	return 0;	
}

int mx_usb_phy_power(struct platform_device *pdev,int type, int on)
{
	if(machine_is_m030()) {
		if (type == S5P_USB_PHY_HOST)
			return m030_usb_host_phy_power(pdev, on);
		else
			return m030_usb_dev_phy_power(pdev, on);
	} else {
		if (type == S5P_USB_PHY_HOST)
			return m032_usb_host_phy_power(pdev, on);
		else
			return m032_usb_dev_phy_power(pdev, on);
	}
	return 0;
}

static struct s5p_ehci_platdata mx_ehci_pdata={
	.phy_power = mx_usb_phy_power,
};

static int __init mx_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &mx_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
	return 0;
}

arch_initcall(mx_ehci_init);

MODULE_DESCRIPTION("mx ehci driver helper");
MODULE_AUTHOR("wbwu <wenbinwu@meizu.com>");
MODULE_LICENSE("GPLV2");
#endif
