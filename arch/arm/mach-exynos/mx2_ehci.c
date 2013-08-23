#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <plat/ehci.h>
#include <plat/cpu.h>
#include <plat/usb-phy.h>
#include <plat/devs.h>
#include <asm/mach-types.h>
#ifdef CONFIG_UMTS_MODEM_XMM6260
#include <mach/modem.h>
#endif

/* USB EHCI */
#ifdef CONFIG_USB_EHCI_S5P
static int m040_usb_dev_phy_power(struct platform_device *pdev,int on)
{
	int ret;
	struct regulator_bulk_data supplies[2];
	int num_consumers = ARRAY_SIZE(supplies);

	dev_dbg(&pdev->dev, "%s() : [%s]\n", __FUNCTION__, on?"ON":"OFF");

	supplies[0].supply = "vdd_ldo16";//
	supplies[1].supply = "vdd_ldo15";//

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

static int m040_usb_host_phy_power(struct platform_device *pdev, int on)
{
	int ret;
	struct regulator_bulk_data supplies[2];
	int num_consumers = ARRAY_SIZE(supplies);

	dev_dbg(&pdev->dev, "%s() : [%s]\n", __FUNCTION__, on?"ON":"OFF");

	supplies[0].supply = "vdd_ldo16";//
	supplies[1].supply = "vdd_ldo15";//

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
int mx2_usb_phy_power(struct platform_device *pdev,int type, int on)
{
	if (type == S5P_USB_PHY_HOST){
		return m040_usb_host_phy_power(pdev, on);
	}else{
		return m040_usb_dev_phy_power(pdev, on);
	}

	return 0;
}
static void mx2_modem_set_active_state(int state)
{
#ifdef CONFIG_UMTS_MODEM_XMM6260
	modem_set_active_state(state);
#endif
	return;
}
static struct s5p_ehci_platdata mx2_ehci_pdata={
	.phy_power = mx2_usb_phy_power,
	.set_cp_active = mx2_modem_set_active_state,
};

static int __init mx2_ehci_init(void)
{
	int ret = 0;
	struct s5p_ehci_platdata *pdata = &mx2_ehci_pdata;
	
	if(machine_is_m040()){
		pdata->late_resume = 1;
		pdata->wait_device = 1;
	}else{
		pdata->late_resume = 0;
		pdata->wait_device  = 0;
		pdata->set_cp_active = NULL;
	}
	ret = platform_device_register(&s5p_device_ehci);
	if(ret)
		return ret;
	s5p_ehci_set_platdata(pdata);

	return ret;
}

arch_initcall(mx2_ehci_init);

MODULE_DESCRIPTION("mx2 ehci driver helper");
MODULE_AUTHOR("wbwu <wenbinwu@meizu.com>");
MODULE_LICENSE("GPLV2");
#endif
