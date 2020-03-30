// SPDX-License-Identifier: GPL-2.0-or-later


#ifndef CONFIG_ARM64
#include <mach/am_regs.h>
#else
#include <linux/reset.h>
#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/platform_device.h>

#include "aml_fe_avl68x2_r848.h"

#include "avl68x2.h"
#include "r848.h"

#include "aml_dvb.h"
#undef pr_err

#define pr_dbg(fmt, args...)          \
  do                                  \
  {                                   \
    if (debug_fe)                     \
      printk("DVB FE: " fmt, ##args); \
  } while (0)
#define pr_err(fmt, args...) printk("DVB FE: " fmt, ##args)
#define pr_inf(fmt, args...) printk("DVB FE: " fmt, ##args)

MODULE_PARM_DESC(debug_fe, "\n\t\t Enable frontend debug information");
static int debug_fe = 1;
module_param(debug_fe, int, 0644);

MODULE_PARM_DESC(frontend_power, "\n\t\t Power GPIO of frontend");
static int frontend_power = -1;
module_param(frontend_power, int, 0644);

MODULE_PARM_DESC(frontend_reset, "\n\t\t Reset GPIO of frontend");
static int frontend_reset = -1;
module_param(frontend_reset, int, 0644);

static struct aml_fe avl68x2_fe[FE_DEV_COUNT];

static char *device_name = "avl68x2";


static int avl68x2_fe_init(struct aml_dvb *advb,
			   struct platform_device *pdev,
			   struct aml_fe *fe,
			   int id)
{
	struct dvb_frontend_ops *ops;
	int ret, i2c_adap_id = 1;
	int demod_i2c_addr = 0x14;
	int tuner_i2c_addr = 0x62;

	struct i2c_adapter *i2c_handle;
#ifdef CONFIG_ARM64
	struct gpio_desc *desc;
	int gpio_reset, gpio_power;
#endif

	struct avl68x2_priv		e2_priv;
	static struct avl68x2_config	e2_config;
	struct avl68x2_chip_pub		e2_pub;

    static struct r848_config	r848_config;

	pr_inf("Init AVL68x2 frontend %d\n", id);

#ifdef CONFIG_OF
	pr_inf("CONFIG_OF defined\n");
	if (of_property_read_u32(pdev->dev.of_node,
				 "dtv_demod0_i2c_adap_id",
				 &i2c_adap_id))
	{
		pr_dbg("error getting i2c_adap_id, of_node=%s\n",
		       pdev->dev.of_node->name);
		ret = -ENOMEM;
		goto err_resource;
	}
	pr_dbg("i2c_adap_id=%d\n", i2c_adap_id);
	desc = of_get_named_gpiod_flags(pdev->dev.of_node,
					"dtv_demod0_reset_gpio-gpios",
					0,
					NULL);
	gpio_reset = desc_to_gpio(desc);
	pr_dbg("gpio_reset=%d\n", gpio_reset);

	desc = of_get_named_gpiod_flags(pdev->dev.of_node,
					"dtv_demod0_power_gpio-gpios",
					0,
					NULL);
	gpio_power = desc_to_gpio(desc);
	pr_dbg("gpio_power=%d\n", gpio_power);

	if (of_property_read_u32(pdev->dev.of_node,
				 "dtv_demod0_i2c_addr",
				 &demod_i2c_addr))
	{
		pr_dbg("error getting dtv_demod0_i2c_addr, of_node=%s\n, using default",
		       pdev->dev.of_node->name);
	}
	if (of_property_read_u32(pdev->dev.of_node,
				 "dtv_demod0_tuner_i2c_addr",
				 &tuner_i2c_addr))
	{
		pr_dbg("error getting dtv_demod0_tuner_i2c_addr, of_node=%s\n, using default",
		       pdev->dev.of_node->name);
	}
#endif /*CONFIG_OF*/

	e2_config.chip_pub = &e2_pub;
	e2_pub.i2c_addr = ((/*demod ID*/ (id & AVL_DEMOD_ID_MASK)) << 8) |
			  ((uint8_t)demod_i2c_addr);
	e2_pub.xtal = Xtal_27M;
	//FIXME e2_pub.tuner_pol = 
	e2_pub.ts_config.eMode = AVL_TS_PARALLEL;
	e2_pub.ts_config.eClockEdge = AVL_MPCM_RISING;
	e2_pub.ts_config.eParallelPhase = AVL_TS_PARALLEL_PHASE_0;
	e2_pub.ts_config.eClockMode = AVL_TS_CONTINUOUS_DISABLE;
	e2_pub.ts_config.ePacketLen = AVL_TS_188;
	e2_pub.ts_config.eSerialPin = AVL_MPSP_DATA0;

	frontend_reset = gpio_reset;
	frontend_power = gpio_power;
	i2c_handle = i2c_get_adapter(i2c_adap_id);

	if (!i2c_handle)
	{
		pr_err("Cannot get i2c adapter for id:%d! \n", i2c_adap_id);
		ret = -ENOMEM;
		goto err_resource;
	}

	fe->fe = dvb_attach(avl68x2_attach, &e2_config, i2c_handle);

	if (!fe->fe)
	{
		pr_err("avl68x2_attach attach failed!!!\n");
		ret = -ENOMEM;
		goto err_resource;
	}
	ex_priv = (struct avl68x2_priv *)fe->fe->demodulator_priv;


	if(dvb_attach(r848_attach, fe->fe, &r848_config, i2c_handle) == NULL)
	{
		dvb_frontend_detach(fe->fe);
		fe->fe = NULL;
		pr_err("r848 attach attach failed!!!\n");
		ret = -ENOMEM;
		goto err_resource;
	}

	pr_inf("AVL68x2 and r848 attached!\n");

	if ((ret = dvb_register_frontend(&advb->dvb_adapter, fe->fe)))
	{
		pr_err("Frontend AVL68x2 registration failed!!!\n");
		ops = &fe->fe->ops;
		if (ops->release != NULL)
			ops->release(fe->fe);
		fe->fe = NULL;
		ret = -ENOMEM;
		goto err_resource;
	}

	pr_inf("Frontend AVL68x2 registered!\n");

	return 0;

err_resource:
	return ret;
}

static int avl68x2_fe_probe(struct platform_device *pdev)
{
  int ret = 0;

  struct aml_dvb *dvb = aml_get_dvb_device();

  if (avl68x2_fe_init(dvb, pdev, &avl68x2_fe[0], 0) < 0)
    return -ENXIO;

  platform_set_drvdata(pdev, &avl68x2_fe[0]);

  return ret;
}

static void avl68x2_fe_release(struct aml_dvb *advb, struct aml_fe *fe)
{
  if (fe && fe->fe)
  {
    dvb_unregister_frontend(fe->fe);
    dvb_frontend_detach(fe->fe);
  }
}

static int avl68x2_fe_remove(struct platform_device *pdev)
{
  struct aml_fe *drv_data = platform_get_drvdata(pdev);
  struct aml_dvb *dvb = aml_get_dvb_device();

  platform_set_drvdata(pdev, NULL);
  avl68x2_fe_release(dvb, drv_data);

  return 0;
}

static int avl68x2_fe_resume(struct platform_device *pdev)
{
  pr_dbg("avl68x2_fe_resume \n");
  return 0;
}

static int avl68x2_fe_suspend(struct platform_device *pdev, pm_message_t state)
{
  pr_dbg("avl68x2_fe_suspend \n");
  return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aml_fe_dt_match[] = {
    {
        .compatible = "amlogic,dvbfe",
    },
    {},
};
//MODULE_DEVICE_TABLE(of, aml_fe_dt_match);
#endif /*CONFIG_OF*/

static struct platform_driver aml_fe_driver = {
    .probe = avl68x2_fe_probe,
    .remove = avl68x2_fe_remove,
    .resume = avl68x2_fe_resume,
    .suspend = avl68x2_fe_suspend,
    .driver = {
        .name = "aml_fe",
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = aml_fe_dt_match,
#endif
    }};

static int __init avlfrontend_init(void)
{
  return platform_driver_register(&aml_fe_driver);
}

static void __exit avlfrontend_exit(void)
{
  platform_driver_unregister(&aml_fe_driver);
}

module_init(avlfrontend_init);
module_exit(avlfrontend_exit);
MODULE_AUTHOR("Availink");
MODULE_DESCRIPTION("AVL68X2 + R848 frontend driver");
MODULE_LICENSE("GPL");
