// SPDX-License-Identifier: GPL-2.0
//
// Touchkey driver for Freescale MPR121 Controllor
//
// Copyright (C) 2011 Freescale Semiconductor, Inc.
// Author: Zhang Jiejing <jiejing.zhang@freescale.com>
//
// Based on mcs_touchkey.c
//
// Copyright (C) 2019 Y Soft Corporation, a.s.
// Author: Pavel Staněk <pavel.stanek@ysoft.com>
// Author: Michal Vokáč <michal.vokac@ysoft.com>
//
// Reworked into polling driver based on mpr121_touchkey.c


#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/input/matrix_keypad.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

/* Register definitions */
#define ELE_TOUCH_STATUS_0_ADDR	0x0
#define ELE_TOUCH_STATUS_1_ADDR	0X1
#define MHD_RISING_ADDR		0x2b
#define NHD_RISING_ADDR		0x2c
#define NCL_RISING_ADDR		0x2d
#define FDL_RISING_ADDR		0x2e
#define MHD_FALLING_ADDR	0x2f
#define NHD_FALLING_ADDR	0x30
#define NCL_FALLING_ADDR	0x31
#define FDL_FALLING_ADDR	0x32
#define ELE0_TOUCH_THRESHOLD_ADDR	0x41
#define ELE0_RELEASE_THRESHOLD_ADDR	0x42
#define DEBOUNCE_TR_ADDR		0x5b
#define AFE_CONF_ADDR			0x5c
#define FILTER_CONF_ADDR		0x5d

/*
 * ELECTRODE_CONF_ADDR: This register configures the number of
 * enabled capacitance sensing inputs and its run/suspend mode.
 */
#define ELECTRODE_CONF_ADDR		0x5e
#define ELECTRODE_CONF_QUICK_CHARGE	0x80
#define AUTO_CONFIG_CTRL_ADDR		0x7b
#define AUTO_CONFIG_CTRL1_ADDR		0x7c
#define AUTO_CONFIG_USL_ADDR		0x7d
#define AUTO_CONFIG_LSL_ADDR		0x7e
#define AUTO_CONFIG_TL_ADDR		0x7f

/* Threshold of touch/release trigger */
#define TOUCH_THRESHOLD			0x08
#define RELEASE_THRESHOLD		0x05
/* Masks for touch and release triggers */
#define TOUCH_STATUS_MASK		0xfff
/* MPR121 has 12 keys */
#define MPR121_MAX_KEY_COUNT		12

#define MPR121_POLL_INTERVAL_MAX	1000
#define MPR121_POLL_INTERVAL_MIN	0
#define MPR121_POLL_INTERVAL_DEF	50
#define MPR121_POLL_INTERVAL_REINIT	500

#define MPR121_READ_RETRY_COUNT		4
#define MPR121_CACHE_MAX_ADDR		0x7F
#define MPR121_REG_CACHE_CHECK_POLL_COUNT	8

#define MPR121_CL		0
/* Filter configuration */
#define MPR121_FFI		3
#define MPR121_CDC		32
#define MPR121_CDT		1
#define MPR121_SFI		0
#define MPR121_ESI		4
/* Auto configuration */
#define MPR121_RETRY		0
#define MPR121_BVA		MPR121_CL
#define MPR121_ARE		1
#define MPR121_ACE		1
#define MPR121_SCTS		1
#define MPR121_OORIE		0
#define MPR121_ARFIE		0
#define MPR121_ACFIE		0

struct mpr121_reg_cache {
	u8 valid;
	u8 value;
};

struct mpr121_touchkey {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	struct input_polled_dev	*poll_dev;
	unsigned int		statusbits;
	unsigned int		keycount;
	u32			keycodes[MPR121_MAX_KEY_COUNT];
	u8			reinit_needed;
	u8			read_errors;
	struct mpr121_reg_cache	reg_cache[MPR121_CACHE_MAX_ADDR + 1];
	u8			next_check_count;
	u8			next_check_addr;
	char			phys[32];
	unsigned int		rows;
	unsigned int		cols;
	unsigned int		row_shift;
	int			vdd_uv;
};

struct mpr121_init_register {
	int addr;
	u8 val;
};

static const struct mpr121_init_register init_reg_table[] = {
	{ MHD_RISING_ADDR,	0x1 },
	{ NHD_RISING_ADDR,	0x1 },
	{ MHD_FALLING_ADDR,	0x1 },
	{ NHD_FALLING_ADDR,	0x1 },
	{ NCL_FALLING_ADDR,	0xff },
	{ FDL_FALLING_ADDR,	0x02 },
	{ FILTER_CONF_ADDR,	0x04 },
	{ AFE_CONF_ADDR,	0x0b },
	{ AUTO_CONFIG_CTRL_ADDR, 0x0b },
};

static int mpr121_write_reg(struct mpr121_touchkey *mpr121, u8 addr, u8 value)
{
	struct i2c_client *client = mpr121->client;
	int ret;

	if (addr <= MPR121_CACHE_MAX_ADDR) {
		mpr121->reg_cache[addr].valid = 1;
		mpr121->reg_cache[addr].value = value;
	}

	ret = i2c_smbus_write_byte_data(client, addr, value);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write error: %d\n", ret);
		return ret;
	}

	return 0;
}

static void mpr121_vdd_supply_disable(void *data)
{
	struct regulator *vdd_supply = data;

	regulator_disable(vdd_supply);
}

static struct regulator *mpr121_vdd_supply_init(struct device *dev)
{
	struct regulator *vdd_supply;
	int err;

	vdd_supply = devm_regulator_get(dev, "vdd");
	if (IS_ERR(vdd_supply)) {
		dev_err(dev, "failed to get vdd regulator: %ld\n",
			PTR_ERR(vdd_supply));
		return vdd_supply;
	}

	err = regulator_enable(vdd_supply);
	if (err) {
		dev_err(dev, "failed to enable vdd regulator: %d\n", err);
		return ERR_PTR(err);
	}

	err = devm_add_action(dev, mpr121_vdd_supply_disable, vdd_supply);
	if (err) {
		regulator_disable(vdd_supply);
		dev_err(dev, "failed to add disable regulator action: %d\n",
			err);
		return ERR_PTR(err);
	}

	return vdd_supply;
}

static int mpr121_phys_init(struct mpr121_touchkey *mpr121,
			    struct i2c_client *client)
{
	const struct mpr121_init_register *reg;
	unsigned char usl, lsl, tl, eleconf;
	int i, t, vdd, ret;

	dev_dbg(&client->dev, "phys init\n");

	/* Set stop mode first */
	ret = mpr121_write_reg(mpr121, ELECTRODE_CONF_ADDR, 0x00);
	if (ret < 0)
		goto err_i2c_write;

	/* Set up touch/release threshold for ele0-ele11 */
	for (i = 0; i <= MPR121_MAX_KEY_COUNT; i++) {
		t = ELE0_TOUCH_THRESHOLD_ADDR + (i * 2);
		ret = mpr121_write_reg(mpr121, t, TOUCH_THRESHOLD);
		if (ret < 0)
			goto err_i2c_write;
		ret = mpr121_write_reg(mpr121, t + 1, RELEASE_THRESHOLD);
		if (ret < 0)
			goto err_i2c_write;
	}

	/* Set up init register */
	for (i = 0; i < ARRAY_SIZE(init_reg_table); i++) {
		reg = &init_reg_table[i];
		ret = mpr121_write_reg(mpr121, reg->addr, reg->val);
		if (ret < 0)
			goto err_i2c_write;
	}

	/*
	 * Capacitance on sensing input varies and needs to be compensated.
	 * The internal MPR121-auto-configuration can do this if it's
	 * registers are set properly (based on vdd_uv).
	 */
	vdd = mpr121->vdd_uv / 1000;
	usl = ((vdd - 700) * 256) / vdd;
	lsl = (usl * 65) / 100;
	tl = (usl * 90) / 100;
	ret = mpr121_write_reg(mpr121, AUTO_CONFIG_USL_ADDR, usl);
	ret |= mpr121_write_reg(mpr121, AUTO_CONFIG_LSL_ADDR, lsl);
	ret |= mpr121_write_reg(mpr121, AUTO_CONFIG_TL_ADDR, tl);

	/*
	 * Quick charge bit will let the capacitive charge to ready
	 * state quickly, or the buttons may not function after system
	 * boot.
	 */
	eleconf = mpr121->keycount | ELECTRODE_CONF_QUICK_CHARGE;
	ret |= mpr121_write_reg(mpr121, ELECTRODE_CONF_ADDR,
					 eleconf);
	if (ret != 0)
		goto err_i2c_write;

	return 0;

err_i2c_write:
	dev_err(&client->dev, "i2c write error: %d\n", ret);
	return ret;
}

static void mpr121_touchkey_release(struct mpr121_touchkey *mpr121)
{
	struct input_dev *input = mpr121->input_dev;
	struct i2c_client *client = mpr121->client;
	unsigned long statusbits;
	unsigned int key_num;

	if (!mpr121->statusbits)
		return;

	statusbits = mpr121->statusbits;
	mpr121->statusbits = 0;
	for_each_set_bit(key_num, &statusbits, mpr121->keycount) {
		unsigned int key_val;

		key_val = mpr121->keycodes[key_num];

		input_event(input, EV_MSC, MSC_SCAN, key_num);
		input_report_key(input, key_val, 0);
	}
	input_sync(input);
}

static int mpr121_touchkey_process(struct mpr121_touchkey *mpr121)
{
	struct input_dev *input = mpr121->input_dev;
	struct i2c_client *client = mpr121->client;
	unsigned int key_num, key_val, pressed;
	unsigned int code;
	int reg;

	reg = i2c_smbus_read_word_data(client, ELE_TOUCH_STATUS_0_ADDR);
	if (reg < 0) {
		dev_err(&client->dev, "i2c read error [%d]\n", reg);
		return reg;
	}

	reg &= TOUCH_STATUS_MASK;
	if (reg == mpr121->statusbits)
		return 0;

	while (reg != mpr121->statusbits) {
		/* use old press bit to figure out which bit changed */
		key_num = ffs(reg ^ mpr121->statusbits) - 1;
		pressed = reg & (1 << key_num);
		mpr121->statusbits ^= (1 << key_num);

		code = MATRIX_SCAN_CODE(0, key_num, mpr121->row_shift);
		key_val = mpr121->keycodes[code];

		input_event(input, EV_MSC, MSC_SCAN, code);
		input_report_key(input, key_val, pressed);
	}
	input_sync(input);

	return 0;
}

static int mpr121_check_regs(struct mpr121_touchkey *mpr121)
{
	struct i2c_client *client = mpr121->client;
	int i, reg;

	for (i = mpr121->next_check_addr; i <= MPR121_CACHE_MAX_ADDR; i++) {
		if (mpr121->reg_cache[i].valid)
			break;
	}

	if (i > MPR121_CACHE_MAX_ADDR) {
		mpr121->next_check_addr = 0;
		return 0;
	}

	mpr121->next_check_addr = i + 1;
	reg = i2c_smbus_read_byte_data(client, i);
	if (reg < 0) {
		dev_err(&client->dev, "i2c read error [%d]\n", reg);
		return -1;
	}

	if (reg != mpr121->reg_cache[i].value)
		return -1;

	return 0;
}

static void mpr121_poll(struct input_polled_dev *dev)
{
	struct mpr121_touchkey *mpr121 = dev->private;
	struct i2c_client *client = mpr121->client;
	int ret;

	if (mpr121->read_errors > MPR121_READ_RETRY_COUNT) {
		mpr121->reinit_needed = 1;
		mpr121->read_errors = 0;
		mpr121_touchkey_release(mpr121);
	}

	if (mpr121->reinit_needed) {
		dev_warn(&client->dev, "reinit needed\n");

		ret = mpr121_phys_init(mpr121, client);
		if (ret >= 0) {
			mpr121->reinit_needed = 0;
			dev->poll_interval = MPR121_POLL_INTERVAL_DEF;
		} else {
			dev->poll_interval = MPR121_POLL_INTERVAL_REINIT;
		}

		return;
	}

	ret = mpr121_touchkey_process(mpr121);
	if (ret < 0) {
		mpr121->read_errors++;
		return;
	}

	mpr121->read_errors = 0;
	mpr121->next_check_count++;
	if (mpr121->next_check_count > MPR121_REG_CACHE_CHECK_POLL_COUNT) {
		mpr121->next_check_count = 0;
		ret = mpr121_check_regs(mpr121);
		if (ret < 0)
			mpr121->read_errors++;
	}
}

static int mpr_touchkey_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct regulator *vdd_supply;
	struct mpr121_touchkey *mpr121;
	struct input_dev *input_dev;
	struct input_polled_dev *poll_dev;
	int error;
	int i;

	vdd_supply = mpr121_vdd_supply_init(dev);
	if (IS_ERR(vdd_supply))
		return PTR_ERR(vdd_supply);

	mpr121 = devm_kzalloc(dev, sizeof(*mpr121), GFP_KERNEL);
	if (!mpr121)
		return -ENOMEM;

	poll_dev = devm_input_allocate_polled_device(dev);
	if (!poll_dev)
		return -ENOMEM;

	mpr121->vdd_uv = regulator_get_voltage(vdd_supply);
	mpr121->client = client;
	mpr121->input_dev = poll_dev->input;
	mpr121->poll_dev = poll_dev;
	mpr121->keycount = device_property_read_u32_array(dev, "linux,keycodes",
							  NULL, 0);
	if (mpr121->keycount > MPR121_MAX_KEY_COUNT) {
		dev_err(dev, "too many keys defined (%d)\n", mpr121->keycount);
		return -EINVAL;
	}

	error = device_property_read_u32_array(dev, "linux,keycodes",
					       mpr121->keycodes,
					       mpr121->keycount);
	if (error) {
		dev_err(dev,
			"failed to read linux,keycode property: %d\n", error);
		return error;
	}

	snprintf(mpr121->phys, sizeof(mpr121->phys), "%s/input0",
		 dev_name(&client->dev));

	poll_dev->private = mpr121;
	poll_dev->poll = mpr121_poll;
	poll_dev->poll_interval = MPR121_POLL_INTERVAL_DEF;
	poll_dev->poll_interval_max = MPR121_POLL_INTERVAL_MAX;
	poll_dev->poll_interval_min = MPR121_POLL_INTERVAL_MIN;

	input_dev = poll_dev->input;
	input_dev->name = "Freescale MPR121 Touchkey";
	input_dev->phys = mpr121->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	if (device_property_read_bool(dev, "autorepeat"))
		__set_bit(EV_REP, input_dev->evbit);
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);

	input_dev->keycode = mpr121->keycodes;
	input_dev->keycodesize = sizeof(mpr121->keycodes[0]);
	input_dev->keycodemax = mpr121->keycount;

	for (i = 0; i < mpr121->keycount; i++)
		input_set_capability(input_dev, EV_KEY, mpr121->keycodes[i]);

	error = mpr121_phys_init(mpr121, client);
	if (error) {
		dev_err(dev, "Failed to init register\n");
		return error;
	}

	error = input_register_polled_device(poll_dev);
	if (error)
		return error;

	i2c_set_clientdata(client, mpr121);

	return 0;
}

static int __maybe_unused mpr_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	i2c_smbus_write_byte_data(client, ELECTRODE_CONF_ADDR, 0x00);

	return 0;
}

static int __maybe_unused mpr_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpr121_touchkey *mpr121 = i2c_get_clientdata(client);

	i2c_smbus_write_byte_data(client, ELECTRODE_CONF_ADDR,
				  mpr121->keycount);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mpr121_touchkey_pm_ops, mpr_suspend, mpr_resume);

static const struct i2c_device_id mpr121_id[] = {
	{ "mpr121p_touchkey", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpr121_id);

#ifdef CONFIG_OF
static const struct of_device_id mpr121_touchkey_dt_match_table[] = {
	{ .compatible = "fsl,mpr121p-touchkey" },
	{ },
};
MODULE_DEVICE_TABLE(of, mpr121_touchkey_dt_match_table);
#endif

static struct i2c_driver mpr_touchkey_driver = {
	.driver = {
		.name	= "mpr121p",
		.pm	= &mpr121_touchkey_pm_ops,
		.of_match_table = of_match_ptr(mpr121_touchkey_dt_match_table),
	},
	.id_table	= mpr121_id,
	.probe		= mpr_touchkey_probe,
};

module_i2c_driver(mpr_touchkey_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michal Vokáč <michal.vokac@ysoft.com>");
MODULE_DESCRIPTION("Touch Key polling driver for Freescale MPR121 Chip");
