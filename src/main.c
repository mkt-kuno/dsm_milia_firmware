/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/logging/log.h>

#include "loadcell.h"

LOG_MODULE_REGISTER(main);

#define HX711_STACK_SIZE 256
#define HX711_PRIORITY 0
#define ADS1115_STACK_SIZE 512
#define ADS1115_PRIORITY 0
#define GP8403_STACK_SIZE 512
#define GP8403_PRIORITY 0

#define ADS1115_RESOLUTION 15 // ADS1115は16ビットの解像度だが差動じゃないので15ビット指定が必要
#define ADS1115_GAIN ADC_GAIN_1_3
#define ADS1115_REFERENCE ADC_REF_INTERNAL
#define ADS1115_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 4)

static uint16_t holding_reg[8];
static int16_t input_reg_i16[16];

static struct LoadCell hx711_list[] = {
	{
		.dout = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_dout), gpios, 0),
		.sck = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_sck), gpios, 0),
	},
	{
		.dout = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_dout), gpios, 1),
		.sck = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_sck), gpios, 1),
	},
	{
		.dout = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_dout), gpios, 2),
		.sck = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_sck), gpios, 2),
	},
	{
		.dout = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_dout), gpios, 3),
		.sck = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_sck), gpios, 3),
	},
	{
		.dout = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_dout), gpios, 4),
		.sck = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_sck), gpios, 4),
	},
	{
		.dout = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_dout), gpios, 5),
		.sck = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_sck), gpios, 5),
	},
	{
		.dout = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_dout), gpios, 6),
		.sck = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_sck), gpios, 6),
	},
	{
		.dout = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_dout), gpios, 7),
		.sck = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(hx711_sck), gpios, 7),
	}
};

#define MODBUS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
static const struct device *const modbus_dev = DEVICE_DT_GET(DT_PARENT(MODBUS_NODE));
static const struct gpio_dt_spec led_gpio_dt_spec = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const ads1115_1_dev = DEVICE_DT_GET(DT_NODELABEL(ads1115_1));
static const struct device *const ads1115_2_dev = DEVICE_DT_GET(DT_NODELABEL(ads1115_2));

static int modbus_slave_coil_rd(uint16_t addr, bool *state)
{
	if (addr > 1) return -ENOTSUP;
	*state = gpio_pin_get_dt(&led_gpio_dt_spec);
	return 0;
}

static int modbus_slave_coil_wr(uint16_t addr, bool state)
{
	if (addr > 1) return -ENOTSUP;
	gpio_pin_set_dt(&led_gpio_dt_spec, (int)state);
	return 0;
}

static int modbus_slave_holding_reg_rd(uint16_t addr, uint16_t *reg)
{
	if (addr >= ARRAY_SIZE(holding_reg)) return -ENOTSUP;
	*reg = holding_reg[addr];
	return 0;
}

static int modbus_slave_holding_reg_wr(uint16_t addr, uint16_t reg)
{
	if (addr >= ARRAY_SIZE(holding_reg)) return -ENOTSUP;
	holding_reg[addr] = reg;
	return 0;
}

static int modbus_slave_input_reg_rd(uint16_t addr, uint16_t *reg)
{
	if (addr >= ARRAY_SIZE(input_reg_i16)) return -ENOTSUP;
	if (addr < 8){
		// HX711 values
		*reg = (uint16_t)(((int16_t)(loadcell_get_filtered_value(&hx711_list[addr])/256.0f)) & 0xFFFF);
	} else {
		// ADS1115 values
		*reg = 0x0000;
	}
	return 0;
}

static struct modbus_user_callbacks modbus_slave_user_callbacks = {
	.coil_rd = modbus_slave_coil_rd,
	.coil_wr = modbus_slave_coil_wr,
	.holding_reg_rd = modbus_slave_holding_reg_rd,
	.holding_reg_wr = modbus_slave_holding_reg_wr,
	.input_reg_rd = modbus_slave_input_reg_rd,
};

const static struct modbus_iface_param modbus_slave_if_param = {
	.mode = MODBUS_MODE_RTU,
	.server = {
		.user_cb = &modbus_slave_user_callbacks,
		.unit_id = 1,
	},
	.serial = {
		.baud = 38400,
		.parity = UART_CFG_PARITY_NONE,
	},
};

static int modbus_slave_init(void)
{
	const char iface_name[] = {DEVICE_DT_NAME(MODBUS_NODE)};
	int iface;

	iface = modbus_iface_get_by_name(iface_name);

	if (iface < 0) {
		LOG_ERR("Failed to get iface index for %s", iface_name);
		return iface;
	}

	return modbus_init_server(iface, modbus_slave_if_param);
}

int main(void)
{
	uint32_t dtr = 0;

	gpio_pin_configure_dt(&led_gpio_dt_spec, GPIO_OUTPUT_INACTIVE);

	if (!device_is_ready(modbus_dev) || usb_enable(NULL)) {
		return 0;
	}

	if (modbus_slave_init()) {
		LOG_ERR("Modbus RTU server initialization failed");
	}
}

static int16_t ads1115_sample_buffer[2]; // サンプルバッファ
static struct adc_channel_cfg ads1115_channel_cfg[2] = {
	{
		.channel_id = 0,
		.differential = 0,
		.gain = ADS1115_GAIN,
		.reference = ADS1115_REFERENCE,
		.acquisition_time = ADS1115_ACQUISITION_TIME,
	},
	{
		.channel_id = 0,
		.differential = 0,
		.gain = ADS1115_GAIN,
		.reference = ADS1115_REFERENCE,
		.acquisition_time = ADS1115_ACQUISITION_TIME,
	}
};

static struct adc_sequence ads1115_sequence[2] = {
	{
		.buffer = &ads1115_sample_buffer[0],
		.buffer_size = 2,
		.resolution = ADS1115_RESOLUTION,
		.channels = BIT(0),
		.calibrate = 0,
		.options = NULL
	},
	{
		.buffer = &ads1115_sample_buffer[1],
		.buffer_size = 2,
		.resolution = ADS1115_RESOLUTION,
		.channels = BIT(0),
		.calibrate = 0,
		.options = NULL
	}
};

int ads1115_read_adc_channel(uint8_t unit_id, uint8_t channel_id)
{
	ads1115_channel_cfg[unit_id].input_positive = channel_id;

    if (adc_channel_setup(ads1115_1_dev, &ads1115_channel_cfg[unit_id]) != 0) {
        LOG_ERR("Failed to setup ADC channel %d", channel_id);
        return -1;
    }

    if (adc_read(ads1115_1_dev, &ads1115_sequence[unit_id]) != 0) {
        LOG_ERR("Failed to read ADC channel %d", channel_id);
        return -1;
    }

    return ads1115_sample_buffer[unit_id];
}

void ads1115_main(void)
{
	while (1) {
		uint8_t unit_id = 0;
		for (int channel = 0; channel < 4; channel++) {
			int value = ads1115_read_adc_channel(unit_id, channel);
			if (value < 0) {
				LOG_ERR("Error reading channel %d", channel);
			} else {
				LOG_INF("ADS1115 U%d CH%d: %d", unit_id, channel, value);
			}
		}
		k_sleep(K_SECONDS(1));
	}
}

void hx711_main(void *param1, void *param2, void *param3)
{
	struct LoadCell *lc = (struct LoadCell *)(param1);
	loadcell_setup(lc);
	loadcell_loop(lc);
}

int gp8403_init(uint8_t adr) {
	struct i2c_msg msgs[1];
	unsigned char buf[2];
	int ret = 0;

	buf[0] = 0x01; //OUTPUT_RANGE
	
	msgs[0].buf = &buf[0];
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	ret = i2c_transfer(i2c_dev, &msgs[0], 1, adr);
	if (ret) {
		LOG_ERR("Failed to set GP8403 init: %d", ret);
	}
	LOG_INF("GP8403 init done");
	
	msgs[0].buf[0] = 0x01; //OUTPUT_RANGE
	msgs[0].buf[1] = 0x11; //0-10V
	msgs[0].len = 2;
	ret = i2c_transfer(i2c_dev, &msgs[0], 1, adr);
	if (ret) {
		LOG_ERR("Failed to set GP8403 output range ret: %d", ret);
	}
	LOG_INF("GP8403 output range 10V set done");

	return ret;
}

int gp8404_set_channel(uint8_t address, uint8_t channel_id, uint16_t data) {
	struct i2c_msg msgs[1];
	uint8_t buf[3];
	int ret = 0;
	uint16_t send_data = data <= 10000 ? data : 10000;
	send_data = (uint16_t)(((float)send_data / 10000) * 0x0FFF) << 4;
	
	buf[0] = channel_id==0? 0x02: 0x04;  // GP8302_CONFIG_CURRENT_REG
	buf[1] = send_data & 0xFF;
	buf[2] = (send_data >> 8) & 0xFF;

	msgs[0].buf = &buf[0];
	msgs[0].len = 3;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer(i2c_dev, &msgs[0], 1, address);
	if (ret) LOG_ERR("Failed to set GP8404 channel %d: %d", channel_id, ret);
	return ret;
}

int gp8404_set_channels(uint8_t address, uint16_t data_0, uint16_t data_1) {
	struct i2c_msg msgs[1];
	uint8_t buf[5];
	uint16_t send_data[2] = {0, 0};
	send_data[0] = data_0 <= 10000 ? data_0 : 10000;
	send_data[0] = (uint16_t)(((float)send_data[0] / 10000) * 0x0FFF) << 4;
	send_data[1] = data_1 <= 10000 ? data_1 : 10000;
	send_data[1] = (uint16_t)(((float)send_data[1] / 10000) * 0x0FFF) << 4;
	
	buf[0] = 0x02; // GP8302_CONFIG_CURRENT_REG
	buf[1] = send_data[0] & 0xFF;
	buf[2] = (send_data[0] >> 8) & 0xFF;
	buf[3] = send_data[1] & 0xFF;
	buf[4] = (send_data[1] >> 8) & 0xFF;

	msgs[0].buf = &buf[0];
	msgs[0].len = 5;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 1, address);
}

int gp8403_test(void) {
	const uint8_t gp8403_adr = 0x58;

	int ret = 0;

	// set GP8403 output range to 0-10V
	ret = gp8403_init(gp8403_adr);
	if (ret) {
		LOG_ERR("Failed to init GP8403: %d", ret);
		return ret;
	}

	// Change Value step by step 0 to 10V in 1V steps
	while (1) {
		// CH 0
		for (uint16_t data = 0; data <= 10000; data += 1000) {
			ret = gp8404_set_channel(gp8403_adr, 0, data);
			LOG_INF("GP8403 CH0: %.3lf[V]", (double)data/1000.0);
			k_msleep(2000);
		}

		// CH 1
		for (uint16_t data = 0; data <= 10000; data += 1000) {
			ret = gp8404_set_channel(gp8403_adr, 1, data);
			LOG_INF("GP8403 CH1: %.3lf[V]", (double)data/1000.0);
			k_msleep(2000);
		}

		// Both set
		for (uint16_t data = 0; data <= 10000; data += 1000) {
			ret = gp8404_set_channels(gp8403_adr, data, data);
			LOG_INF("GP8403 CH0&1: %.3lf[V]", (double)data/1000.0);
			k_msleep(2000);
		}

		// Reset to 0V
		ret = gp8404_set_channels(gp8403_adr, 0, 0);
		LOG_INF("GP8403 CH0&1: 0.000[V]");
		k_msleep(2000);
	}
}

K_THREAD_DEFINE(hx711_0, HX711_STACK_SIZE, hx711_main, &hx711_list[0], NULL, NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(hx711_1, HX711_STACK_SIZE, hx711_main, &hx711_list[1], NULL, NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(hx711_2, HX711_STACK_SIZE, hx711_main, &hx711_list[2], NULL, NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(hx711_3, HX711_STACK_SIZE, hx711_main, &hx711_list[3], NULL, NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(hx711_4, HX711_STACK_SIZE, hx711_main, &hx711_list[4], NULL, NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(hx711_5, HX711_STACK_SIZE, hx711_main, &hx711_list[5], NULL, NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(hx711_6, HX711_STACK_SIZE, hx711_main, &hx711_list[6], NULL, NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(hx711_7, HX711_STACK_SIZE, hx711_main, &hx711_list[7], NULL, NULL, HX711_PRIORITY, 0, 0);
//K_THREAD_DEFINE(ads1115_1, ADS1115_STACK_SIZE, ads1115_main, NULL, NULL, NULL, ADS1115_PRIORITY, 0, 0);
//K_THREAD_DEFINE(gp8403, GP8403_STACK_SIZE, gp8403_test, NULL, NULL, NULL, GP8403_PRIORITY, 0, 0);