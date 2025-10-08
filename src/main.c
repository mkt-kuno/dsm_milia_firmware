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

#define USB_WORKQUEUE_PRIORITY 	(-2)
#define HX711_STACK_SIZE 		(384)
#define HX711_PRIORITY 			(USB_WORKQUEUE_PRIORITY+2)
#define ADS1115_STACK_SIZE 		(512)
#define ADS1115_PRIORITY 		(USB_WORKQUEUE_PRIORITY+2)
#define GP8403_STACK_SIZE 		(512)
#define GP8403_PRIORITY 		(USB_WORKQUEUE_PRIORITY+3)

#define ADS1115_RESOLUTION 	15 // ADS1115は16ビットの解像度だが差動じゃないので15ビット指定が必要
#define ADS1115_GAIN 		ADC_GAIN_1_3
#define ADS1115_REFERENCE 	ADC_REF_INTERNAL
#define ADS1115_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 4)

#define GP8403_MAX_CH	8
static int16_t ads1115_result[8];
static uint16_t gp8403_values[8] = {0};

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
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *const modbus_dev = DEVICE_DT_GET(DT_PARENT(MODBUS_NODE));
static const struct gpio_dt_spec mculed_gpio_dt_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);

static const struct device *const ads1115_dev[] = {
	DEVICE_DT_GET(DT_NODELABEL(ads1115_1)),
	DEVICE_DT_GET(DT_NODELABEL(ads1115_2)),
};

typedef struct gp8403_message {
	uint16_t channel; // 0~7
	uint16_t value; // 0~10000
} gp8403_msg_t;
#define GP8403_MSG_BUF_SIZE (GP8403_MAX_CH*2)
gp8403_msg_t gp8403_msg_buffer[GP8403_MSG_BUF_SIZE];
struct k_msgq gp8403_msgq;

void hx711_main(void *param1, void *param2, void *param3);
void gp8403_main(void *param1, void *param2, void *param3);
void ads1115_main(void *param1, void *param2, void *param3);

K_THREAD_DEFINE(tid_hx711_0, HX711_STACK_SIZE, hx711_main, &hx711_list[0], true,  NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_hx711_1, HX711_STACK_SIZE, hx711_main, &hx711_list[1], true,  NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_hx711_2, HX711_STACK_SIZE, hx711_main, &hx711_list[2], true,  NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_hx711_3, HX711_STACK_SIZE, hx711_main, &hx711_list[3], true,  NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_hx711_4, HX711_STACK_SIZE, hx711_main, &hx711_list[4], true,  NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_hx711_5, HX711_STACK_SIZE, hx711_main, &hx711_list[5], true,  NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_hx711_6, HX711_STACK_SIZE, hx711_main, &hx711_list[6], true,  NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_hx711_7, HX711_STACK_SIZE, hx711_main, &hx711_list[7], true,  NULL, HX711_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_ads1115_0, ADS1115_STACK_SIZE, ads1115_main, 0, NULL, NULL, ADS1115_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_ads1115_1, ADS1115_STACK_SIZE, ads1115_main, 1, NULL, NULL, ADS1115_PRIORITY, 0, 0);
K_THREAD_DEFINE(tid_gp8403, GP8403_STACK_SIZE, gp8403_main, NULL, NULL, NULL, GP8403_PRIORITY, 0, 0);

static int modbus_slave_coil_rd(uint16_t addr, bool *state)
{
	if (addr > 0) return -ENOTSUP;
	// addr 0: MCU Board-LED
	if (addr == 0) {
		*state = gpio_pin_get_dt(&mculed_gpio_dt_spec);
	}
	return 0;
}

static int modbus_slave_coil_wr(uint16_t addr, bool state)
{
	if (addr > 0) return -ENOTSUP;
	// addr 0: MCU Board-LED
	if (addr == 0) {
		gpio_pin_set_dt(&mculed_gpio_dt_spec, state);
	}
	return 0;
}

static int modbus_slave_holding_reg_rd(uint16_t addr, uint16_t *reg)
{
	if (addr >= GP8403_MAX_CH) return -ENOTSUP;
	uint32_t key = 0;
	key = irq_lock();
	{
		*reg = gp8403_values[addr];
	}
	irq_unlock(key);
	return 0;
}

static int modbus_slave_holding_reg_wr(uint16_t addr, uint16_t reg)
{
	if (addr >= GP8403_MAX_CH) return -ENOTSUP;
	if (reg > 10000) reg = 10000;

	gp8403_msg_t send_data = {0};
	send_data.channel = addr;
	send_data.value = reg;

	int ret = k_msgq_put(&gp8403_msgq, &send_data, K_NO_WAIT);
	if (ret) {
		LOG_ERR("GP8403 msgq full");
	}
	return 0;
}

static int modbus_slave_input_reg_rd(uint16_t addr, uint16_t *reg)
{
	if (addr < 8){
		// HX711 values
		*reg = (uint16_t)(loadcell_get_raw_value(&hx711_list[addr]) & 0xFFFF);
	} else  if (addr < 16) {
		// ADS1115 values
		uint32_t key = 0;
		key = irq_lock();
		{
			*reg = (uint16_t)(ads1115_result[addr - 8] & 0xFFFF);
		}
		irq_unlock(key);
	} else {
		return -ENOTSUP;
	}
	return 0;
}

static int modbus_slave_input_reg_rd_fp(uint16_t addr, float *reg)
{
	if (addr < 8){
		// HX711 values
		*reg = loadcell_get_raw_value_i32(&hx711_list[addr]);
	} else  if (addr < 16) {
		// ADS1115 values
		uint32_t key = 0;
		key = irq_lock();
		{
			*reg = ads1115_result[addr - 8];
		}
		irq_unlock(key);
	} else {
		return -ENOTSUP;
	}
	return 0;
}

static struct modbus_user_callbacks modbus_slave_user_callbacks = {
	.coil_rd = modbus_slave_coil_rd,
	.coil_wr = modbus_slave_coil_wr,
	.holding_reg_rd = modbus_slave_holding_reg_rd,
	.holding_reg_wr = modbus_slave_holding_reg_wr,
	.input_reg_rd = modbus_slave_input_reg_rd,
	.input_reg_rd_fp = modbus_slave_input_reg_rd_fp,
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
	gpio_pin_configure_dt(&mculed_gpio_dt_spec, GPIO_OUTPUT_INACTIVE);

	k_msgq_init(&gp8403_msgq, (char *)gp8403_msg_buffer, sizeof(gp8403_msg_t), GP8403_MSG_BUF_SIZE);

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

int16_t ads1115_read_adc_channel(uint8_t unit_id, uint8_t channel_id)
{
	ads1115_channel_cfg[unit_id].input_positive = channel_id;
	const struct device *const dev = ads1115_dev[unit_id];
    if (adc_channel_setup(dev, &ads1115_channel_cfg[unit_id]) != 0) {
        LOG_ERR("Failed to setup ADC channel %d", channel_id);
        return -1;
    }

    if (adc_read(dev, &ads1115_sequence[unit_id]) != 0) {
        LOG_ERR("Failed to read ADC channel %d", channel_id);
        return -1;
    }

    return ads1115_sample_buffer[unit_id];
}

void ads1115_main(void *param1, void *param2, void *param3)
{
	uint32_t key = 0;
	uint8_t unit_id = (uint8_t)((uint32_t)(param1) & 0xFF);
	if (unit_id > 1) {
		LOG_ERR("Invalid unit_id %d", unit_id);
		return;
	}

	while (1) {
		for (int channel = 0; channel < 4; channel++) {
			int16_t value = ads1115_read_adc_channel(unit_id, channel);
			key = irq_lock();
			{
				ads1115_result[unit_id * 4 + channel] = value;
			}
			irq_unlock(key);
		}
	}
}

void hx711_main(void *param1, void *param2, void *param3)
{
	struct LoadCell *lc = (struct LoadCell *)(param1);
	bool interrupt_enable = (bool)(param2);
	loadcell_setup(lc, interrupt_enable);
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

int gp8403_set_channel(uint8_t address, uint8_t channel_id, uint16_t data) {
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

int gp8403_set_channels(uint8_t address, uint16_t data_0, uint16_t data_1) {
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

void gp8403_main(void *param1, void *param2, void *param3) {
	const uint8_t gp8403_adr[] = {0x58, 0x59, 0x5A, 0x5B};
	int ret = 0;

	// set GP8403 output range to 0-10V
	for (int ch = 0; ch < 4; ch++) {
		gp8403_values[2*ch + 0] = 0;
		gp8403_values[2*ch + 1] = 0;
		ret = gp8403_init(gp8403_adr[ch]);
		if (ret) {
			LOG_ERR("Failed to init GP8403 adrs:0x%2d", gp8403_adr[ch]);
		}
		ret = gp8403_set_channels(gp8403_adr[ch], 0, 0);
		if (ret) {
			LOG_ERR("Failed to set GP8403 adrs:0x%2d", gp8403_adr[ch]);
		}
	}

	// Change Value
	uint16_t gp8403_request[8] = {0};
	for (;;) {
		gp8403_msg_t recv_data;
		// Wait for request
		k_msgq_get(&gp8403_msgq, &recv_data, K_FOREVER);
		gp8403_request[recv_data.channel] = recv_data.value;
		// Get all requests in the queue
		while (k_msgq_peek(&gp8403_msgq, &recv_data) == 0)
		{
			int ret = k_msgq_get(&gp8403_msgq, &recv_data, K_NO_WAIT);
			if (ret) {
				LOG_ERR("Failed to get GP8403 msgq");
				break;
			}
			gp8403_request[recv_data.channel] = recv_data.value;
		}

		for (int ch = 0; ch < 4; ch++) {
			// Change DAC outputs, both channels
			uint16_t req0 = gp8403_request[2*ch + 0];
			uint16_t req1 = gp8403_request[2*ch + 1];
			if (gp8403_values[2*ch + 0] != req0
			 && gp8403_values[2*ch + 1] != req1) {
				ret = gp8403_set_channels(gp8403_adr[ch], req0, req1);
				gp8403_values[2*ch + 0] = req0;
				gp8403_values[2*ch + 1] = req1;
				if (ret) {
					LOG_ERR("Failed to set GP8403 adrs:0x%2d", gp8403_adr[ch]);
				}
			}
			// Change DAC output only for channel 0
			else if (gp8403_values[2*ch + 0] != req0) {
				ret = gp8403_set_channel(gp8403_adr[ch], 0, req0);
				gp8403_values[2*ch + 0] = req0;
				if (ret) {
					LOG_ERR("Failed to set GP8403 adrs:0x%2d", gp8403_adr[ch]);
				}
			}
			// Change DAC output only for channel 1
			else if (gp8403_values[2*ch + 1] != req1) {
				ret = gp8403_set_channel(gp8403_adr[ch], 1, req1);
				gp8403_values[2*ch + 1] = req1;
				if (ret) {
					LOG_ERR("Failed to set GP8403 adrs:0x%2d", gp8403_adr[ch]);
				}
			}
		}
	}
}