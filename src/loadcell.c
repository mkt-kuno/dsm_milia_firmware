#include "loadcell.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lc);

#if LOADCELL_ENABLE_INTERRUPT
#define LOADCELL_EVENT (0x0001)
static void loadcell_dout_ready_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	struct LoadCell *lc = CONTAINER_OF(cb, struct LoadCell, gpio_cb);
	gpio_pin_interrupt_configure_dt(&lc->dout, GPIO_INT_DISABLE);
	k_event_set(&lc->event, LOADCELL_EVENT);
}
#endif

static inline int loadcell_onebit_in(struct LoadCell *lc) {
    int val = 0;
    gpio_pin_set_dt(&lc->sck, 1);
    k_busy_wait(1);
    gpio_pin_set_dt(&lc->sck, 0);
    val = gpio_pin_get_dt(&lc->dout);
    k_busy_wait(1);
    return val;
}

static inline void loadcell_onebit_out(struct LoadCell *lc, int output) {
    gpio_pin_set_dt(&lc->sck, 1);
    k_busy_wait(1);
	gpio_pin_set_dt(&lc->dout, output);
    gpio_pin_set_dt(&lc->sck, 0);
    k_busy_wait(1);
}

static inline int32_t loadcell_get_24bit_data(struct LoadCell *lc) {
    int32_t val = 0x00000000;
    for (int bit = 0; bit < 24; bit++) val = ((0x7FFFFFFF & val) << 1) | (loadcell_onebit_in(lc) & 0x01);
	if (val & 0x00800000) val |= 0xFF000000;
    return val;
}

int32_t loadcell_get_raw_value_i32(struct LoadCell *lc) {
	int32_t ret = 0;
	uint32_t key = irq_lock();
	{
		ret = lc->previous_value;
	}
	irq_unlock(key);
    return ret;
}

int16_t loadcell_get_raw_value(struct LoadCell *lc) {
	int32_t ret = 0;
	uint32_t key = irq_lock();
	{
		ret = lc->previous_value;
	}
	irq_unlock(key);
    return (int16_t)((ret >> 8) & 0x0000FFFF);
}

int16_t loadcell_get_filtered_value(struct LoadCell *lc) {
	float ret = 0.0f;
	uint32_t key = irq_lock();
	{
#if LOADCELL_ENABLE_FILTER
		ret = lc->filtered_value;
#else
		ret = lc->previous_value;
#endif
	}
	irq_unlock(key);
	return ret;
}

void loadcell_setup(struct LoadCell *lc) {
	int timeout = 1000;
	// init value
	lc->is_init = false;
	lc->previous_value = 0;
#if LOADCELL_ENABLE_FILTER
	// init fir filter
	for (int i = 0; i < SMA; i++) lc->filter_buf[i] = 0;
	lc->p_filter_buf = 0;
	lc->filtered_value = 0.0f;
#endif
	// init gpio
    gpio_pin_configure_dt(&lc->dout, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&lc->sck, GPIO_OUTPUT | GPIO_PULL_UP);

#if LOADCELL_ENABLE_INTERRUPT
	if (lc->is_interrupt_enable) {
		k_event_init(&lc->event);
		gpio_init_callback(&lc->gpio_cb, loadcell_dout_ready_callback, BIT(lc->dout.pin));
		gpio_add_callback_dt(&lc->dout, &lc->gpio_cb);
	}
#endif
    // Reset and power down
    gpio_pin_set_dt(&lc->sck, 0);
    gpio_pin_set_dt(&lc->sck, 1);
#if CONFIG_LOADCELL_CS1237
    k_msleep(100);
#endif
#if CONFIG_LOADCELL_HX711
	k_usleep(100);
#endif
    gpio_pin_set_dt(&lc->sck, 0);

    // Wait for waking up
    while (gpio_pin_get_dt(&lc->dout) == 0) {
        k_msleep(1);
		if (--timeout == 0) k_sleep(K_FOREVER);
    }

    // Wait for power up
    while (gpio_pin_get_dt(&lc->dout) == 1) {
        k_msleep(1);
		if (--timeout == 0) k_sleep(K_FOREVER);
    }

#if LOADCELL_ENABLE_INTERRUPT
	if (lc->is_interrupt_enable) {
		gpio_pin_interrupt_configure_dt(&lc->dout, GPIO_INT_EDGE_TO_ACTIVE);
	}
#endif
}

void loadcell_loop(struct LoadCell *lc) {
	int32_t val = 0;
	uint32_t key = 0;

	const int32_t wait_bit_us = 10 * 1000 / CONFIG_LOADCELL_FREQ;
	const int32_t wait_next_us = 800 * 1000 / CONFIG_LOADCELL_FREQ;

	while(true) {
#if LOADCELL_ENABLE_INTERRUPT
		if (lc->is_interrupt_enable){
			k_event_wait(&lc->event, LOADCELL_EVENT, true, K_FOREVER);
		}
		else {
			while (gpio_pin_get_dt(&lc->dout) == 1) k_usleep(wait_bit_us);
		}
#else
		while (gpio_pin_get_dt(&lc->dout) == 1) k_usleep(wait_bit_us);
#endif
		key = irq_lock();
		{
			val = loadcell_get_24bit_data(lc);
#if CONFIG_LOADCELL_HX711
			// 25bit (ch A gain 128)
			// 26bit (ch B gain 32)
			// 27bit (ch A gain 64)

			// set ch A gain 128
			loadcell_onebit_in(lc);
#endif
#if CONFIG_LOADCELL_CS1237
			// Set Mode "same as HX711 protocol"
			// keep HIGH, if data is not READY
			// bit: 25-27
			loadcell_onebit_in(lc);
			loadcell_onebit_in(lc);
			loadcell_onebit_in(lc);

			if (lc->is_init == false) {
				// set DOUT output
				gpio_pin_configure_dt(&lc->dout, GPIO_OUTPUT | GPIO_PULL_UP);
				// 28-29bit (force High)
				loadcell_onebit_out(lc, 1);
				loadcell_onebit_out(lc, 1);
				// set mode Write "Function config" (0x65)
				// 30-36bit
				loadcell_onebit_out(lc, 1);
				loadcell_onebit_out(lc, 1);
				loadcell_onebit_out(lc, 0);
				loadcell_onebit_out(lc, 0);
				loadcell_onebit_out(lc, 1);
				loadcell_onebit_out(lc, 0);
				loadcell_onebit_out(lc, 1);
				// wait 1bit for change dir
				// 37bit
				loadcell_onebit_out(lc, 1);
				// set "Function config"
				// 38-45bit
				loadcell_onebit_out(lc, 0);
				loadcell_onebit_out(lc, 0);
				// freq bits 00: 10Hz 01: 40Hz
				// freq bits 10:640Hz 11:1280Hz
#if (CONFIG_LOADCELL_FREQ == 640 || CONFIG_LOADCELL_FREQ == 1280)
				loadcell_onebit_out(lc, 1);
#else
				loadcell_onebit_out(lc, 0);
#endif
#if (CONFIG_LOADCELL_FREQ == 40 || CONFIG_LOADCELL_FREQ == 1280)
				loadcell_onebit_out(lc, 1);
#else
				loadcell_onebit_out(lc, 0);
#endif
				loadcell_onebit_out(lc, 1);// PGA bits  00: x1   01: x2
				loadcell_onebit_out(lc, 1);// PGA bits  10: x64  11: x128
				loadcell_onebit_out(lc, 0);
				loadcell_onebit_out(lc, 0);
				// wait 1bit for change dir
				loadcell_onebit_out(lc, 1);
				// reset DOUT input
				gpio_pin_configure_dt(&lc->dout, GPIO_INPUT | GPIO_PULL_UP);
				
				lc->is_init = true;
			}
#endif
			lc->previous_value = val;
		}
		irq_unlock(key);
#if LOADCELL_ENABLE_INTERRUPT
		if (lc->is_interrupt_enable){
			gpio_pin_interrupt_configure_dt(&lc->dout, GPIO_INT_EDGE_TO_INACTIVE);
		} else {
			k_usleep(wait_next_us);
		}
#endif
#if !LOADCELL_ENABLE_INTERRUPT
		k_usleep(wait_next_us);
#endif
#if LOADCELL_ENABLE_FILTER
		// filter
		lc->filter_buf[lc->p_filter_buf] = val;
		lc->p_filter_buf = (lc->p_filter_buf + 1) % SMA;
		int32_t _sum = 0;
		for (int i = 0; i < SMA; i++) _sum += lc->filter_buf[i];
		key = irq_lock();
		{
			lc->filtered_value = (((_sum/SMA) >> 8) & 0x0000FFFF);;
		}
		irq_unlock(key);
#endif
	}
}
