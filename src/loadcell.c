#include "loadcell.h"

#include <limits.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lc);

#if LOADCELL_ENABLE_FILTER
struct lc_sos {
	int32_t b0;
	int32_t b1;
	int32_t b2;
	int32_t a1;
	int32_t a2;
};

static inline int32_t sat32(int64_t x)
{
	if (x > INT32_MAX) {
		return INT32_MAX;
	}
	if (x < INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)x;
}

static inline int32_t q31_mul(int32_t a, int32_t b)
{
	int64_t prod = (int64_t)a * (int64_t)b;

	prod += (prod >= 0) ? (1LL << 30) : -(1LL << 30);
	return (int32_t)(prod >> 31);
}

static inline int32_t sos_df2t(const struct lc_sos *c, struct lc_sos_state *s, int32_t x)
{
	int64_t acc = (int64_t)q31_mul(c->b0, x) + s->z1;
	int32_t y = sat32(acc);
	int64_t z1 = (int64_t)q31_mul(c->b1, x) + s->z2 - (int64_t)q31_mul(c->a1, y);
	int64_t z2 = (int64_t)q31_mul(c->b2, x) - (int64_t)q31_mul(c->a2, y);

	s->z1 = sat32(z1);
	s->z2 = sat32(z2);
	return y;
}

/*
 * IIR pipeline (Fs = 80 Hz, Q31, DF2T):
 * - Notches at 20 Hz and 30 Hz to suppress 60/50 Hz aliases after sampling.
 * - Low-pass is a conservative 3x one-pole cascade at 3.2 Hz to keep |a1| < 1 in Q31.
 *   Replace the last 3 sections with an offline-designed SOS (ellip/cheby2/butter) if
 *   sharper stopband rejection is required.
 *
 * Example (Python/scipy) for elliptic SOS generation and Q31 conversion:
 *   sos = signal.ellip(6, 1, 70, 3.2, 'low', fs=80, output='sos')
 *   q31 = np.round(sos[:, :3] * 2**31).astype(np.int64)
 *   # a0 is 1; use a1/a2 from sos[:, 4:6] and convert to Q31 the same way.
 */
static const struct lc_sos lc_sos_coeffs[LOADCELL_SOS_COUNT] = {
	/* Notch @20 Hz, Q=0.8 (wide to tolerate drift, fits Q31 range). */
	{ 1321528399, 0, 1321528399, 0, 495573150 },
	/* Notch @30 Hz, Q=0.8 (wide to tolerate drift, fits Q31 range). */
	{ 1489299873, 2106188079, 1489299873, 2106188079, 831116099 },
	/* Low-pass 1-pole @3.2 Hz (section 1/3). */
	{ 477240275, 0, 0, -1670243373, 0 },
	/* Low-pass 1-pole @3.2 Hz (section 2/3). */
	{ 477240275, 0, 0, -1670243373, 0 },
	/* Low-pass 1-pole @3.2 Hz (section 3/3). */
	{ 477240275, 0, 0, -1670243373, 0 },
};

static void loadcell_filter_reset(struct LoadCell *lc)
{
	uint32_t key = irq_lock();
	struct lc_filter_state *st = &lc->lc_filter_states;

	if (st != NULL) {
		memset(st, 0, sizeof(*st));
	}
	lc->filtered_value = 0;
	irq_unlock(key);
}

static bool loadcell_filter_process(struct LoadCell *lc, int32_t x, int32_t *out)
{
	struct lc_filter_state *st = &lc->lc_filter_states;
	int32_t y = x;

	if (st == NULL) {
		return false;
	}

	for (int i = 0; i < LOADCELL_SOS_COUNT; i++) {
		y = sos_df2t(&lc_sos_coeffs[i], &st->sos[i], y);
	}

	st->decim_count++;
	if (st->decim_count >= LOADCELL_DECIM_FACTOR) {
		st->decim_count = 0;
		st->last_output = y;
		if (out != NULL) {
			*out = y;
		}
		return true;
	}

	return false;
}
#endif

#define LOADCELL_EVENT (0x0001)
static void loadcell_dout_ready_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	struct LoadCell *lc = CONTAINER_OF(cb, struct LoadCell, gpio_cb);
	gpio_pin_interrupt_configure_dt(&lc->dout, GPIO_INT_DISABLE);
	k_event_set(&lc->event, LOADCELL_EVENT);
}

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
#if LOADCELL_ENABLE_FILTER
		ret = lc->filtered_value;
#else
		ret = lc->previous_value;
#endif
	}
	irq_unlock(key);
    return ret;
}

int16_t loadcell_get_raw_value(struct LoadCell *lc) {
	int32_t ret = loadcell_get_raw_value_i32(lc);
    return (int16_t)((ret >> 8) & 0x0000FFFF);
}

void loadcell_setup(struct LoadCell *lc) {
	int timeout = 1000;
	// init value
	lc->is_init = false;
	lc->previous_value = 0;
#if LOADCELL_ENABLE_FILTER
	lc->p_filter_buf = -1;
	loadcell_filter_reset(lc);
#endif
	// init gpio
    gpio_pin_configure_dt(&lc->dout, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&lc->sck, GPIO_OUTPUT | GPIO_PULL_UP);

	if (lc->is_interrupt_enable) {
		k_event_init(&lc->event);
		gpio_init_callback(&lc->gpio_cb, loadcell_dout_ready_callback, BIT(lc->dout.pin));
		gpio_add_callback_dt(&lc->dout, &lc->gpio_cb);
	}

    // Reset and power down
    gpio_pin_set_dt(&lc->sck, 0);
    gpio_pin_set_dt(&lc->sck, 1);
	if (lc->chip_type == LOADCELL_CHIP_CS1237) {
		k_msleep(100);
	} else {
		k_usleep(100);
	}
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

	if (lc->is_interrupt_enable) {
		gpio_pin_interrupt_configure_dt(&lc->dout, GPIO_INT_EDGE_TO_ACTIVE);
	}
}

void loadcell_loop(struct LoadCell *lc) {
	int32_t val = 0;
	uint32_t key = 0;

	const int32_t wait_bit_us = 10 * 1000 / CONFIG_LOADCELL_FREQ;
	const int32_t wait_next_us = 800 * 1000 / CONFIG_LOADCELL_FREQ;

	while(true) {
		if (lc->is_interrupt_enable){
			k_event_wait(&lc->event, LOADCELL_EVENT, true, K_FOREVER);
		}
		else {
			while (gpio_pin_get_dt(&lc->dout) == 1) k_usleep(wait_bit_us);
		}
		key = irq_lock();
		{
			val = loadcell_get_24bit_data(lc);
			if (lc->chip_type == LOADCELL_CHIP_CS1237) {
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
			} else {
				// 25bit (ch A gain 128)
				// 26bit (ch B gain 32)
				// 27bit (ch A gain 64)

				// set ch A gain 128
				loadcell_onebit_in(lc);
			}
			lc->previous_value = val;
		}
		irq_unlock(key);
		
		if (lc->is_interrupt_enable){
			gpio_pin_interrupt_configure_dt(&lc->dout, GPIO_INT_EDGE_TO_INACTIVE);
		} else {
			k_usleep(wait_next_us);
		}

#if LOADCELL_ENABLE_FILTER
		// IIR + decimate (80 Hz -> 10 Hz). Update output every 8 samples.
		int32_t filt_out = 0;
		if (loadcell_filter_process(lc, val, &filt_out)) {
			key = irq_lock();
			{
				lc->filtered_value = filt_out;
			}
			irq_unlock(key);
		}
#endif
	}
}
