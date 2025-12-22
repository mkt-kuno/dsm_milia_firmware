#ifndef LOADCELL_INCLUDE_H
#define LOADCELL_INCLUDE_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#ifndef CONFIG_LOADCELL_FREQ
#define CONFIG_LOADCELL_FREQ 	(10)
#endif
#ifndef CONFIG_LOADCELL_ENABLE_FILTER
#define CONFIG_LOADCELL_ENABLE_FILTER 0
#endif
#define LOADCELL_ENABLE_FILTER CONFIG_LOADCELL_ENABLE_FILTER
#if LOADCELL_ENABLE_FILTER
#define SMA (CONFIG_LOADCELL_FREQ/10)
#endif

enum loadcell_chip_type {
    LOADCELL_CHIP_HX711 = 0,
    LOADCELL_CHIP_CS1237 = 1,
};

struct LoadCell {
    struct gpio_dt_spec dout;
    struct gpio_dt_spec sck;
    enum loadcell_chip_type chip_type;
    int32_t previous_value;
    bool is_init;
    bool is_interrupt_enable;
    struct gpio_callback gpio_cb;
    struct k_event event;
#if LOADCELL_ENABLE_FILTER
    int32_t filtered_value;
    int32_t filter_buf[SMA];
    int p_filter_buf;
#endif
};

void loadcell_setup(struct LoadCell *lc);
void loadcell_loop(struct LoadCell *lc);
int32_t loadcell_get_raw_value_i32(struct LoadCell *lc);
int16_t loadcell_get_raw_value(struct LoadCell *lc);

#endif // LOADCELL_INCLUDE_H
