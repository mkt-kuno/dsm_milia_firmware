#ifndef LOADCELL_INCLUDE_H
#define LOADCELL_INCLUDE_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#define LOADCELL_ENABLE_INTERRUPT (0)
#define LOADCELL_ENABLE_FILTER (1)

#define CONFIG_LOADCELL_CS1237  (0)
#define CONFIG_LOADCELL_HX711   (1)

#define CONFIG_LOADCELL_FREQ 	(80)
#ifndef CONFIG_LOADCELL_FREQ
#define CONFIG_LOADCELL_FREQ 	(10)
#endif

#if LOADCELL_ENABLE_FILTER
#define SMA (CONFIG_LOADCELL_FREQ/10)
#endif

struct LoadCell {
    struct gpio_dt_spec dout;
    struct gpio_dt_spec sck;
    int32_t previous_value;
    bool is_init;
#if LOADCELL_ENABLE_INTERRUPT
    struct gpio_callback gpio_cb;
    struct k_event event;
#endif
#if LOADCELL_ENABLE_FILTER
    int16_t filtered_value;
    int32_t filter_buf[SMA];
    int p_filter_buf;
#endif
};

void loadcell_setup(struct LoadCell *lc);
void loadcell_loop(struct LoadCell *lc);
int32_t loadcell_get_raw_value_i32(struct LoadCell *lc);
int16_t loadcell_get_raw_value(struct LoadCell *lc);
int16_t loadcell_get_filtered_value(struct LoadCell *lc);

#endif // LOADCELL_INCLUDE_H