#ifndef _GPIO_LV_H
#define _GPIO_LV_H

#include <linux/gpio.h>
#include <linux/of_gpio.h>

struct gpio_lv_pin {
	/* Configuration parameters */
	int input_pin; 		/* Input gpio pin number */
	int direction_pin; 	/* Direction gpio pin number */
	enum of_gpio_flags input_flags;    /* Pin flags*/
	enum of_gpio_flags direction_flags;/* Pin flags*/	
	int claimed;		/* Is pin activated*/

};

struct gpio_lv_platform_data {
	struct gpio_lv_pin *pins;
	int npins;
	struct gpio_chip* chip;
};

struct gpio_lv {
	struct gpio_chip chip;
	struct device *dev;
};

#endif
