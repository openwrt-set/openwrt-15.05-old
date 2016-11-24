#ifndef _GPIO_TR_H
#define _GPIO_TR_H

#include <linux/gpio.h>
#include <linux/of_gpio.h>

struct gpio_tr_pin {
	/* Configuration parameters */
	int high_pin;		/* High level gpio pin number */
	int low_pin;		/* Low level gpio pin number */
	int input_pin;		/* Input gpio pin number */
	int direction;
	enum of_gpio_flags input_flags;
	enum of_gpio_flags high_flags;
	enum of_gpio_flags low_flags;
	int claimed;		/* Is pin requested */

};

struct gpio_tr_platform_data {
	struct gpio_tr_pin *pins;
	int npins;
	struct gpio_chip* chip;
};

struct gpio_tr {
	struct gpio_chip chip;
	struct device *dev;
};

#endif
