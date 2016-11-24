/*

Driver for GPIO through levelshifters using two pins from SoC for direction and value

*/
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio-tr.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

static int deffered = 0;

void gpio_tr_set_value(struct gpio_chip *chip, unsigned offset, int value );

int gpio_tr_get_direction(struct gpio_chip *chip, unsigned offset ){
    struct gpio_tr_platform_data *pdata = (struct gpio_tr_platform_data *)dev_get_platdata(chip->dev);
   
	return pdata->pins[offset].direction;
}

int gpio_tr_dir_input(struct gpio_chip *chip, unsigned offset ){
    const struct gpio_tr_platform_data *pdata = dev_get_platdata(chip->dev);

	pdata->pins[offset].direction = GPIOF_DIR_IN;
    
    gpio_set_value( pdata->pins[offset].high_pin, 0);
    gpio_set_value( pdata->pins[offset].low_pin, 0);
    
    return 0;
}
int gpio_tr_dir_output(struct gpio_chip *chip, unsigned offset, int value ){
    const struct gpio_tr_platform_data *pdata = dev_get_platdata(chip->dev);
    
	pdata->pins[offset].direction = GPIOF_DIR_OUT;

	gpio_tr_set_value( chip, offset, value );

    return 0;
}
int gpio_tr_get_value(struct gpio_chip *chip, unsigned offset ){
    const struct gpio_tr_platform_data *pdata = dev_get_platdata(chip->dev);
    
    return gpio_get_value( pdata->pins[offset].input_pin);
}

void gpio_tr_set_value(struct gpio_chip *chip, unsigned offset, int value ){
    const struct gpio_tr_platform_data *pdata = dev_get_platdata(chip->dev);

	if( value ) {
	    gpio_set_value_cansleep( pdata->pins[offset].high_pin, 1);
	    gpio_set_value_cansleep( pdata->pins[offset].low_pin, 0);
	} else {
	    gpio_set_value_cansleep( pdata->pins[offset].high_pin, 0);
	    gpio_set_value_cansleep( pdata->pins[offset].low_pin, 1);
	}
    
    return;
}

static int gpio_tr_request(struct gpio_chip *chip, unsigned offset) {
    const struct gpio_tr_platform_data *pdata = dev_get_platdata(chip->dev);

    if( pdata == NULL ){
		dev_err(chip->dev, "Platform data return NULL." );
		return -EFAULT;
    }
	
    struct gpio_tr_pin* pin = &pdata->pins[offset];
	
	dev_dbg(chip->dev, "Request (%d) pin input(%d) high(%d) low(%d)", offset, pin->input_pin, pin->high_pin, pin->low_pin);
	
	if( gpio_request(pin->input_pin, NULL) != 0 ){
	    dev_err(chip->dev, "Input pin(%d) in tr-pin %d already in use",pin->input_pin, offset);
	    return -EBUSY;
	}

	if( gpio_request(pin->high_pin, NULL) != 0 ){
	    gpio_free(pin->input_pin);
	    dev_err(chip->dev, "High level pin(%d) in tr-pin %d already in use",pin->high_pin, offset);
	    return -EBUSY;
	}
	
	if( gpio_request(pin->low_pin, NULL) != 0 ){
	    gpio_free(pin->input_pin);
	    gpio_free(pin->high_pin);
	    dev_err(chip->dev, "Low level pin(%d) in tr-pin %d already in use",pin->low_pin, offset);
	    return -EBUSY;
	}
	
	gpio_direction_input(pin->input_pin);
	gpio_direction_output(pin->high_pin, 0);
	gpio_direction_output(pin->low_pin, 0);
	
    pin->claimed = 1;

	return 0;
}

void gpio_tr_free(struct gpio_chip *chip, unsigned offset){
    const struct gpio_tr_platform_data *pdata = dev_get_platdata(chip->dev);
    struct gpio_tr_pin* pin = &pdata->pins[offset];
    
    if( pin->claimed ){
        gpio_free(pin->input_pin);
        gpio_free(pin->high_pin);
        gpio_free(pin->low_pin);
        pin->claimed = 0;
    }
    
    return;
}

static int gpio_tr_to_irq( struct gpio_chip *chip, unsigned offset ){
    const struct gpio_tr_platform_data *pdata = dev_get_platdata(chip->dev);
    return gpio_to_irq( pdata->pins[offset].input_pin);
}

static int gpio_tr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_tr_platform_data *pdata = NULL;
	struct device_node *root = dev->of_node;
	struct device_node *child;
	struct gpio_tr_pin* pins;
	struct gpio_tr* chip;
	
	int i, ret;

	if ( !root ){
	    dev_err(dev,"No device tree data available\n");
	    return -EINVAL;
	}
	
    chip = devm_kzalloc(dev, sizeof( struct gpio_tr ), GFP_KERNEL);
	
	if( !chip ){
		dev_err(dev,"failed to allocate lv_chip struct\n");
		return -ENOMEM;
	}
	
	if( dev->platform_data == NULL){
		dev->platform_data = devm_kzalloc( dev, sizeof( struct gpio_tr_platform_data ), GFP_KERNEL);
	}
	
	pdata = dev->platform_data;
	
	/* Get count of declared pins*/
	pdata->npins = of_get_child_count(root);

	pins = devm_kzalloc( dev, sizeof( struct gpio_tr_pin ) * pdata->npins, GFP_KERNEL);

	if( !pins ){
	    dev_err(dev,"failed to allocate pins struct\n");
	    return -ENOMEM;
	}
	
	/* set handlers for gpio operations*/
	chip->chip.direction_input	= gpio_tr_dir_input;
	chip->chip.direction_output	= gpio_tr_dir_output;
	chip->chip.get			= gpio_tr_get_value;
	chip->chip.set			= gpio_tr_set_value;
	chip->chip.to_irq		= gpio_tr_to_irq;
	chip->chip.get_direction	= gpio_tr_get_direction;
	chip->chip.request		= gpio_tr_request;
	chip->chip.free			= gpio_tr_free;
	
	/* initialize undelying gpios to begin state */
	i = 0;
	
	for_each_child_of_node(root,child) {
	    struct gpio_tr_pin* pin = &(pins[i]);
	    
	    /*Get pin info from device tree record*/
		pin->input_pin  = of_get_named_gpio_flags(child, "input_pin", 0, &(pins->input_flags));
		pin->high_pin  	= of_get_named_gpio_flags(child, "high_pin", 0, &(pins->high_flags));
		pin->low_pin	= of_get_named_gpio_flags(child, "low_pin", 0, &(pins->low_flags));

		/* Initialize and check pins */
		if( gpio_is_valid(pin->input_pin) == false ){
			if ( ! deffered ) {
				deffered = 1;
				return -EPROBE_DEFER;
			}
			dev_err(dev, "Input pin(%d) in %d pin not valid",pin->input_pin, i);
			return -EINVAL;
		} 

		if( gpio_is_valid(pin->high_pin) == false ){
			if ( ! deffered ) {
				deffered = 1;
				return -EPROBE_DEFER;
			}
			dev_err(dev, "High level pin(%d) in %d pin not valid",pin->high_pin, i);
			return -EINVAL;
	    }
		
		if( gpio_is_valid(pin->low_pin) == false ){
			if ( ! deffered ) {
				deffered = 1;
				return -EPROBE_DEFER;
			}
			dev_err(dev, "Low level pin(%d) in %d pin not valid",pin->low_pin, i);
			return -EINVAL;
	    }
		dev_info( dev, "gpio-tr pin %d, input(%d) high(%d) low(%d)", i, pin->input_pin, pin->high_pin, pin->low_pin );
		i++;
	}
	
	pdata->pins = pins;
	chip->chip.ngpio = pdata->npins;
	chip->chip.can_sleep = 1;
	chip->chip.owner	= THIS_MODULE;
	chip->chip.base = -1; /* this is "virtual" gpio chip, so we aren't need to set base num explicitly.*/
	chip->chip.dev = dev;

	/* register gpio chip */
	ret = gpiochip_add( &chip->chip );

	return ret;
}

static int gpio_tr_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_tr_platform_data *pdata = dev_get_platdata(dev);
	int i;
	
	for( i=0; i < pdata->npins; i++ ){
	    struct gpio_tr_pin* pin = &(pdata->pins[i]);
	    
	    if( pin->claimed ){
		    gpio_free(pin->input_pin);
		    gpio_free(pin->high_pin);
		    gpio_free(pin->low_pin);
		    pin->claimed = 0;
	    }
	}
	
//	kfree( pdata->pins );
//	kfree( pdata->chip );
//	kfree( pdata );
	
	return 0;
}

static const struct of_device_id gpio_tr_of_match[] = {
	{ .compatible = "gpio-tr" },
	{},
};


static struct platform_driver gpio_tr_device_driver = {
	.probe		= gpio_tr_probe,
	.remove		= gpio_tr_remove,
	.driver		= {
		.name	= "gpio-tr",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_tr_of_match),
	}
};


MODULE_DEVICE_TABLE(of, gpio_tr_of_match);

static int __init gpio_tr_init(void)
{
	return platform_driver_register(&gpio_tr_device_driver);
}

static void __exit gpio_tr_exit(void)
{
	platform_driver_unregister(&gpio_tr_device_driver);
}

device_initcall(gpio_tr_init);
module_exit(gpio_tr_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Nazarenko <nnazarenko@radiofid.com>");
MODULE_DESCRIPTION("Wrapper GPIO driver over some logic");

MODULE_ALIAS("platform:gpio-tr");
