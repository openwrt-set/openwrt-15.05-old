/*

Driver for GPIO through levelshifters using two pins from SoC for direction and value

*/
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio-lv.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

static int deffered = 0;

int gpio_lv_get_direction(struct gpio_chip *chip, unsigned offset ){
    struct gpio_lv_platform_data *pdata = (struct gpio_lv_platform_data *)dev_get_platdata(chip->dev);
    
    struct gpio_lv_pin* pin = &pdata->pins[offset];
    dev_dbg(chip->dev, "get direction (%d) pin input(%d) direction(%d)", offset, pin->input_pin, pin->direction_pin);
    
    return !gpio_get_value( pin->direction_pin );
}

int gpio_lv_dir_input(struct gpio_chip *chip, unsigned offset ){
    const struct gpio_lv_platform_data *pdata = dev_get_platdata(chip->dev);
    
    struct gpio_lv_pin* pin = &pdata->pins[offset];
    dev_dbg(chip->dev, "set direction input (%d) pin input(%d) direction(%d)", offset, pin->input_pin, pin->direction_pin);
    
    gpio_set_value( pin->direction_pin, 0);
    gpio_direction_input( pin->input_pin);
    
    return 0;
}
int gpio_lv_dir_output(struct gpio_chip *chip, unsigned offset, int value ){
    const struct gpio_lv_platform_data *pdata = dev_get_platdata(chip->dev);
    
    struct gpio_lv_pin* pin = &pdata->pins[offset];
    dev_dbg(chip->dev, "set direction output (%d) pin input(%d) direction(%d)", offset, pin->input_pin, pin->direction_pin);
    
    gpio_set_value( pin->direction_pin, 1);
    gpio_direction_output( pin->input_pin, value);
    
    return 0;
}
int gpio_lv_get_value(struct gpio_chip *chip, unsigned offset ){
    const struct gpio_lv_platform_data *pdata = dev_get_platdata(chip->dev);
    
    return gpio_get_value( pdata->pins[offset].input_pin);    
}
void gpio_lv_set_value(struct gpio_chip *chip, unsigned offset, int value ){
    const struct gpio_lv_platform_data *pdata = dev_get_platdata(chip->dev);
    gpio_set_value( pdata->pins[offset].input_pin, value);
    
    return;
}

static int gpio_lv_request(struct gpio_chip *chip, unsigned offset) {
    const struct gpio_lv_platform_data *pdata = dev_get_platdata(chip->dev);

    if( pdata == NULL ){
		dev_err(chip->dev, "Platform data return NULL." );
		return -EFAULT;
    }
	
    struct gpio_lv_pin* pin = &pdata->pins[offset];
	
	dev_dbg(chip->dev, "Request (%d) pin input(%d) output(%d)", offset, pin->input_pin, pin->direction_pin);
	
	if( gpio_request(pin->input_pin, NULL) != 0 ){
	    dev_err(chip->dev, "Input pin(%d) in lv-pin %d already in use",pin->input_pin, offset);
	    return -EBUSY;
	}

	if( gpio_request(pin->direction_pin, NULL) != 0 ){
	    gpio_free(pin->input_pin);
	    dev_err(chip->dev, "Direction pin(%d) in lv-pin %d already in use",pin->direction_pin, offset);
	    return -EBUSY;
	}
	
	gpio_direction_output(pin->direction_pin, 0);
	
    pin->claimed = 1;

	return 0;
}

void gpio_lv_free(struct gpio_chip *chip, unsigned offset){
    const struct gpio_lv_platform_data *pdata = dev_get_platdata(chip->dev);
    struct gpio_lv_pin* pin = &pdata->pins[offset];
    
    if( pin->claimed ){
        gpio_free(pin->input_pin);
        gpio_free(pin->direction_pin);
        pin->claimed = 0;
    }
    
    return;
}

static int gpio_lv_to_irq( struct gpio_chip *chip, unsigned offset ){
    const struct gpio_lv_platform_data *pdata = dev_get_platdata(chip->dev);
    return gpio_to_irq( pdata->pins[offset].input_pin);
}

static int gpio_lv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_lv_platform_data *pdata = NULL;
	struct device_node *root = dev->of_node;
	struct device_node *child;
	struct gpio_lv_pin* pins;
	struct gpio_lv* chip;
	
	int i, ret;

	if ( !root ){
	    dev_err(dev,"No device tree data available\n");
	    return -EINVAL;
	}
	
    chip = devm_kzalloc(dev, sizeof( struct gpio_lv ), GFP_KERNEL);
	
	if( !chip ){
		dev_err(dev,"failed to allocate lv_chip struct\n");
		return -ENOMEM;
	}
	
	if( dev->platform_data == NULL){
		dev->platform_data = devm_kzalloc( dev, sizeof( struct gpio_lv_platform_data ), GFP_KERNEL);
	}
	
	pdata = dev->platform_data;
	
	/* Get count of declared pins*/
	pdata->npins = of_get_child_count(root);

	pins = devm_kzalloc( dev, sizeof( struct gpio_lv_pin ) * pdata->npins, GFP_KERNEL);

	if( !pins ){
	    dev_err(dev,"failed to allocate pins struct\n");
	    return -ENOMEM;
	}
	
	/* set handlers for gpio operations*/
	chip->chip.direction_input	= gpio_lv_dir_input;
	chip->chip.direction_output	= gpio_lv_dir_output;
	chip->chip.get				= gpio_lv_get_value;
	chip->chip.set				= gpio_lv_set_value;
    chip->chip.to_irq			= gpio_lv_to_irq;
	chip->chip.get_direction	= gpio_lv_get_direction;
	chip->chip.request			= gpio_lv_request;
	chip->chip.free				= gpio_lv_free;
	
	/* initialize undelying gpios to begin state */
	i = 0;
	
	for_each_child_of_node(root,child) {
	    struct gpio_lv_pin* pin = &(pins[i]);
	    
	    /*Get pin info from device tree record*/
		pin->input_pin      = of_get_named_gpio_flags(child, "input_pin", 0, &(pins->input_flags));
		pin->direction_pin  = of_get_named_gpio_flags(child, "direction_pin", 0, &(pins->direction_flags));

		/* Initialize and check pins */
		if( gpio_is_valid(pin->input_pin) == false ){
			if ( ! deffered ) {
				deffered = 1;
				return -EPROBE_DEFER;
			}
			dev_err(dev, "Input pin(%d) in %d pin not valid",pin->input_pin, i);
			return -EINVAL;
		} 

		if( gpio_is_valid(pin->direction_pin) == false ){
			if ( ! deffered ) {
				deffered = 1;
				return -EPROBE_DEFER;
			}
			dev_err(dev, "Direction pin(%d) in %d pin not valid",pin->direction_pin, i);
			return -EINVAL;
	    }
		dev_info( dev, "gpio-lv pin %d, input(%d) dir(%d)", i, pin->input_pin, pin->direction_pin );
		i++;
	}
	
	pdata->pins = pins;
	chip->chip.ngpio = pdata->npins;
	chip->chip.owner	= THIS_MODULE;
	chip->chip.base = -1; /* this is "virtual" gpio chip, so we aren't need to set base num explicitly.*/
	chip->chip.dev = dev;

	/* register gpio chip */
	ret = gpiochip_add( &chip->chip );

	return ret;
}

static int gpio_lv_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_lv_platform_data *pdata = dev_get_platdata(dev);
	int i;
	
	for( i=0; i < pdata->npins; i++ ){
	    struct gpio_lv_pin* pin = &(pdata->pins[i]);
	    
	    if( pin->claimed ){
		    gpio_free(pin->input_pin);
		    gpio_free(pin->direction_pin);
		    pin->claimed = 0;
	    }
	}
	
//	kfree( pdata->pins );
//	kfree( pdata->chip );
//	kfree( pdata );
	
	return 0;
}

static const struct of_device_id gpio_lv_of_match[] = {
	{ .compatible = "gpio-lv" },
	{},
};


static struct platform_driver gpio_lv_device_driver = {
	.probe		= gpio_lv_probe,
	.remove		= gpio_lv_remove,
	.driver		= {
		.name	= "gpio-lv",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_lv_of_match),
	}
};


MODULE_DEVICE_TABLE(of, gpio_lv_of_match);

static int __init gpio_lv_init(void)
{
	return platform_driver_register(&gpio_lv_device_driver);
}

static void __exit gpio_lv_exit(void)
{
	platform_driver_unregister(&gpio_lv_device_driver);
}

device_initcall(gpio_lv_init);
module_exit(gpio_lv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Nazarenko <nnazarenko@radiofid.com>");
MODULE_DESCRIPTION("Wrapper GPIO driver over LV*** levelshifter");
MODULE_ALIAS("platform:gpio-lv");
