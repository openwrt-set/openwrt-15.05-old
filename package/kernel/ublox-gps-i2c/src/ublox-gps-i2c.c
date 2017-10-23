#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define UBLOX_GPS_NUM 1
#define UBLOX_GPS_MAJOR 0
#define READ_TIME 100 
#define SIZE_BUFFER 32
#define MAX_SIZE_BUFFER 1024

struct i2c_client *ublox_client;

struct tty_driver *ublox_tty;
struct tty_port ublox_port;

static int ublox_gps_is_open;

static void ublox_gps_write_read(struct work_struct *private);

static DECLARE_DELAYED_WORK(ublox_gps_wr, ublox_gps_write_read);

static int __ublox_read(u8 *val, int count)
{
    int ret = 0;
    int i;
    val[0] = 0xFF;
    ret = i2c_master_recv(ublox_client, &val[0], 1);
    if (val[0] == 0xFF)
        return -1;
    ret = i2c_master_recv(ublox_client, &val[1], count - 1);
    if (ret < 0)
        return 1;
    return 0;
}

static int ublox_read(u8 *val, int size)
{
    int ret = 0;
    int count = 0;

    while ((count + SIZE_BUFFER) < size) {
        ret = __ublox_read(&val[count], SIZE_BUFFER);
        if (ret < 0)
            break;
        count += SIZE_BUFFER;
    }
    return count;
}

static int ublox_write(u8 *val, int count)
{
    int i, ret = 0;
    if (count <= 0)
        return ret;

    for (i = 0; i < count; i++) {
        if (val[i] == 0xFF)
            continue;
        val[ret] = val[i];
        ret++;
    }
    tty_insert_flip_string(&ublox_port, val, ret);
    return ret;
}

static void ublox_gps_write_read(struct work_struct *private)
{
    s16 count = 0;
    u8 buf[MAX_SIZE_BUFFER];
    int ret = 0;
    int is_push = 0;
    if (!ublox_gps_is_open)
        return;

    while ((ret = ublox_read(buf, MAX_SIZE_BUFFER)) > 0) {
        ublox_write(buf, ret);
        is_push = 1;
    }
    if(is_push)
        tty_flip_buffer_push(&ublox_port);

    schedule_delayed_work(&ublox_gps_wr, msecs_to_jiffies(READ_TIME));
}

static int ublox_gps_serial_install(struct tty_driver *driver, struct tty_struct *tty)
{
    tty_port_init(&ublox_port);
    tty->port = &ublox_port;
    return tty_standard_install(driver, tty);
}

static int ublox_gps_serial_open(struct tty_struct *tty, struct file *flp)
{
    if (!tty)
        return -ENOMEM;

    ublox_port.low_latency = true;
    ublox_gps_is_open = true;

    schedule_delayed_work(&ublox_gps_wr, 0);
    return 0;
}

static void ublox_gps_serial_close(struct tty_struct *tty, struct file *flp)
{
    if(!ublox_gps_is_open)
        return;

    ublox_gps_is_open = false;
}

static int ublox_gps_serial_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
    if(!ublox_gps_is_open)
        return 0;

    return count;
}

static int ublox_gps_write_room(struct tty_struct *tty)
{
    if (!ublox_gps_is_open)
        return 0;
    return 1024;
}

static const struct tty_operations ublox_gps_serial_ops = {
    .install = ublox_gps_serial_install,
    .open = ublox_gps_serial_open,
    .close = ublox_gps_serial_close,
    .write = ublox_gps_serial_write,
    .write_room = ublox_gps_write_room
};

static int ublox_gps_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret = 0;
    ublox_tty = alloc_tty_driver(UBLOX_GPS_NUM);

    ublox_tty->driver_name = "ublox_gps";
    ublox_tty->name = "ttyGNSS";
    ublox_tty->major = UBLOX_GPS_MAJOR;
    ublox_tty->minor_start = 0;
    ublox_tty->type = TTY_DRIVER_TYPE_SERIAL;
    ublox_tty->subtype = SERIAL_TYPE_NORMAL;
    ublox_tty->flags = TTY_DRIVER_REAL_RAW;
    ublox_tty->init_termios = tty_std_termios;
    ublox_tty->init_termios.c_cflag = B38400 | CS8 | CREAD | HUPCL | CLOCAL;
    ublox_tty->init_termios.c_ispeed = 38400;
    ublox_tty->init_termios.c_ospeed = 38400;

    tty_set_operations(ublox_tty, &ublox_gps_serial_ops); 

    ret = tty_register_driver(ublox_tty);
    if (ret) {
        printk(KERN_ERR "failed to registers ublox_gps_tty_driver");
        goto err_out;
    }

    if (!i2c_check_functionality(client->adapter,
                I2C_FUNC_SMBUS_BYTE_DATA)) {
        printk(KERN_ERR "SMBUS Byte Data no Suppported");
        goto err_out;
    }

    ublox_client = client;
    ublox_gps_is_open = false;

    return ret;
err_out:
    put_tty_driver(ublox_tty);
    return -EINVAL;
}

static int ublox_gps_remove(struct i2c_client *client)
{
    tty_unregister_driver(ublox_tty);
    put_tty_driver(ublox_tty);

    ublox_client = NULL;

    return 0;
}

static const struct i2c_device_id ublox_gps_id[] = {
    { "neo", 0x42 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ublox_gps_id );

static const struct of_device_id ublox_gps_of_match[] = {
    { .compatible = "ublox,neo" },
    { }
};
MODULE_DEVICE_TABLE(of, ublox_gps_of_match);

static struct i2c_driver ublox_gps_i2c_driver = {
    .driver = {
        .name = "ublox_gps",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ublox_gps_of_match)
    },
    .probe = ublox_gps_probe,
    .remove = ublox_gps_remove,
    .id_table = ublox_gps_id
};

module_i2c_driver(ublox_gps_i2c_driver);

MODULE_AUTHOR("Dmitriy Peresypkin <dperesypkin@radiofid.ru>");
MODULE_DESCRIPTION("UBLOX GPS over i2c driver");
MODULE_LICENSE("GPL");
