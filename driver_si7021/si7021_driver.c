#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include "si7021_driver.h"

#define SI7021_MAX_MINORS 2

#define SI7021_CMD_RESET 0xFE
#define SI7021_CMD_TEMP_MEASURE 0xE3
#define SI7021_CMD_HUMI_MEASURE 0xE5
#define SI7021_CMD_READ_ID_1 0xFA0F
#define SI7021_CMD_READ_ID_2 0xFCC9


static int si7021_major;
static unsigned char si7021_minors[SI7021_MAX_MINORS] = {0};
static struct class* si7021_class;

struct si7021_data {
    struct cdev cdev;
    unsigned long flags;
#define SI7021_BUSY_BIT_POS 0
    struct i2c_client *client;
};

/* Si7021 has commands that are 1- or 2-byte long */
static int si7021_send_cmd_8(struct i2c_client* client, u8 cmd) {
    return i2c_master_send(client, (char*)&cmd, sizeof(cmd));
}

static int si7021_send_cmd_16(struct i2c_client* client, u16 cmd) {
    cmd = (u16 __force)cpu_to_be16(cmd);
    return i2c_master_send(client, (char*)&cmd, sizeof(cmd));
}

static int si7021_open(struct inode *inode, struct file *file)
{
    struct si7021_data *si7021_data = container_of(inode->i_cdev, struct si7021_data, cdev);

    if (test_and_set_bit(SI7021_BUSY_BIT_POS, &si7021_data->flags))
		return -EBUSY;

    file->private_data = si7021_data;

    return 0;
}

/* Helper function to issue a temperature/humidity measure command
and read the result in a buffer */
static int si7021_measure(struct i2c_client* client, u8 cmd, char* buf, int size) {
    int ret;

    ret = si7021_send_cmd_8(client, cmd);
    if (ret < 0) {
        dev_err(&client->dev, "failed to send data to si7021\n");
        return ret;
    }
    ret = i2c_master_recv(client, buf, size);
    if (ret < 0) {
        dev_err(&client->dev, "failed to receive data from si7021\n");
        return ret;
    }

    return 0;
}

static ssize_t si7021_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    struct si7021_data* si7021_data = (struct si7021_data*)file->private_data;
    struct si7021_result result;
    int ret;

    ret = si7021_measure(si7021_data->client, SI7021_CMD_TEMP_MEASURE,
                            (char*)&result.temp, sizeof(result.temp));
    if (ret < 0)
        return ret;

    result.temp = ((unsigned int)result.temp * 17572) / 65536 - 4685;
    result.temp /= 100;

    ret = si7021_measure(si7021_data->client, SI7021_CMD_HUMI_MEASURE,
                            (char*)&result.rl_hum, sizeof(result.rl_hum));
    if (ret < 0)
        return ret;

    /* The relative humidity value must be in range <0,100> */
    result.rl_hum = clamp_val(result.rl_hum, 3145, 55574);
    result.rl_hum = ((unsigned int)result.rl_hum * 125) / 65536 - 6;

    if (copy_to_user(buf, &result, min(count, sizeof(result))))
        return -EFAULT;

    return count;
}

static ssize_t si7021_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    return 0;
}

static long si7021_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    union {
        long long read_id;
        struct {
            unsigned int read_id_low;
            unsigned int read_id_high;
        };
    } read_id;
    struct si7021_data* si7021_data = (struct si7021_data*)file->private_data;
    struct i2c_client* client = si7021_data->client;

    switch(cmd) {
        case SI7021_IOCTL_RESET:
            ret = si7021_send_cmd_8(client, SI7021_CMD_RESET);
            if (ret < 0)
                goto send_err;
            break;
        case SI7021_IOCTL_READ_ID:
            ret = si7021_send_cmd_16(client, SI7021_CMD_READ_ID_1);
            if (ret < 0)
                goto send_err;
            ret = i2c_master_recv(client, (char *)&read_id.read_id_high, 
                                    sizeof(read_id.read_id_high));
            if (ret < 0)
                goto recv_err;

            ret = si7021_send_cmd_16(client, SI7021_CMD_READ_ID_2);
            if (ret < 0)
                goto send_err;
            ret = i2c_master_recv(client, (char *)&read_id.read_id_low,
                                    sizeof(read_id.read_id_low));
            if (ret < 0)
                goto recv_err;

            if (copy_to_user((u64*)arg, &read_id.read_id, sizeof(read_id.read_id)))
                ret = -EFAULT;
            break;
        default:
            ret = -EINVAL;
    }

    return ret;
send_err:
    dev_err(&client->dev, "failed to send data to si7021\n");
    return ret;
recv_err:
    dev_err(&client->dev, "failed to receive data from si7021\n");
    return ret;
}

static int si7021_release (struct inode *inode, struct file *file)
{
    struct si7021_data* si7021_data = file->private_data;

    clear_bit(SI7021_BUSY_BIT_POS, &si7021_data->flags);
    return 0;
}

const struct file_operations si7021_fops = {
    .owner = THIS_MODULE,
    .open = si7021_open,
    .read = si7021_read,
    .write = si7021_write,
    .unlocked_ioctl = si7021_ioctl,
    .release = si7021_release
};

static int get_si7021_minor(void)
{
    unsigned int i;

    for (i = 0; i < SI7021_MAX_MINORS; i++)
        if (si7021_minors[i] == 0)
            return i;

    return -1;
}

static int si7021_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    long ret;
    struct si7021_data* data;
    unsigned int minor;

    minor = get_si7021_minor();
    if (minor == -1) {
        ret = -EIO;
        dev_err_probe(&client->dev, ret, "reached max number of devices\n");
        return ret;
    }
    si7021_minors[minor] = 1;

    data = devm_kzalloc(&client->dev, sizeof(struct si7021_data), GFP_KERNEL);
    if (!data) {
        ret = -ENOMEM;
        dev_err_probe(&client->dev, ret, "unable to allocate driver data\n");
        goto err_min_ret;
    }

    /* reset the device and wait 15ms which is the powerup time 
    after issuing a software reset command */
    si7021_send_cmd_8(client, SI7021_CMD_RESET);
    msleep(15);

    cdev_init(&data->cdev, &si7021_fops);
    ret = cdev_add(&data->cdev, MKDEV(si7021_major, minor), 1);
    if (ret) {
        dev_err_probe(&client->dev, ret, "cdev_add failed\n");
        goto err_min_ret;
    }

    data->client = client;

    i2c_set_clientdata(client, data);

    ret = IS_ERR(device_create(si7021_class, &client->dev,
                             MKDEV(si7021_major, minor), NULL,
                             "si7021-%u", minor));
    if (IS_ERR((void*)ret))
        dev_err_probe(&client->dev, ret, "cannot create char device\n");

    dev_info(&client->dev, "successful probe of device: %s\n",
                    client->name);
    return 0;

err_min_ret:
    si7021_minors[minor] = 0;
    return ret;
}

static int si7021_remove(struct i2c_client *client)
{
    struct si7021_data* data;
    struct cdev* cdev;
    unsigned int minor;

    data = i2c_get_clientdata(client);
    cdev = &data->cdev;
    minor = MINOR(cdev->dev);
    
    cdev_del(&data->cdev);
    si7021_minors[minor] = 0;

    device_destroy(si7021_class, MKDEV(si7021_major, minor));

    return 0;
}


static const struct of_device_id si7021_dt_ids[] = {
	{ .compatible = "si7021" },
	{ }
};
 MODULE_DEVICE_TABLE(of, si7021_dt_ids); /* https://stackoverflow.com/questions/22901282 */

static struct i2c_driver si7021_driver = {
	.driver = {
		.name = "si7021",
		.of_match_table = si7021_dt_ids,
	},
	.probe = si7021_probe,
	.remove = si7021_remove,
};

static int __init si7021_init(void)
{
    int ret;
    dev_t dev;

    ret = alloc_chrdev_region(&dev, 0, SI7021_MAX_MINORS, "si7021_driver");
    if (ret != 0) {
        printk(KERN_ERR "si7021_driver: cannot allocate chrdev region\n");
        return ret;
    }
    si7021_major = MAJOR(dev);

    si7021_class = class_create(THIS_MODULE, "thermal");
    if (IS_ERR(si7021_class)) {
        printk(KERN_ERR "si7021_driver: cannot create si7021 class\n");
        goto err_unreg;
    }

    ret = i2c_add_driver(&si7021_driver);
    if (ret) {
        printk(KERN_ERR "si7021_driver: error while registering the driver\n");
        goto err_cls;
    }

    printk(KERN_INFO "si7021_driver: successfully registered\n");
    return 0;

err_cls:
    class_destroy(si7021_class);
err_unreg:
    unregister_chrdev_region(si7021_major, SI7021_MAX_MINORS);
    return ret;
}

static void __exit si7021_cleanup(void)
{
    printk(KERN_INFO "si7021_driver removal\n");

	unregister_chrdev_region(si7021_major, SI7021_MAX_MINORS);
    i2c_del_driver(&si7021_driver);
	class_destroy(si7021_class);
}

module_init(si7021_init);
module_exit(si7021_cleanup);


MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("Antoni Pokusinski");
MODULE_DESCRIPTION ("si7021 driver");
