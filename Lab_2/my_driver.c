#include <linux/module.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <asm/ioctl.h>
#include "my_driver.h"


#define MY_MAJOR 140
#define MY_MAX_MINORS 3


struct my_device_data {
    struct cdev cdev;
    long var;
    unsigned int curr_op;
};

static int my_open(struct inode *inode, struct file *file)
{
    struct my_device_data *my_data = container_of(inode->i_cdev, struct my_device_data, cdev);
    file->private_data = my_data;

    return 0;
}

static ssize_t my_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    struct my_device_data *my_data = (struct my_device_data *)file->private_data;
    size_t buf_size = count < (sizeof(my_data->var) - *offset) ? count : (sizeof(my_data->var) - *offset);
    
    if(copy_to_user(buf, &my_data->var, sizeof(long)))
        return -EFAULT;

    *offset += buf_size;
    return buf_size;
}

static ssize_t my_write(struct file *file, const char __user *buf, size_t count, loff_t *offset) 
{
    long num_buf;
    size_t buf_size = count < sizeof(num_buf) ? count : sizeof(num_buf);
    struct my_device_data *my_data = (struct my_device_data *)file->private_data;

    if(copy_from_user(&num_buf, buf, buf_size))
        return -EFAULT;    

    switch (my_data->curr_op) {
        case ADD:
            my_data->var += num_buf;
            break;
        case SUB:
            my_data->var -= num_buf;
            break;
        case MUL:
            my_data->var *= num_buf;
            break;
        case DIV:
            if (num_buf == 0) {
                printk(KERN_ERR"Div0 attempt!\n");
                return buf_size;
            }
            my_data->var /= num_buf;
            break;
        default:
            break;
    }

    return buf_size;
}

static long my_ioctl (struct file *file, unsigned int cmd, unsigned long arg) 
{
    struct my_device_data *my_data = (struct my_device_data *)file->private_data;
    
    switch(cmd) {
        case MY_IOCTL_RESET:
            my_data->var = 0;
            break;
        case MY_IOCTL_CHANGE_OP:
            my_data->curr_op = (unsigned int)arg;
            if (my_data->curr_op > DIV)
                return -EINVAL;
            break;
        default:
            return -EINVAL;
    }
    return 0;
}

static int my_release (struct inode *inode, struct file *file)
{
    return 0;
}

const struct file_operations my_fops = {
    .owner = THIS_MODULE,
    .open = my_open,
    .read = my_read,
    .write = my_write,
    .unlocked_ioctl = my_ioctl,
    .release = my_release
};

struct my_device_data devs[MY_MAX_MINORS];

static int my_driver_probe(struct platform_device *pdev)
{
    struct resource* res;
    int i, err;
    err = register_chrdev_region(MKDEV(MY_MAJOR, 0), MY_MAX_MINORS, "my_device_driver");
    if (err != 0) 
        return err;

    for(i = 0; i < MY_MAX_MINORS; i++) {
        cdev_init(&devs[i].cdev, &my_fops);
        cdev_add(&devs[i].cdev, MKDEV(MY_MAJOR, i), 1);
        devs[i].var = 0;
        devs[i].curr_op = ADD;
    }
    return 0;
}

static int my_driver_remove(struct platform_device *pdev)
{
    int i;
    for(i = 0; i < MY_MAX_MINORS; i++)
        cdev_del(&devs[i].cdev);
    unregister_chrdev_region(MKDEV(MY_MAJOR, 0), MY_MAX_MINORS);
    return 0;
}

static const struct of_device_id my_driver_dt_ids[] = 
{
    { .compatible = "uwr,my-driver" },
    {}
};

static struct platform_driver my_driver = {
    .driver = {
        .name = "my_awesome_driver" ,
        .of_match_table = my_driver_dt_ids,
    },
    .probe = my_driver_probe,
    .remove = my_driver_remove,
};

int init_module ()
{
    printk(KERN_ERR"Registering the driver!\n");
    return platform_driver_register(&my_driver);
}

void cleanup_module ()
{
    printk(KERN_ERR"Unregistering the driver!\n");
    platform_driver_unregister(&my_driver);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("Antoni Pokusinski");
MODULE_DESCRIPTION ("My first kernel module");
