#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/io.h>

#include "mydriver.h"

#define STATUS_REG_OFFSET    0x00
#define OPERATION_REG_OFFSET 0x04
#define DAT0_REG_OFFSET      0x08
#define DAT1_REG_OFFSET      0x0c
#define RESULT_REG_OFFSET    0x10


#define MY_MAJOR 140
#define MY_MAX_MINORS 1

static DEFINE_SPINLOCK(open_lock);
static int is_opened = 0;

struct my_device_data {
    struct cdev cdev;
    void __iomem *base;
};

static inline void write_addr(u32 val, void __iomem *addr)
{
    writel((u32 __force)cpu_to_le32(val), addr);
}
static inline u32 read_addr(void __iomem *addr)
{
    return le32_to_cpu(( __le32 __force)readl(addr));
}

static int my_open(struct inode *inode, struct file *file)
{
    struct my_device_data *my_data = container_of(inode->i_cdev, struct my_device_data, cdev);
    file->private_data = my_data;

    /* we dont want to open the device >1 times - interference could occur */
    spin_lock(&open_lock);
    if (!is_opened)
        is_opened = 1;
    else
        return -EPERM;
    spin_unlock(&open_lock);
    
    return 0;
}

static ssize_t my_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    struct my_device_data *my_data = (struct my_device_data *)file->private_data;
    u32 result = read_addr(my_data->base + RESULT_REG_OFFSET);
    size_t buf_size = count < (sizeof(result) - *offset) ? count : (sizeof(result) - *offset);
    
    if(copy_to_user(buf, &result, sizeof(result)))
        return -EFAULT;

    *offset += buf_size;
    return buf_size;
}

static ssize_t my_write(struct file *file, const char __user *buf, size_t count, loff_t *offset) 
{
    u32 old_dat1, new_dat1 = 0;
    size_t buf_size = count < sizeof(new_dat1) ? count : sizeof(new_dat1);
    struct my_device_data *my_data = (struct my_device_data *)file->private_data;

    if(copy_from_user(&new_dat1, buf, buf_size))
        return -EFAULT;    

    old_dat1 = read_addr(my_data->base + DAT1_REG_OFFSET);
    write_addr(old_dat1, my_data->base + DAT0_REG_OFFSET);
    write_addr(new_dat1, my_data->base + DAT1_REG_OFFSET);

    return buf_size;
}

static long my_ioctl (struct file *file, unsigned int cmd, unsigned long arg) 
{
    u32 status;
    struct my_device_data *my_data = (struct my_device_data *)file->private_data;
    
    switch(cmd) {
        case MY_IOCTL_RESET:
            write_addr((u32)arg, my_data->base + STATUS_REG_OFFSET);
            break;
        case MY_IOCTL_CHOOSE_OP:
            write_addr((u32)arg, my_data->base + OPERATION_REG_OFFSET);
            break;
        case MY_IOCTL_CHECK_STATUS:
            status = read_addr(my_data->base + STATUS_REG_OFFSET);
            if (copy_to_user((u32*)arg, &status, sizeof(status)))
                return -EFAULT;
            break;
        default:
            return -EINVAL;
    }
    return 0;
}

static int my_release (struct inode *inode, struct file *file)
{
    is_opened = 0;
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

static struct class *my_class = NULL;
struct my_device_data devs[MY_MAX_MINORS];

static int my_driver_probe(struct platform_device *pdev)
{
    struct resource *res;
    void __iomem *base;
    int i, err;

    err = register_chrdev_region(MKDEV(MY_MAJOR, 0), MY_MAX_MINORS, "my_device_driver");
    if (err != 0)
        return err;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(base))
        return PTR_ERR(base);

    my_class = class_create(THIS_MODULE, "chardev");

    for(i = 0; i < MY_MAX_MINORS; i++) {
        cdev_init(&devs[i].cdev, &my_fops);
        cdev_add(&devs[i].cdev, MKDEV(MY_MAJOR, i), 1);
        devs[i].base = base;
        device_create(my_class, NULL, MKDEV(MY_MAJOR, i), NULL, "chardev%d", i);
    }  

    return 0;
}

static int my_driver_remove(struct platform_device *pdev)
{
    int i;
    for(i = 0; i < MY_MAX_MINORS; i++) {
        cdev_del(&devs[i].cdev);
        device_destroy(my_class, MKDEV(MY_MAJOR, i));
    }

    class_destroy(my_class);

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
        .name = "my_device_driver" ,
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
MODULE_DESCRIPTION ("Simple driver implementing char device API");
