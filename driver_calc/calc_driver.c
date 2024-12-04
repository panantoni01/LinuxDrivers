#include <linux/module.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <asm/ioctl.h>
#include <asm/io.h>
#include "calc_driver.h"

#define STATUS_REG_OFFSET 0x00
#define OPERATION_REG_OFFSET 0x04
#define DAT0_REG_OFFSET 0x08
#define DAT1_REG_OFFSET 0x0c
#define RESULT_REG_OFFSET 0x10

static int calc_major;

#define CALC_MAX_MINORS 3

static unsigned char calc_minors[CALC_MAX_MINORS] = { 0 };

struct calc_device_data {
	struct cdev cdev;
	void *__iomem base;
	unsigned int opened;
	spinlock_t open_lock;
};

static inline void write_addr(u32 val, void __iomem *addr)
{
	writel((u32 __force)cpu_to_le32(val), addr);
}

static inline u32 read_addr(void __iomem *addr)
{
	return le32_to_cpu((__le32 __force)readl(addr));
}

static int calc_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct calc_device_data *calc_data =
		container_of(inode->i_cdev, struct calc_device_data, cdev);
	file->private_data = calc_data;

	/* we dont want to open the device >1 times - interference could occur */
	spin_lock(&calc_data->open_lock);
	if (calc_data->opened)
		ret = -EBUSY;
	else
		calc_data->opened++;
	spin_unlock(&calc_data->open_lock);

	return ret;
}

static inline void *__iomem get_base_ptr(struct file *file)
{
	return ((struct calc_device_data *)file->private_data)->base;
}

static ssize_t calc_read(struct file *file, char __user *buf, size_t count,
			 loff_t *offset)
{
	void *base_ptr = get_base_ptr(file);
	u32 result = read_addr(base_ptr + RESULT_REG_OFFSET);
	size_t buf_size = count < (sizeof(result) - *offset) ?
				  count :
				  (sizeof(result) - *offset);

	if (copy_to_user(buf, &result, sizeof(result)))
		return -EFAULT;

	*offset += buf_size;
	return buf_size;
}

static ssize_t calc_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *offset)
{
	u32 old_data_reg1, user_data = 0;
	size_t buf_size = count < sizeof(user_data) ? count : sizeof(user_data);
	void *base_ptr = get_base_ptr(file);

	if (copy_from_user(&user_data, buf, buf_size))
		return -EFAULT;

	/* Transfer DAT1_REG->DAT0_REG and write user data to DAT1_REG */
	old_data_reg1 = read_addr(base_ptr + DAT1_REG_OFFSET);
	write_addr(old_data_reg1, base_ptr + DAT0_REG_OFFSET);
	write_addr(user_data, base_ptr + DAT1_REG_OFFSET);

	return buf_size;
}

static long calc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void *base_ptr = get_base_ptr(file);
	u32 status;

	switch (cmd) {
	case CALC_IOCTL_RESET:
		write_addr((u32)STATUS_MASK_ALL, base_ptr + STATUS_REG_OFFSET);
		break;
	case CALC_IOCTL_CHANGE_OP:
		write_addr((u32)arg, base_ptr + OPERATION_REG_OFFSET);
		break;
	case CALC_IOCTL_CHECK_STATUS:
		status = read_addr(base_ptr + STATUS_REG_OFFSET);
		if (copy_to_user((u32 *)arg, &status, sizeof(status)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int calc_release(struct inode *inode, struct file *file)
{
	struct calc_device_data *calc_data = file->private_data;

	spin_lock(&calc_data->open_lock);
	calc_data->opened = 0;
	spin_unlock(&calc_data->open_lock);

	return 0;
}

static struct class *calc_class;

const struct file_operations calc_fops = { .owner = THIS_MODULE,
					   .open = calc_open,
					   .read = calc_read,
					   .write = calc_write,
					   .unlocked_ioctl = calc_ioctl,
					   .release = calc_release };

static int get_calc_minor(void)
{
	unsigned int i;

	for (i = 0; i < CALC_MAX_MINORS; i++)
		if (calc_minors[i] == 0)
			return i;

	return -1;
}

static int calc_driver_probe(struct platform_device *pdev)
{
	struct calc_device_data *data;
	unsigned int minor;
	long ret;
	struct resource *mem_res;

	minor = get_calc_minor();
	if (minor == -1) {
		printk(KERN_ERR "calc_driver: reached max number of devices\n");
		return -EIO;
	}
	calc_minors[minor] = 1;

	data = devm_kzalloc(&pdev->dev, sizeof(struct calc_device_data),
			    GFP_KERNEL);
	if (!data) {
		printk(KERN_ERR
		       "calc_driver: unable to allocate driver data\n");
		ret = -ENOMEM;
		goto err_min_ret;
	}

	cdev_init(&data->cdev, &calc_fops);
	ret = cdev_add(&data->cdev, MKDEV(calc_major, minor), 1);
	if (ret) {
		printk(KERN_ERR "calc_driver: cdev_add failed\n");
		goto err_min_ret;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(mem_res)) {
		printk(KERN_ERR "calc_driver: cannot get memory resource\n");
		ret = PTR_ERR(mem_res);
		goto err_cdev_del;
	}

	data->base = devm_ioremap_resource(&pdev->dev, mem_res);
	if (IS_ERR(data->base)) {
		printk(KERN_ERR "calc_driver: cannot remap memory resource\n");
		ret = PTR_ERR(data->base);
		goto err_cdev_del;
	}

	spin_lock_init(&data->open_lock);
	data->opened = 0;

	platform_set_drvdata(pdev, data);

	if (IS_ERR(device_create(calc_class, &pdev->dev,
				 MKDEV(calc_major, minor), NULL, "calc-%u",
				 minor)))
		printk(KERN_ERR "calc_driver: cannot create char device\n");

	printk(KERN_INFO "calc_driver: successful probe of device: %s\n",
	       pdev->name);
	return 0;

err_cdev_del:
	cdev_del(&data->cdev);
err_min_ret:
	calc_minors[minor] = 0;
	return ret;
}

static int calc_driver_remove(struct platform_device *pdev)
{
	struct calc_device_data *data;
	unsigned int minor;

	data = platform_get_drvdata(pdev);
	minor = MINOR(data->cdev.dev);

	cdev_del(&data->cdev);
	calc_minors[minor] = 0;

	device_destroy(calc_class, MKDEV(calc_major, minor));

	return 0;
}

static const struct of_device_id calc_driver_dt_ids[] = {
	{ .compatible = "calc-driver" },
	{}
};

static struct platform_driver calc_driver = {
    .driver = {
        .name = "calc_driver",
        .of_match_table = calc_driver_dt_ids,
    },
    .probe = calc_driver_probe,
    .remove = calc_driver_remove,
};

int init_module(void)
{
	int ret;
	dev_t dev;

	ret = alloc_chrdev_region(&dev, 0, CALC_MAX_MINORS, "calc_driver");
	if (ret != 0) {
		printk(KERN_ERR "calc_driver: cannot allocate chrdev region\n");
		return ret;
	}
	calc_major = MAJOR(dev);

	calc_class = class_create("calc_class");
	if (IS_ERR(calc_class)) {
		printk(KERN_ERR "calc_driver: cannot create calc_class\n");
		goto err_unreg;
	}

	ret = platform_driver_register(&calc_driver);
	if (ret) {
		printk(KERN_ERR
		       "calc_driver: error while registering the driver\n");
		goto err_cls;
	}

	printk(KERN_INFO "calc_driver: successfully registered\n");
	return 0;

err_cls:
	class_destroy(calc_class);
err_unreg:
	unregister_chrdev_region(calc_major, CALC_MAX_MINORS);
	return ret;
}

void cleanup_module()
{
	printk(KERN_INFO "calc_driver removal\n");

	unregister_chrdev_region(calc_major, CALC_MAX_MINORS);
	platform_driver_unregister(&calc_driver);
	class_destroy(calc_class);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Antoni Pokusinski");
MODULE_DESCRIPTION("Simple driver for virtual calc device");
