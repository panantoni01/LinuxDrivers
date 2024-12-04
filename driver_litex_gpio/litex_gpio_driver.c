#include <linux/module.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include "litex_gpio_driver.h"

#define REG_GPIO_STATE 0x0
#define REG_INTERRUPT_STATUS 0xc
#define REG_INTERRUPT_PENDING 0x10
#define REG_INTERRUPT_ENABLE 0x14

#define GPIO_MAX_MINORS 3

static int gpio_major;
static unsigned char gpio_minors[GPIO_MAX_MINORS] = { 0 };
static struct class *gpio_class;

struct gpio_device_data {
	struct cdev cdev;
	void *__iomem base;
	unsigned int counter;
	spinlock_t counter_lock;
	struct completion btn_press_completion;
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

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_device_data *gpio_data = dev_id;

	if (read_addr(gpio_data->base + REG_INTERRUPT_PENDING) == 0)
		return IRQ_NONE;

	spin_lock(&gpio_data->counter_lock);
	gpio_data->counter++;
	complete(&gpio_data->btn_press_completion);
	spin_unlock(&gpio_data->counter_lock);

	write_addr(1, gpio_data->base + REG_INTERRUPT_PENDING);
	return IRQ_HANDLED;
}

static int gpio_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct gpio_device_data *gpio_data =
		container_of(inode->i_cdev, struct gpio_device_data, cdev);
	file->private_data = gpio_data;

	spin_lock(&gpio_data->open_lock);
	if (gpio_data->opened)
		ret = -EBUSY;
	else
		gpio_data->opened++;
	spin_unlock(&gpio_data->open_lock);

	return ret;
}

static ssize_t gpio_read(struct file *file, char __user *buf, size_t count,
			 loff_t *offset)
{
	/**
    * It is assumed that only a single user application can interact with the
    * driver at a time and that it will look like this:
    * while(true) {
    *    read("/dev/litex-gpio-x", &x);
    *    printf("Caught interrupt number... %d!\n", x);
    * }
    */
	struct gpio_device_data *gpio_data =
		(struct gpio_device_data *)file->private_data;
	int result;
	size_t buf_size = count < (sizeof(result) - *offset) ?
				  count :
				  (sizeof(result) - *offset);

	wait_for_completion_interruptible(&gpio_data->btn_press_completion);

	result = gpio_data->counter;
	if (copy_to_user(buf, &result, sizeof(result)))
		return -EFAULT;

	*offset += buf_size;
	return buf_size;

	return 0;
}

static ssize_t gpio_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *offset)
{
	return 0;
}

static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct gpio_device_data *gpio_data =
		(struct gpio_device_data *)file->private_data;
	unsigned long flags;

	switch (cmd) {
	case GPIO_IOCTL_RESET:
		spin_lock_irqsave(&gpio_data->counter_lock, flags);
		gpio_data->counter = 0;
		reinit_completion(&gpio_data->btn_press_completion);
		spin_unlock_irqrestore(&gpio_data->counter_lock, flags);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int gpio_release(struct inode *inode, struct file *file)
{
	struct gpio_device_data *gpio_data = file->private_data;

	spin_lock(&gpio_data->open_lock);
	gpio_data->opened = 0;
	spin_unlock(&gpio_data->open_lock);

	return 0;
}

const struct file_operations gpio_fops = { .owner = THIS_MODULE,
					   .open = gpio_open,
					   .read = gpio_read,
					   .write = gpio_write,
					   .unlocked_ioctl = gpio_ioctl,
					   .release = gpio_release };

static int get_gpio_minor(void)
{
	unsigned int i;

	for (i = 0; i < GPIO_MAX_MINORS; i++)
		if (gpio_minors[i] == 0)
			return i;

	return -1;
}

static int gpio_driver_probe(struct platform_device *pdev)
{
	struct gpio_device_data *data;
	unsigned int minor;
	long ret, irq;
	struct resource *mem_res;

	minor = get_gpio_minor();
	if (minor == -1) {
		printk(KERN_ERR "gpio_driver: reached max number of devices\n");
		return -EIO;
	}
	gpio_minors[minor] = 1;

	data = devm_kzalloc(&pdev->dev, sizeof(struct gpio_device_data),
			    GFP_KERNEL);
	if (!data) {
		printk(KERN_ERR
		       "gpio_driver: unable to allocate driver data\n");
		ret = -ENOMEM;
		goto err_min_ret;
	}

	cdev_init(&data->cdev, &gpio_fops);
	ret = cdev_add(&data->cdev, MKDEV(gpio_major, minor), 1);
	if (ret) {
		printk(KERN_ERR "gpio_driver: cdev_add failed\n");
		goto err_min_ret;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(mem_res)) {
		printk(KERN_ERR "gpio_driver: cannot get memory resource\n");
		ret = PTR_ERR(mem_res);
		goto err_cdev_del;
	}

	data->base = devm_ioremap(&pdev->dev, mem_res->start,
				  resource_size(mem_res));
	if (IS_ERR(data->base)) {
		printk(KERN_ERR "gpio_driver: cannot remap memory resource\n");
		ret = PTR_ERR(data->base);
		goto err_cdev_del;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		printk(KERN_ERR "gpio_driver: cannot get irq resource\n");
		ret = irq;
		goto err_cdev_del;
	}

	ret = devm_request_irq(&pdev->dev, irq, gpio_irq_handler, IRQF_SHARED,
			       pdev->name, data);
	if (ret) {
		printk(KERN_ERR "gpio_driver: failed to request interrupt\n");
		goto err_cdev_del;
	}

	spin_lock_init(&data->counter_lock);
	data->counter = 0;

	spin_lock_init(&data->open_lock);
	data->opened = 0;

	init_completion(&data->btn_press_completion);

	platform_set_drvdata(pdev, data);

	if (IS_ERR(device_create(gpio_class, &pdev->dev,
				 MKDEV(gpio_major, minor), NULL,
				 "litex-gpio-%u", minor)))
		printk(KERN_ERR "gpio_driver: cannot create char device\n");

	write_addr(1, data->base + REG_INTERRUPT_ENABLE);

	printk(KERN_INFO "gpio_driver: successful probe of device: %s\n",
	       pdev->name);
	return 0;

err_cdev_del:
	cdev_del(&data->cdev);
err_min_ret:
	gpio_minors[minor] = 0;
	return ret;
}

static int gpio_driver_remove(struct platform_device *pdev)
{
	struct gpio_device_data *data;
	unsigned int minor;

	data = platform_get_drvdata(pdev);
	minor = MINOR(data->cdev.dev);

	cdev_del(&data->cdev);
	gpio_minors[minor] = 0;

	device_destroy(gpio_class, MKDEV(gpio_major, minor));

	return 0;
}

static const struct of_device_id gpio_driver_dt_ids[] = {
	{ .compatible = "litex,gpio_in" },
	{}
};

static struct platform_driver gpio_driver = {
    .driver = {
        .name = "gpio_driver",
        .of_match_table = gpio_driver_dt_ids,
    },
    .probe = gpio_driver_probe,
    .remove = gpio_driver_remove,
};

int init_module(void)
{
	int ret;
	dev_t dev;

	ret = alloc_chrdev_region(&dev, 0, GPIO_MAX_MINORS, "gpio_driver");
	if (ret != 0) {
		printk(KERN_ERR "gpio_driver: cannot allocate chrdev region\n");
		return ret;
	}
	gpio_major = MAJOR(dev);

	gpio_class = class_create("gpio");
	if (IS_ERR(gpio_class)) {
		printk(KERN_ERR "gpio_driver: cannot create gpio class\n");
		goto err_unreg;
	}

	ret = platform_driver_register(&gpio_driver);
	if (ret) {
		printk(KERN_ERR
		       "gpio_driver: error while registering the driver\n");
		goto err_cls;
	}

	printk(KERN_INFO "gpio_driver: successfully registered\n");
	return 0;

err_cls:
	class_destroy(gpio_class);
err_unreg:
	unregister_chrdev_region(gpio_major, GPIO_MAX_MINORS);
	return ret;
}

void cleanup_module()
{
	printk(KERN_INFO "gpio_driver removal\n");

	unregister_chrdev_region(gpio_major, GPIO_MAX_MINORS);
	platform_driver_unregister(&gpio_driver);
	class_destroy(gpio_class);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Antoni Pokusinski");
MODULE_DESCRIPTION("Simple driver for virtual LiteX's GPIO device");
