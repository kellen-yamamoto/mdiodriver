#include <linux/module.h>
#include <linux/mdio-bitbang.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>

#define MDIO_READ 2
#define MDIO_WRITE 1

#define MDIO_C45 (1<<15)
#define MDIO_C45_ADDR (MDIO_C45 | 0)
#define MDIO_C45_READ (MDIO_C45 | 3)
#define MDIO_C45_WRITE (MDIO_C45 | 1)

#define MDIO_SETUP_TIME 10
#define MDIO_HOLD_TIME 10

#define GPIO1 17
#define GPIO2 27
#define GPIO3 23
#define GPIO4 24
#define GPIO_MDIO GPIO2
#define GPIO_MDC GPIO4
#define GPIO_RESERVED1 GPIO1
#define GPIO_RESERVED2 GPIO3


#define MDIO_DELAY 250

#define MDIO_READ_DELAY 400

static struct platform_device *platform_device;

/*------- GPIO Functions ------*/
static void setmdc(int val)
{
    gpio_set_value(GPIO_MDC, val);
}

static void setmdio(int val)
{
    gpio_set_value(GPIO_MDIO, val);
}


static int getmdio(void)
{
    return gpio_get_value(GPIO_MDIO);
}


/* MDIO must already be configured as output. */
static void mdiobb_send_bit(int val)
{
    setmdio(val);
    ndelay(MDIO_DELAY);
    setmdc(1);
    ndelay(MDIO_DELAY);
    setmdc(0);
}

/* MDIO must already be configured as input. */
static int mdiobb_get_bit(void)
{
    ndelay(MDIO_DELAY);
    setmdc(1);
    ndelay(MDIO_READ_DELAY);
    setmdc(0);

    return getmdio();
}

/* MDIO must already be configured as output. */
static void mdiobb_send_num(u16 val, int bits)
{
    int i;

    for (i = bits - 1; i >= 0; i--)
        mdiobb_send_bit((val >> i) & 1);
}

/* MDIO must already be configured as input. */
static u16 mdiobb_get_num(int bits)
{
    int i;
    u16 ret = 0;

    for (i = bits - 1; i >= 0; i--) {
        ret <<= 1;
        ret |= mdiobb_get_bit();
    }

    return ret;
}

/* Utility to send the preamble, address, and
 * register (common to read and write).
 */
static void mdiobb_cmd(int op, u8 PHY, u8 reg)
{
    int i;
    gpio_direction_output(GPIO_MDIO, 1);	
    /*
     * Send a 32 bit preamble ('1's) with an extra '1' bit for good
     * measure.  The IEEE spec says this is a PHY optional
     * requirement.  The AMD 79C874 requires one after power up and
     * one after a MII communications error.  This means that we are
     * doing more preambles than we need, but it is safer and will be
     * much more robust.
     */

    for (i = 0; i < 33; i++)
        mdiobb_send_bit(1);

    /* send the start bit (01) and the read opcode (10) or write (10).
       Clause 45 operation uses 00 for the start and 11, 10 for
       read/write */
    mdiobb_send_bit(0);
    if (op & MDIO_C45)
        mdiobb_send_bit(0);
    else
        mdiobb_send_bit(1);
    mdiobb_send_bit((op >> 1) & 1);
    mdiobb_send_bit((op >> 0) & 1);

    mdiobb_send_num(PHY, 5);
    mdiobb_send_num(reg, 5);
}

/* In clause 45 mode all commands are prefixed by MDIO_ADDR to specify the
   lower 16 bits of the 21 bit address. This transfer is done identically to a
   MDIO_WRITE except for a different code. To enable clause 45 mode or
   MII_ADDR_C45 into the address. Theoretically clause 45 and normal devices
   can exist on the same bus. Normal devices should ignore the MDIO_ADDR
   phase. */
static int mdiobb_cmd_addr(int phy, u32 addr)
{
    unsigned int dev_addr = (addr >> 16) & 0x1F;
    unsigned int reg = addr & 0xFFFF;
    mdiobb_cmd(MDIO_C45_ADDR, phy, dev_addr);

    /* send the turnaround (10) */
    mdiobb_send_bit(1);
    mdiobb_send_bit(0);

    mdiobb_send_num(reg, 16);

    gpio_direction_input(GPIO_MDIO);

    mdiobb_get_bit();

    return dev_addr;
}

static int mdiobb_read(int phy, int reg)
{
    int ret, i;

    if (reg & MII_ADDR_C45) {
        reg = mdiobb_cmd_addr(phy, reg);
        mdiobb_cmd(MDIO_C45_READ, phy, reg);
    } else
        mdiobb_cmd(MDIO_READ, phy, reg);

    gpio_direction_input(GPIO_MDIO);

    if (mdiobb_get_bit() != 0) {
        for (i = 0; i < 32; i++)
            mdiobb_get_bit();

        return 0xbeef;
    }
    ret = mdiobb_get_num(16);
    mdiobb_get_bit();
    return ret;
}

static int mdiobb_write(int phy, int reg, u16 val)
{
    if (reg & MII_ADDR_C45) {
        reg = mdiobb_cmd_addr(phy, reg);
        mdiobb_cmd(MDIO_C45_WRITE, phy, reg);
    } else
        mdiobb_cmd(MDIO_WRITE, phy, reg);

    /* send the turnaround (10) */
    mdiobb_send_bit(1);
    mdiobb_send_bit(0);

    mdiobb_send_num(val, 16);

    gpio_direction_input(GPIO_MDIO);	

    mdiobb_get_bit();
    return 0;
}

/*-------------- SYSFS ---------------*/
struct mdio_data {
    int reg;
    int PHY;
    u8 c45;
    struct mutex lock;
};

static ssize_t show_c45(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct mdio_data *pdata = dev_get_drvdata(dev);
 
    mutex_lock(&pdata->lock);
    
    ret = sprintf(buf, "%d\n", pdata->c45);

    mutex_unlock(&pdata->lock);

    return ret;
}

static ssize_t set_c45(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct mdio_data *pdata = dev_get_drvdata(dev);
    u8 val;
    int error;

    mutex_lock(&pdata->lock);

    error = kstrtou8(buf, 10, &val);
    if (error)
        return error;

    pdata->c45 = val;

    mutex_unlock(&pdata->lock);

    return count;
}
static DEVICE_ATTR(c45, S_IWUSR | S_IRUGO, show_c45, set_c45);

static ssize_t show_PHY(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct mdio_data *pdata = dev_get_drvdata(dev);

    mutex_lock(&pdata->lock);

    ret = sprintf(buf, "%d\n", pdata->PHY);

    mutex_unlock(&pdata->lock);

    return ret;
}

static ssize_t set_PHY(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct mdio_data *pdata = dev_get_drvdata(dev);
    u16 val;
    int error;

    mutex_lock(&pdata->lock);

    error = kstrtou16(buf, 10, &val);
    if (error)
        return error;

    pdata->PHY = val;

    mutex_unlock(&pdata->lock);

    return count;	
}
static DEVICE_ATTR(PHY, S_IWUSR | S_IRUGO, show_PHY, set_PHY);

static ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct mdio_data *pdata = dev_get_drvdata(dev);

    mutex_lock(&pdata->lock);

    ret = sprintf(buf, "%d\n", pdata->reg);

    mutex_unlock(&pdata->lock);

    return ret;
}

static ssize_t set_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct mdio_data *pdata = dev_get_drvdata(dev);
    u32 val;
    int error;

    mutex_lock(&pdata->lock);

    error = kstrtou32(buf, 10, &val);
    if (error)
        return error;

    pdata->reg = val;
    if (pdata->c45)
        pdata->reg |= MII_ADDR_C45;

    mutex_unlock(&pdata->lock);

    return count;	
}
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, show_reg, set_reg);

static ssize_t show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct mdio_data *pdata = dev_get_drvdata(dev);
    
    mutex_lock(&pdata->lock);

    ret = sprintf(buf, "%d\n", mdiobb_read(pdata->PHY, pdata->reg));

    mutex_unlock(&pdata->lock);

    return ret;
}

static ssize_t set_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct mdio_data *pdata = dev_get_drvdata(dev);
    u16 val;
    int error;

    mutex_lock(&pdata->lock);

    error = kstrtou16(buf, 10, &val);
    if (error)
        return error;

    mdiobb_write(pdata->PHY, pdata->reg, val);

    mutex_unlock(&pdata->lock);

    return count;
}
static DEVICE_ATTR(data, S_IWUSR | S_IRUGO, show_data, set_data);

static const struct gpio mdio_gpios[] __initconst_or_module = {
    {
        .gpio   = GPIO_MDIO,
        .flags  = GPIOF_OUT_INIT_LOW,
        .label  = "gpio-mdio",
    },
    {
        .gpio   = GPIO_MDC,
        .flags  = GPIOF_OUT_INIT_LOW,
        .label  = "gpio-mdc",
    },
    {
        .gpio   = GPIO_RESERVED1,
        .label  = "Reserved-mdio",
    },
    {
        .gpio   = GPIO_RESERVED2,
        .label  = "Reserved-mdio",
    }
};

static ssize_t show_lock(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct mdio_data *pdata = dev_get_drvdata(dev);
    
    mutex_lock(&pdata->lock);

    if (gpio_request_array(mdio_gpios, ARRAY_SIZE(mdio_gpios)))
        ret = sprintf(buf, "%d\n", 0);
    else ret = sprintf(buf, "%d\n", 1);

    mutex_unlock(&pdata->lock);

    return ret;
}

static ssize_t free_lock(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct mdio_data *pdata = dev_get_drvdata(dev);

    mutex_lock(&pdata->lock);

    gpio_free_array(mdio_gpios, ARRAY_SIZE(mdio_gpios));

    mutex_unlock(&pdata->lock);

    return count;
}

static DEVICE_ATTR(lock, S_IWUSR | S_IRUGO, show_lock, free_lock);

static struct attribute *gpio_attributes[] = {
    &dev_attr_PHY.attr,
    &dev_attr_reg.attr,
    &dev_attr_data.attr,
    &dev_attr_c45.attr,
    &dev_attr_lock.attr,
    NULL
};

static const struct attribute_group gpio_group = {
    .attrs = gpio_attributes,
};

/*------- Module Setup/cleanup --------*/

static int __init mdio_bb_init(void)
{
    int err;
    struct mdio_data *pdata;
    printk("gpio-mdio init\n");

    //gpio_direction_output(GPIO_MDC, 0);
    platform_device = platform_device_register_simple("MDIO", 0, NULL, 0);	

    /*
    device_create_file(&platform_device->dev, &dev_attr_PHY);
    device_create_file(&platform_device->dev, &dev_attr_reg);
    device_create_file(&platform_device->dev, &dev_attr_data);
    device_create_file(&platform_device->dev, &dev_attr_c45);
    device_create_file(&platform_device->dev, &dev_attr_lock);
*/
    pdata = devm_kzalloc(&platform_device->dev, sizeof(struct mdio_data), GFP_KERNEL);
    if (!pdata)
        return -ENOMEM;

    mutex_init(&pdata->lock);

    dev_set_drvdata(&platform_device->dev, pdata);	
    
    err = sysfs_create_group(&platform_device->dev.kobj, &gpio_group);
    if (err < 0)
        return err;

    return 0;
}
module_init(mdio_bb_init);

static void __exit mdio_bb_exit(void)
{
    printk("gpio-mdio exit\n");

    platform_device_unregister(platform_device);	
}
module_exit(mdio_bb_exit);

MODULE_DESCRIPTION("MDIO GPIO Bitbanging Driver for RPi");
MODULE_LICENSE("GPL");
