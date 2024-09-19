#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <fsl_ssi.h>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include "ccsi.h"

// max Minor devices
#define MAX_DEV 1

#define MAX_WRITE_DATA  30

//for some reason this isnt in fsl_ssi.h (everything else is)
#define SSI_SCR_CLK_IST	 0x00000200


struct imx_ssi_dev {
    void __iomem *base;
    struct clk *clk;
};

static struct imx_ssi_dev *g_ssi;

//mutex for the ssi/ccsi port
DEFINE_MUTEX(port_mutex);

DEFINE_MUTEX(tx_lock);

// device data holder, this structure may be extended to hold additional data
struct mychar_device_data {
    struct cdev cdev;
};

// global storage for device Major number
static int dev_major = 0;

// sysfs class structure
static struct class *ccsidev_class = NULL;

// array of mychar_device_data for
static struct mychar_device_data ccsidev_data[MAX_DEV];

static int ccsidev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

static int ccsidev_open(struct inode *inode, struct file *file) {
    if (mutex_trylock(&port_mutex) == 1){
        printk("ccsidev: CCSI Device opened\n");
        return 0;
    } else {
        printk("ccsidev: FAILED to open ccsi. lock contention.\n");
    }
    return -1;
}

static int ccsidev_release(struct inode *inode, struct file *file) {
    mutex_unlock(&port_mutex);
    printk("ccsidev: Device closed\n");
    return 0;
}

static long ccsidev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    printk("ccsidev: Device ioctl\n");
    return 0;
}

static ssize_t ccsidev_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    printk("ccsidev: Device read\n");
    return 0;
}

static int ccsidev_write(struct file *file, const char __user *u_buf, size_t size, loff_t *offset) {
    uint8_t frm_usr[MAX_WRITE_DATA], out[MAX_WRITE_DATA];
    size_t out_len;
    if (g_ssi == NULL){
        return -EFAULT;
    }

    printk("ccsidev: Write got ssi resrouce @ %p \n",g_ssi->base);
    
    if (size > MAX_WRITE_DATA) {
        return -EFAULT;
    }
    

    if (copy_from_user_nofault(frm_usr, u_buf, size) == 0) {
        printk("ccsidev: Copied %zd bytes from the user\n", size);
    } else {
        printk("ccsidev: usercopy failed!\n");
        return -EFAULT;
    }

    out_len = output_len_calc(size);
    // uint8_t out[out_len];
    ccsi_packet_gen(frm_usr, size, out, out_len, 0);
    mutex_lock(&tx_lock);
    ///zero out the bus and transition from start
    writel(0xFFFF, g_ssi->base + REG_SSI_STX0); //pull high
    udelay(10);
    writel(0xFFFE, g_ssi->base + REG_SSI_STX0); //trasnition low one CC before data

    int i = 0;
    while(i < out_len){
        if (i+1 < out_len){
            writel( (uint16_t)(out[i] << 8) | out[i+1], g_ssi->base + REG_SSI_STX0);
        } else {
            writel( (uint16_t)(out[i] << 8) | 0x00FF, g_ssi->base + REG_SSI_STX0); //no data is high so padd with 0xFF
        }
        i=i+2;
    }

    //leave 0xFFFF in the TX reg to hold the line high
    udelay(100);
    writel(0xFFFF, g_ssi->base + REG_SSI_STX0); 

    out[size] = 0;
    printk("Writing %d bytes; Header from the user: 0x%02X 0x%02X\n", out_len, out[0], out[1]);
    if(size > 2){
        printk("First Data = 0x%02X\n", out[2]);
    }

    mutex_unlock(&tx_lock);
    //every 16bit word needs a check bit which is the not of bit 16 MSb

    return size;
}


// initialize file_operations
static const struct file_operations ccsidev_fops = {
    .owner      = THIS_MODULE,
    .open       = ccsidev_open,
    .release    = ccsidev_release,
    .unlocked_ioctl = ccsidev_ioctl,
    .read       = ccsidev_read,
    .write       = ccsidev_write
};


static int imx_ssi_probe(struct platform_device *pdev)
{
    u32 reg;
    printk("starting imx-ssi\n");
    struct imx_ssi_dev *ssi;
    struct resource *res;
    int ret;

    ssi = devm_kzalloc(&pdev->dev, sizeof(*ssi), GFP_KERNEL);
    if (!ssi)
        return -ENOMEM;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    ssi->base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(ssi->base))
        return PTR_ERR(ssi->base);

    ssi->clk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(ssi->clk))
        return PTR_ERR(ssi->clk);

    ret = clk_prepare_enable(ssi->clk);
    if (ret)
        return ret;

    //static for use in write.
    g_ssi = ssi;
    printk("got ssi resrouce @ %p \n",g_ssi->base);

    // Configure SSI
    reg = readl(ssi->base + REG_SSI_SCR);
    printk("current control value = %x\n", reg);
    writel(0x0, ssi->base + REG_SSI_SCR);  // Disable SSI
    u32 send;
    send = SSI_SCR_SSIEN;
    send |= SSI_SCR_TE;
    send |= SSI_SCR_TFR_CLK_DIS;
    send |= SSI_SCR_CLK_IST; //clock idle high
    // send &= ~(SSI_SCR_CLK_IST); //clock idle low => this seems required for rising edge trigger
    send |= SSI_SCR_SYN;
    writel(send, ssi->base + REG_SSI_SCR);  // Enable SSI bit0 and TX bit 1

    reg = readl(ssi->base + REG_SSI_SCR);
    printk("SSI_SCR control = %x\n", reg);
   
    reg = readl(ssi->base + REG_SSI_STCR);
    printk("read SSI_STCR = %x\n", reg);

    u32 tcr = 0x0U;
    tcr |= SSI_STCR_TXBIT0; //LSB
    tcr |= SSI_STCR_TSCKP;
    // tcr &= ~(SSI_STCR_TSCKP); //data clocked on rising edge
    tcr |= SSI_STCR_TFDIR; //TFDIR fram sync is internal
    tcr |= SSI_STCR_TXDIR; //TXDIR to internal transmit clock
    tcr |= SSI_STCR_TFEN0; //fifo enable
    writel(tcr, ssi->base + REG_SSI_STCR); 

    reg = readl(ssi->base + REG_SSI_STCCR);
    printk("read SSI_STCCR [tx_clock] = %x\n", reg);
    reg = 40; //random devider for now
    reg |= SSI_SxCCR_WL(16); //32bits wordlen (max)
    // reg |= SSI_SxCCR_DC(2);
    reg &= ~(SSI_SxCCR_DC_MASK); //disable frame sync
    // reg |= (0x3 << 13); //wordlen 8bits??
    printk("writing SSI_STCCR [tx_clock] = %x\n", reg);
    writel(reg, ssi->base + REG_SSI_STCCR); 

    printk("writing 0xA1 after control = %x\n", reg);
    // writel(0x03, ssi->base + REG_SSI_SCR);  // enable SSI
    writel(send, ssi->base + REG_SSI_SCR);  // enable SSI


    writel(0xFFFF, ssi->base + REG_SSI_STX0); 
    // msleep(1);
    udelay(100);
    writel(0xFFFE, ssi->base + REG_SSI_STX0); 


    uint8_t cmd[] = {0xAA, 0x10}; //chip id

    size_t out_len = output_len_calc(sizeof(cmd));
    uint8_t out[out_len];
    ccsi_packet_gen(cmd, sizeof(cmd), out, out_len, 0);
    printk("writing chipid - %d bytes\n", out_len);

    int i = 0;
    while(i < out_len){
        if (i+1 < out_len){
            writel( (uint16_t)(out[i] << 8) | out[i+1], ssi->base + REG_SSI_STX0);
        } else {
            writel( (uint16_t)(out[i] << 8) & 0xFFFF, ssi->base + REG_SSI_STX0);
        }
        i=i+2;
    }
    writel(0xFFFF, ssi->base + REG_SSI_STX0); //send something

    // msleep(1);
    udelay(100);

    writel(0xFFFE, ssi->base + REG_SSI_STX0); //send something
   
   
    cmd[1] = 0x70; // read chip id
    ccsi_packet_gen(cmd, sizeof(cmd), out, out_len, 0);
    printk("read chip id - %d bytes\n", out_len);
 
    i = 0;
    while(i < out_len){
        if (i+1 < out_len){
            writel( (uint16_t)(out[i] << 8) | out[i+1], ssi->base + REG_SSI_STX0);
        } else {
            writel( (uint16_t)(out[i] << 8) & 0xFFFF, ssi->base + REG_SSI_STX0);
        }
        i=i+2;
    }

    writel(0xFFFF, ssi->base + REG_SSI_STX0); //send something

    // writel(0x03, ssi->base + REG_SSI_SCR);  // enable SSI
    platform_set_drvdata(pdev, ssi);
    printk("done\n");

    //char dev

    int err, idx;
    dev_t dev;

    err = alloc_chrdev_region(&dev, 0, MAX_DEV, "ccsidev");

    dev_major = MAJOR(dev);

    ccsidev_class = class_create(THIS_MODULE, "ccsidev");
    ccsidev_class->dev_uevent = ccsidev_uevent;

    for (idx = 0; idx < MAX_DEV; idx++) {
        cdev_init(&ccsidev_data[idx].cdev, &ccsidev_fops);
        ccsidev_data[idx].cdev.owner = THIS_MODULE;

        cdev_add(&ccsidev_data[idx].cdev, MKDEV(dev_major, idx), 1);

        device_create(ccsidev_class, NULL, MKDEV(dev_major, idx), NULL, "ccsidev-%d", idx);
    }

    printk("char_done\n");

    return 0;
}

static int imx_ssi_remove(struct platform_device *pdev)
{
    struct imx_ssi_dev *ssi = platform_get_drvdata(pdev);
    int i;
    // writel(0x03, ssi->base + REG_SSI_SCR);  // enable SSI
    clk_disable_unprepare(ssi->clk);

    for (i = 0; i < MAX_DEV; i++) {
        device_destroy(ccsidev_class, MKDEV(dev_major, i));
    }

    class_unregister(ccsidev_class);
    class_destroy(ccsidev_class);

    unregister_chrdev_region(MKDEV(dev_major, 0), MINORMASK);


    return 0;
}

static const struct of_device_id imx_ssi_dt_ids[] = {
    { .compatible = "fsl,imx6q-ssi", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_ssi_dt_ids);

static struct platform_driver imx_ssi_driver = {
    .probe = imx_ssi_probe,
    .remove = imx_ssi_remove,
    .driver = {
        .name = "imx-ssi",
        .of_match_table = imx_ssi_dt_ids,
    },
};


module_platform_driver(imx_ssi_driver);

MODULE_AUTHOR("Galen Church");
MODULE_DESCRIPTION("i.MX6 SSI->CCSI Driver");
MODULE_LICENSE("GPL");