#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <fsl_ssi.h>

#include <linux/gpio/consumer.h>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include "ccsi.h"

// max Minor devices
#define MAX_DEV 1

#define MAX_WRITE_DATA  30
#define FULL_FRAME 256 * 3 *2

//for some reason this isnt in fsl_ssi.h (everything else is)
#define SSI_SCR_CLK_IST	 0x00000200

struct gpio_desc *cs_line;

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

int send_packet(uint8_t *to_send, size_t size){
    uint8_t out[MAX_WRITE_DATA];
    size_t out_len;

    gpiod_set_value(cs_line, 0);

    out_len = output_len_calc(size);
    ccsi_packet_gen(to_send, size, out, out_len, 0);
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

    gpiod_set_value(cs_line, 1);

    writel(0xFFFF, g_ssi->base + REG_SSI_STX0); 

    return 0;
}

static int ccsidev_write(struct file *file, const char __user *u_buf, size_t size, loff_t *offset) {
    uint8_t frm_usr[FULL_FRAME], tmp_send[8];
    size_t out_len;

    int i = 0;
    if (g_ssi == NULL){
        return -EFAULT;
    }
    
    if (size > FULL_FRAME) {
        return -EFAULT;
    }
    
    if (copy_from_user_nofault(frm_usr, u_buf, size) == 0) {
        // printk("ccsidev: Copied %zd bytes from the user\n", size);
    } else {
        printk("ccsidev: usercopy failed!\n");
        return -EFAULT;
    }

    mutex_lock(&tx_lock);

    if (size <= 30){
        send_packet(frm_usr, size);
    } else {
        // printk("CCSI sending frame of %ld\n", size);
        //assume that its a while frame
        //TODO: fix magic numbers
        if(size == 256 * 3 *2){
            //this is a frame with no header bytes..add them
            tmp_send[0] = 0xAA;
            tmp_send[1] = 0x30;

            while (i + 6 <= size){
                memcpy(&tmp_send[2], &frm_usr[i], 6);
                send_packet(tmp_send, 8);
                i=i+6;
                udelay(20);
            }
            // printk("looped %d times size = %d\n", i, size);
        } else {
            printk("CCSI bad size, not sending = %ld\n", size);
        }
        
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

    cs_line = devm_gpiod_get(&pdev->dev, "cs", GPIOD_OUT_LOW);

	gpiod_set_value(cs_line, 1);

    ret = clk_prepare_enable(ssi->clk);
    if (ret)
        return ret;

    //static for use in write.
    g_ssi = ssi;
    printk("got ssi resrouce @ %p \n",g_ssi->base);

    // Configure SSI
    // SSI Control Register
    // Read out value for debug
    reg = readl(ssi->base + REG_SSI_SCR);
    printk("current control value = %x\n", reg);

    // Turn off the SSI initially
    writel(0x0, ssi->base + REG_SSI_SCR);  // Disable SSI

    //write values to register
    u32 scr_reg = (u32)( SSI_SCR_SSIEN | SSI_SCR_TE | SSI_SCR_TFR_CLK_DIS | SSI_SCR_CLK_IST | SSI_SCR_SYN );
    writel(scr_reg, ssi->base + REG_SSI_SCR);

    //re-read for debug
    reg = readl(ssi->base + REG_SSI_SCR);
    printk("SSI_SCR control = %x\n", reg);
   
    reg = readl(ssi->base + REG_SSI_STCR);
    printk("read SSI_STCR = %x\n", reg);


    // write the TCR
    u32 tcr = 0U;
    tcr |= SSI_STCR_TXBIT0; //LSB
    tcr |= SSI_STCR_TSCKP;
    // tcr &= ~(SSI_STCR_TSCKP); //data clocked on rising edge
    tcr |= SSI_STCR_TFDIR; //TFDIR fram sync is internal
    tcr |= SSI_STCR_TXDIR; //TXDIR to internal transmit clock
    tcr |= SSI_STCR_TFEN0; //fifo enable
    writel(tcr, ssi->base + REG_SSI_STCR); 

    reg = readl(ssi->base + REG_SSI_STCCR);
    printk("read SSI_STCCR [tx_clock] = %x\n", reg);

    //config the TCCR
    u32 tccr_reg = 0U;
    tccr_reg = 5; //random devider for now
    tccr_reg |= SSI_SxCCR_WL(16); //32bits wordlen (max)
    // reg |= SSI_SxCCR_DC(2);
    tccr_reg &= ~(SSI_SxCCR_DC_MASK); //disable frame sync
    // reg |= (0x3 << 13); //wordlen 8bits??
    printk("writing SSI_STCCR [tx_clock] = %x\n", reg);
    writel(tccr_reg, ssi->base + REG_SSI_STCCR); 

    printk("writing 0xA1 after control = %x\n", reg);
    writel(scr_reg, ssi->base + REG_SSI_SCR);  // enable SSI
   
    writel(0xFFFF, ssi->base + REG_SSI_STX0); //pull line up

    platform_set_drvdata(pdev, ssi);
    printk("CCSI Initialized\n");

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

    printk("CCSI Charachter driver created\n");

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