/*
 *	Driver for Allwinner A20 PS2 host controller
 *
 *	Author: Vishnu Patekar <vishnupatekar0510@gmail.com>
 *		Aaron.maoye <leafy.myeh@newbietech.com>
 *
 *		Based on 3.0 kernel
 *
 */

#include <linux/module.h>
#include <linux/serio.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/delay.h>

#define DRIVER_NAME		"sunxi-ps2"

/* register offset definitions */
#define PS2_REG_GCTL		(0x00)	/*  PS2 Module Global Control Reg */
#define PS2_REG_DATA		(0x04)  /*  PS2 Module Data Reg		*/
#define PS2_REG_LCTL		(0x08)	/*  PS2 Module Line Control Reg */
#define PS2_REG_LSTS		(0x0C)  /*  PS2 Module Line Status Reg	*/
#define PS2_REG_FCTL		(0x10)	/*  PS2 Module FIFO Control Reg */
#define PS2_REG_FSTS		(0x14)	/*  PS2 Module FIFO Status Reg	*/
#define PS2_REG_CLKDR		(0x18)	/*  PS2 Module Clock Divider Reg*/

/*  PS2 GLOBAL CONTROL REGISTER PS2_GCTL */
#define PS2_GCTL_INTFLAG	BIT(4)
#define PS2_GCTL_INTEN		BIT(3)
#define PS2_GCTL_RESET		BIT(2)
#define PS2_GCTL_MASTER		BIT(1)
#define PS2_GCTL_BUSEN		BIT(0)

/* PS2 LINE CONTROL REGISTER */
#define PS2_LCTL_NOACK		BIT(18)
#define PS2_LCTL_TXDTOEN	BIT(8)
#define PS2_LCTL_STOPERREN	BIT(3)
#define PS2_LCTL_ACKERREN	BIT(2)
#define PS2_LCTL_PARERREN	BIT(1)
#define PS2_LCTL_RXDTOEN	BIT(0)

/* PS2 LINE STATUS REGISTER */
#define PS2_LSTS_TXTDO		BIT(8)
#define PS2_LSTS_STOPERR	BIT(3)
#define PS2_LSTS_ACKERR		BIT(2)
#define PS2_LSTS_PARERR		BIT(1)
#define PS2_LSTS_RXTDO		BIT(0)

/* PS2 FIFO CONTROL REGISTER */
#define PS2_FCTL_TXRST		BIT(17)
#define PS2_FCTL_RXRST		BIT(16)
#define PS2_FCTL_TXUFIEN	BIT(10)
#define PS2_FCTL_TXOFIEN	BIT(9)
#define PS2_FCTL_TXRDYIEN	BIT(8)
#define PS2_FCTL_RXUFIEN	BIT(2)
#define PS2_FCTL_RXOFIEN	BIT(1)
#define PS2_FCTL_RXRDYIEN	BIT(0)

/* PS2 FIFO STATUS REGISTER */
#define PS2_FSTS_TXUF		BIT(10)
#define PS2_FSTS_TXOF		BIT(9)
#define PS2_FSTS_TXRDY		BIT(8)
#define PS2_FSTS_RXUF		BIT(2)
#define PS2_FSTS_RXOF		BIT(1)
#define PS2_FSTS_RXRDY		BIT(0)


#define PS2_LINE_ERROR_BIT \
	(PS2_LSTS_TXTDO|PS2_LSTS_STOPERR|PS2_LSTS_ACKERR| \
	PS2_LSTS_PARERR|PS2_LSTS_RXTDO)

#define PS2_FIFO_ERROR_BIT \
	(PS2_FSTS_TXUF|PS2_FSTS_TXOF|PS2_FSTS_TXRDY|PS2_FSTS_RXUF| \
	PS2_FSTS_RXOF|PS2_FSTS_RXRDY)

#define PS2_SAMPLE_CLK		(1000000)
#define PS2_SCLK		(125000)

struct sunxips2data {
	struct serio *serio;
	struct device *dev;

	/* IO mapping base */
	void __iomem	*reg_base;

	/* clock management */
	struct clk	*clk;

	/* irq */
	spinlock_t	lock;
	int		irq;
};

/*********************/
/* Interrupt handler */
/*********************/
static irqreturn_t sunxips2_interrupt(int irq, void *dev_id)
{
	struct sunxips2data *drvdata = dev_id;
	u32 intr_status;
	u32 fifo_status;
	unsigned char byte;
	u32 rval;
	u32 error = 0;

	spin_lock(&drvdata->lock);

	/* Get the PS/2 interrupts and clear them */
	intr_status  = readl(drvdata->reg_base + PS2_REG_LSTS);
	fifo_status  = readl(drvdata->reg_base + PS2_REG_FSTS);

	/*Check Line Status Register*/
	if (intr_status & 0x10f) {
		if (intr_status & PS2_LSTS_STOPERR)
			dev_info(drvdata->dev, "PS/2 Stop Bit Error!");
		if (intr_status & PS2_LSTS_ACKERR)
			dev_info(drvdata->dev, "PS/2 Acknowledge Error!\n");
		if (intr_status & PS2_LSTS_PARERR)
			dev_info(drvdata->dev, "PS/2 Parity Error!\n");
		if (intr_status & PS2_LSTS_TXTDO)
			dev_info(drvdata->dev, "PS/2 Transmit Data Timeout!\n");
		if (intr_status & PS2_LSTS_RXTDO)
			dev_info(drvdata->dev, "PS/2 Receive Data Timeout!\n");

		/*reset PS/2 controller*/
		writel(readl(drvdata->reg_base + PS2_REG_GCTL) | PS2_GCTL_RESET,
			drvdata->reg_base + PS2_REG_GCTL);

		rval = PS2_LSTS_TXTDO | PS2_LSTS_STOPERR | PS2_LSTS_ACKERR |
			PS2_LSTS_PARERR | PS2_LSTS_RXTDO;
		writel(rval, drvdata->reg_base + PS2_REG_LSTS);
		error = 1;
	}

	/*Check FIFO Status Register*/
	if (fifo_status & 0x0606) {
		if (fifo_status & PS2_FSTS_TXUF)
			dev_info(drvdata->dev, "PS/2 Tx FIFO Underflow!\n");
		if (fifo_status & PS2_FSTS_TXOF)
			dev_info(drvdata->dev, "PS/2 Tx FIFO Overflow!\n");
		if (fifo_status & PS2_FSTS_RXUF)
			dev_info(drvdata->dev, "PS/2 Rx FIFO Underflow!\n");
		if (fifo_status & PS2_FSTS_RXOF)
			dev_info(drvdata->dev, "PS/2 Rx FIFO Overflow!\n");
		/*reset PS/2 controller*/
		writel(readl(drvdata->reg_base + PS2_REG_GCTL) | PS2_GCTL_RESET,
			drvdata->reg_base + PS2_REG_GCTL);

		rval = PS2_FSTS_TXUF | PS2_FSTS_TXOF | PS2_FSTS_TXRDY |
			PS2_FSTS_RXUF | PS2_FSTS_RXOF | PS2_FSTS_RXRDY;
		writel(rval, drvdata->reg_base + PS2_REG_FSTS);
		error = 1;
	}

	rval = (fifo_status >> 16) & 0x3;
	while (!error && rval--) {
		byte = readl(drvdata->reg_base + PS2_REG_DATA) & 0xff;
		/* dev_info(drvdata->dev, "PS/2 Receive %02x\n", byte); */
		serio_interrupt(drvdata->serio, byte, 0);
	}

	writel(intr_status, drvdata->reg_base + PS2_REG_LSTS);
	writel(fifo_status, drvdata->reg_base + PS2_REG_FSTS);

	spin_unlock(&drvdata->lock);

	return IRQ_HANDLED;
}


static int sunxips2_open(struct serio *pserio)
{
	struct sunxips2data *drvdata = pserio->port_data;
	u32 src_clk = 0;
	u32 clk_scdf;
	u32 clk_pcdf;
	u32 rval;

	/*Set Line Control And Enable Interrupt*/
	rval = PS2_LCTL_TXDTOEN | PS2_LCTL_STOPERREN | PS2_LCTL_ACKERREN
		| PS2_LCTL_PARERREN | PS2_LCTL_RXDTOEN;
	writel(rval, drvdata->reg_base + PS2_REG_LCTL);

	/*Reset FIFO*/
	rval = PS2_FCTL_TXRST|PS2_FCTL_RXRST|PS2_FCTL_TXUFIEN|PS2_FCTL_TXOFIEN
		|PS2_FCTL_RXUFIEN|PS2_FCTL_RXOFIEN|PS2_FCTL_RXRDYIEN;
	writel(rval, drvdata->reg_base + PS2_REG_FCTL);

	src_clk = clk_get_rate(drvdata->clk);

	if (!src_clk) {
		dev_info(drvdata->dev, "w_ps2c_set_sclk error, source clock is 0.");
		return -1;
	}

	/*Set Clock Divider Register*/
	clk_scdf = DIV_ROUND_UP(src_clk, PS2_SAMPLE_CLK) - 1;
	clk_pcdf = DIV_ROUND_UP(PS2_SAMPLE_CLK, PS2_SCLK) - 1;
	rval = (clk_scdf<<8) | clk_pcdf;
	writel(rval, drvdata->reg_base + PS2_REG_CLKDR);

	/*Set Global Control Register*/
	rval = PS2_GCTL_RESET | PS2_GCTL_INTEN | PS2_GCTL_MASTER
		| PS2_GCTL_BUSEN;
	writel(rval, drvdata->reg_base + PS2_REG_GCTL);

	return 0;
}

static void sunxips2_close(struct serio *pserio)
{
	struct sunxips2data *drvdata = pserio->port_data;
	unsigned long flags;

	spin_lock_irqsave(&drvdata->lock, flags);
	/* Disable the PS2 interrupts */
	writel(0, drvdata->reg_base + PS2_REG_GCTL);
	spin_unlock_irqrestore(&drvdata->lock, flags);
}

static int sunxips2_write(struct serio *pserio, unsigned char val)
{
	unsigned long expire = jiffies + msecs_to_jiffies(10000);
	struct sunxips2data *drvdata = (struct sunxips2data *)pserio->port_data;

	do {
		if (readl(drvdata->reg_base + PS2_REG_FSTS) & PS2_FSTS_TXRDY) {
			writel(val, drvdata->reg_base + PS2_REG_DATA);
			return 0;
		}
	} while (time_before(jiffies, expire));

	return 0;
}

static int sunxips2_probe(struct platform_device *pdev)
{
	struct resource *res; /* IO mem resources */
	struct sunxips2data *drvdata;
	struct serio *serio;
	struct device *dev = &pdev->dev;
	unsigned int irq;
	int error;

	drvdata = devm_kzalloc(dev, sizeof(struct sunxips2data), GFP_KERNEL);
	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!drvdata || !serio) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	spin_lock_init(&drvdata->lock);

	/* IO */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(drvdata->reg_base)) {
		dev_err(dev, "failed to map registers\n");
		error = PTR_ERR(drvdata->reg_base);
		goto err_free_mem;
	}

	drvdata->clk = devm_clk_get(dev, NULL);
	if (!IS_ERR(drvdata->clk)) {
		error = clk_prepare_enable(drvdata->clk);
		if (error < 0) {
			dev_err(dev, "failed to enable clock %d\n", error);
			goto err_free_mem;
		}
	} else {
		error = PTR_ERR(drvdata->clk);
		dev_err(dev, "couldn't get clock %d\n", error);
		goto err_free_mem;
	}

	serio->id.type = SERIO_8042;
	serio->write = sunxips2_write;
	serio->open = sunxips2_open;
	serio->close = sunxips2_close;
	serio->port_data = drvdata;
	serio->dev.parent = dev;
	strlcpy(serio->name, dev_name(dev), sizeof(serio->name));
	strlcpy(serio->phys, dev_name(dev), sizeof(serio->phys));

	/* Get IRQ for the device */
	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(dev, "no IRQ found\n");
		error = -ENXIO;
		goto error_disable_clk;
	}

	drvdata->irq = irq;
	drvdata->serio = serio;
	drvdata->dev = dev;
	error = devm_request_threaded_irq(dev, drvdata->irq,
		sunxips2_interrupt, NULL, 0, DRIVER_NAME, drvdata);

	if (error) {
		dev_err(drvdata->dev, "Interrupt alloc failed %d:error:%d\n",
			drvdata->irq, error);
		goto error_disable_clk;
	}

	serio_register_port(serio);
	platform_set_drvdata(pdev, drvdata);

	return 0;	/* success */

error_disable_clk:
	clk_disable_unprepare(drvdata->clk);

err_free_mem:
	kfree(serio);
	return error;
}

static int sunxips2_remove(struct platform_device *pdev)
{
	struct sunxips2data *drvdata = platform_get_drvdata(pdev);

	serio_unregister_port(drvdata->serio);
	disable_irq(drvdata->irq);

	if (!IS_ERR(drvdata->clk))
		clk_disable_unprepare(drvdata->clk);
	kfree(drvdata->serio);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id sunxips2_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-ps2", },
	{ },
};

MODULE_DEVICE_TABLE(of, sunxips2_of_match);

/*platform driver structure*/
static struct platform_driver sunxips2_of_driver = {
	.probe		= sunxips2_probe,
	.remove		= sunxips2_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = sunxips2_of_match,
	},
};
module_platform_driver(sunxips2_of_driver);

MODULE_AUTHOR("Vishnu Patekar <vishnupatekar0510@gmail.com>");
MODULE_AUTHOR("Aaron.maoye <leafy.myeh@newbietech.com>");
MODULE_DESCRIPTION("Sunxi PS/2 driver");
MODULE_LICENSE("GPL");
