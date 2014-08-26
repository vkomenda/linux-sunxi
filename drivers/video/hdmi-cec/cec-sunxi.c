/*
 * CEC driver for Allwinner SunXi SoCs.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 *
 * This driver registers the SinXi CEC hardware register with the hdmi-cec
 * driver. The hardware is very simple. Therefore basic CEC functions have to be
 * performed by this driver.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include "hdmi-cec.h"

#define SUNXI_CEC_REG (0xf1c16214) /* Everything: ISR, RX and TX. */

/* CEC register fields */
#define CEC_RX     0x0100  /* phys. line value, either high or low */
#define CEC_TX     0x0200
#define CEC_ENABLE 0x0800

#define setl(bits, addr)   writel(readl(addr) | bits, addr)
#define clearl(bits, addr) writel(readl(addr) & ~bits, addr)

struct cec_priv {
	void __iomem*		reg;
	struct platform_device*	pdev;
	int			irq;
	struct cec_driver	cec_drv;
	struct completion	tx_complete;
	spinlock_t		lock;
	u8                      initiator;
	u8                      sendFailed;
};

/* Enable software control. */
static coid cecSoftwareEnable (struct cec_priv* priv)
{
	setl   (CEC_ENABLE, priv->reg);
}

/* Disable software control. */
static void cecSoftwareDisable (struct cec_priv* priv)
{
	clearl (CEC_ENABLE, priv->reg);
}

static void cecPullLow (struct cec_priv* priv)
{
	clearl (CEC_TX, priv->reg);
}

static void cecPullHigh (struct cec_priv* priv)
{
	setl   (CEC_TX, priv->reg);
}

static u8 cecCheckLine (struct cec_priv* priv)
{
	return (u8) ((readl (priv->reg) >> 8) & 1);
}

/*
static void cec_write(struct cec_priv* priv, u8 bit)
{
	if (bit)
		setl   (CEC_TX, priv->reg);
	else
		clearl (CEC_TX, priv->reg);
}
*/

static void cecSendStart(struct cec_priv* priv)
{
	cecPullLow  (priv);
	udelay      (CEC_START_LOW);
	cecPullHigh (priv);
	udelay      (CEC_START_HIGH);
}

static void cecSend0(struct cec_priv* priv)
{
	cecPullLow  (priv);
	udelay      (CEC_0_LOW);
	cecPullHigh (priv);
	udelay      (CEC_0_HIGH);
}

static void cecSend1(struct cec_priv* priv)
{
	cecPullLow  (priv);
	udelay      (CEC_1_LOW);
	cecPullHigh (priv);
	udelay      (CEC_1_HIGH);
}

/*
 * Request an acknowledgement from the recepient. Returns 0 iff the
 * acknowledgement is not received.
 */
static u8 cecRequestAck (struct cec_priv* priv)
{
	u8 ackReceived = 0;
	/* Assert bit 1. */
	cecPullLow  (priv);
	udelay      (CEC_1_LOW);
	cecPullHigh (priv);
	/* Acknowledgement follows from the line remaining low within a safe
	 * sampling period. */
	udelay      (CEC_ACK_SAMPLE);
	if (cecCheckLine (priv) == 0)
		ackReceived = 1;
	/* Synchronisation delay. */
        udelay      (CEC_ACK_REST);
	return ackReceived;
}

#define cecSendAck cecSend0
/*
static void cecSendAck(struct cec_priv* priv)
{
	while (cecCheckLine (priv))
		cpu_relax ();
	cecPull...
}
*/

static u8 cecSendByte (struct cec_priv* priv, u8 payload, u8 eom)
{
	u8 i;
	/* 8 data payload bits */
	for (i = 0; i < 8; i++) {
		if (payload & 0x80)
			cecSend1 (priv);
		else
			cecSend0 (priv);
		payload = payload << 1;
	}
	/* End Of Message bit */
	if (eom)
		cecSend1 (priv);
	else
		cecSend0 (priv);
	/* Byte delivery acknowledgement */
	return cecRequestAck (priv);
}

/* Arbitration draft. TODO: Extend arbittration into the start pulse until the
 * end of the initiator address in the header frame. */
static u8 cecArbitration (struct cec_priv* priv)
{
	u8 lineFree = 1;
	u32 freeTime = 0, elapsed = 0;
	if (priv->initiator) {
		if (priv->sendFailed)
			freeTime = CEC_SEND_FAILED;
		else
			freeTime = CEC_NEXT_FRAME;
	}
	else {
		freeTime = CEC_NEW_INITIATOR;
	}
	while (elapsed <= freeTime) {
		if (cecCheckLine (priv) == 0) {
			lineFree = 0;
			break;
		}
		udelay (SAMPLE_PERIOD);
		elapsed = elapsed + SAMPLE_PERIOD;
	}
	if (!lineFree) {
		priv->initiator = 0;
	}
	return lineFree;
}

/* Expects the start pulse. Returns 0 upon the end of the pulse or a status code
 * if an error occurred.*/
static int cecExpectStart (struct cec_priv* priv)
{
	int status = 0;
	/* The number of periods to sample when high exceeds the max. time in
	 * the specification to detect the presence of an error. */
	u32 highPeriodsLeft = CEC_START_HIGH_MAX / FAST_SAMPLE_PERIOD + 1;
	while (cecCheckLine (priv) == 1)
		msleep (INTERFRAME_SAMPLE_PERIOD);
	while (cecCheckLine (priv) == 0)
		cpu_relax ();
	while (cecCheckLine (priv) == 1 && highPeriodsLeft != 0) {
		udelay (FAST_SAMPLE_PERIOD);
		highPeriodsLeft--;
	}
	if (!highPeriodsLeft)
		status = -EMSGSIZE;
	return status;
}

static u8 cecReceiveBit (struct cec_priv* priv)
{
	u8 result;

	while (cecCheckLine (priv) == 1) {
		udelay (SAMPLE_PERIOD);
	}
	while (cecCheckLine (priv) == 0) {
		udelay (SAMPLE_PERIOD);
		elapsed = elapsed + SAMPLE_PERIOD;
	}
	if (elapsed <= CEC_1_LOW_MAX)       /* Received 1. */
		result = 1;
	else if (elapsed <= CEC_0_LOW_MAX)  /* Received 0. */
		result = 0;
	else                                /* An error occurred. */
		result = 0xff;
	return result;
}

static irqreturn_t cec_isr(int irq, void *dev_id)
{
	struct cec_priv *priv = dev_id;
	u8 cause;
	u8 msg[CEC_MAX_MSG_LEN], count;

	/* read cause and mask all other causes */
	cause = ioread8(priv->reg);
	iowrite8(0, priv->regs + CEC_EXAMPLE_IM_REG);

	if (cause & CEC_EXAMPLE_ISR_TX)
		complete(&priv->tx_complete);

	/* we should probably use a bottom-half here, but there is not
	 * so much work to do */
	if (cause & CEC_EXAMPLE_ISR_RX) {
		count = cause >> CEC_EXAMPLE_ISR_MSG_CNT;
		ioread8_rep(priv->regs + CEC_EXAMPLE_FIFO_REG,
				msg, count);
		cec_receive_message(&priv->cec_drv, msg, count);
	}

	return IRQ_HANDLED;
}

static int cec_example_set_addr(struct cec_driver *drv, const u8 addr)
{
	struct cec_example_priv *priv =
		container_of(drv, struct cec_example_priv, cec_drv);

	iowrite8(addr, priv->regs + CEC_EXAMPLE_ADDR_REG);

	return 0;
}

static int
cec_example_send(struct cec_driver *drv, const u8 *data, const u8 len)
{
	struct cec_example_priv *priv =
		container_of(drv, struct cec_example_priv, cec_drv);
	unsigned int timeout;
	u8 reg;
	unsigned long flags;

	init_completion(&priv->tx_complete);

	spin_lock_irqsave(&priv->lock, flags);
	/* enable TX completion interrupt */
	reg = ioread8(priv->regs + CEC_EXAMPLE_ISR_REG);
	reg |=  CEC_EXAMPLE_ISR_TX;
	iowrite8(reg, priv->regs + CEC_EXAMPLE_ISR_REG);

	/* write to the hardware fifo */
	iowrite8_rep(priv->regs + CEC_EXAMPLE_FIFO_REG,
			data, len);

	spin_unlock_irqrestore(&priv->lock, flags);

	/* ISR will complete our tx completion */
	timeout = wait_for_completion_interruptible_timeout(
				&priv->tx_complete, HZ / 2);

	return !timeout ? -ETIMEDOUT : 0;
}

static int cec_example_reset(struct cec_driver *drv)
{
	struct cec_example_priv *priv =
		container_of(drv, struct cec_example_priv, cec_drv);

	/* clear all registers */
	iowrite8_rep(priv->regs, 0, 6 + CEC_MAX_MSG_LEN);

	return 0;
}

static struct cec_driver_ops cec_example_ops = {
	.set_logical_address	= cec_example_set_addr,
	.send			= cec_example_send,
	.reset			= cec_example_reset,
};

static int cec_probe(struct platform_device *pdev)
{
	struct cec_priv *priv;
	struct resource *r;
	int irq;
	int ret;

	r = platform_get_resource(pdev, 0, IORESOURCE_MEM);
	if (!r)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENODEV;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regs = ioremap(r->start, resource_size(r));
	if (!priv->regs) {
		ret = -ENOMEM;
		goto out;
	}

	spin_lock_init(&priv->lock);

	platform_set_drvdata(pdev, priv);
	priv->irq = irq;
	priv->pdev = pdev;
	priv->cec_drv.ops = &cec_example_ops;

	ret = register_cec_driver(&priv->cec_drv);
	if (ret)
		goto out_iomem;

	ret = request_irq(irq, cec_example_isr, 0, pdev->name, priv);
	if (ret)
		goto out_drv;

	/* enable CEC RX completion (TX enabled later) */
	iowrite8(CEC_EXAMPLE_ISR_RX, priv->regs + CEC_EXAMPLE_ISR_REG);

	dev_info(&pdev->dev, "CEC SunXi driver registered");

	return 0;

out_drv:
	unregister_cec_driver(&priv->cec_drv);
out_iomem:
	iounmap(priv->regs);
out:
	kfree(priv);
	return ret;
}

static int cec_example_remove(struct platform_device *pdev)
{
	struct cec_priv *priv = platform_get_drvdata(pdev);

	/* disable all interrupts */
	iowrite8(0, priv->regs + CEC_EXAMPLE_ISR_REG);
	free_irq(priv->irq, priv);
	unregister_cec_driver(&priv->cec_drv);
	iounmap(priv->regs);
	kfree(priv);

	return 0;
}

static struct platform_driver cec_driver = {
	.driver	= {
		.name	= "cec-example",
		.owner	= THIS_MODULE,
	},
	.probe	= cec_probe,
	.remove	= __devexit_p(cec_remove),
};

static int __init cec_init(void)
{
	return platform_driver_register(&cec_driver);
}

static void __exit cec_exit(void)
{
	platform_driver_unregister(&cec_driver);
}

module_init(cec_example_init);
module_exit(cec_example_exit);

MODULE_AUTHOR("Hutchison Technologies");
MODULE_DESCRIPTION("HDMI-CEC driver for SunXi");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cec-sunxi");
