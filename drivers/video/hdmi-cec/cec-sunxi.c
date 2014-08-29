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

#define set32(bits, addr)   writel(readl(addr) | bits, addr)
#define clear32(bits, addr) writel(readl(addr) & ~bits, addr)

struct cec_priv {
	void __iomem*		reg;
	struct platform_device*	pdev;
	int			irq;
	u8                      irqCause;
	u8                      addr;
//	CecLAddr                laddr;
	struct cec_driver	cec_drv;
	struct completion	tx_complete;
	spinlock_t		lock;
	CecMode                 mode;
	u8                      sendFailed;
};

/* Enable software control. */
static coid cecSoftwareEnable (struct cec_priv* const priv)
{
	set32   (CEC_ENABLE, priv->reg);
}

/* Disable software control. */
static void cecSoftwareDisable (struct cec_priv* const priv)
{
	clear32 (CEC_ENABLE, priv->reg);
}

static void cecPullLow (struct cec_priv* const priv)
{
	clear32 (CEC_TX, priv->reg);
}

static void cecPullHigh (struct cec_priv* const priv)
{
	set32   (CEC_TX, priv->reg);
}

static u8 cecCheckLine (struct cec_priv* const priv)
{
	return (readl (priv->reg) & CEC_RX) ? 1 : 0;
}

/*
static void cec_write(struct cec_priv* priv, u8 bit)
{
	if (bit)
		set32   (CEC_TX, priv->reg);
	else
		clear32 (CEC_TX, priv->reg);
}
*/

static void cecSendStart(struct cec_priv* const priv)
{
	cecPullLow  (priv);
	udelay      (CEC_START_LOW);
	cecPullHigh (priv);
	udelay      (CEC_START_HIGH);
}

static void cecSend0(struct cec_priv* const priv)
{
	cecPullLow  (priv);
	udelay      (CEC_0_LOW);
	cecPullHigh (priv);
	udelay      (CEC_0_HIGH);
}

static void cecSend1(struct cec_priv* const priv)
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
static u8 cecRequestAck (struct cec_priv* const priv)
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

static u8 cecSendByte (struct cec_priv* const priv, u8 payload, u8 eom)
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
static int cecArbitration (struct cec_priv* const priv)
{
	int lineFree = 1;
	u32 freeTime = 0, elapsed = 0;
	if (priv->mode == CecModeInitiator) {
		if (priv->sendFailed)
			freeTime = CEC_WAIT_SEND_FAILED;
		else
			freeTime = CEC_WAIT_NEXT_FRAME;
	}
	else
		freeTime = CEC_WAIT_NEW_INITIATOR;
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

/* Polls the CEC line while it is equal to @target@, with a timeout. Returns the
 * elapsed time in µs or an error code. */
static inline int cecPoll (struct cec_priv* const priv, u8 target,
			   u32 period, u32 timeout)
{
        int result = 0;
	u32 elapsed;
	while (cecCheckLine (priv) == target && elapsed <= timeout) {
		udelay (period);
		elapsed = elapsed + period;
	}
	if (elapsed > timeout)
		result = -ETIME;  /* Timeout expired. */
	else
		result = elapsed;
	return result;
}

/* Expects the start pulse. Returns 0 upon the end of the pulse or a status code
 * if an error occurred.
 *
 * FIXME: Cause a software interrupt.
 */
static int cecExpectStart (struct cec_priv* const priv)
{
	int result = 0;
	u32 elapsed = 0, elapsed_high = 0;
	while (cecCheckLine (priv) == 1)
		msleep (INTERFRAME_SAMPLE_PERIOD);
	elapsed_low = cecPoll (priv, 0, FAST_SAMPLE_PERIOD, CEC_START_LOW_MAX);
	if (elapsed_low > CEC_START_LOW_MAX)
		result = -ENOMSG;
	else {
		elapsed_high = cecPoll (priv, 1, FAST_SAMPLE_PERIOD,
					CEC_START_MAX - elapsed_low);
		if (elapsed_high < 0)
			result = -ENOMSG;
	}
	return result;
}

static int cecReceiveBit (struct cec_priv* const priv)
{
	int result;
	u32 elapsed_low = 0, elapsed_high = 0;
	elapsed_low = cecPoll (priv, 0, SAMPLE_PERIOD, CEC_0_LOW_MAX);
	if (elapsed_low <= CEC_1_LOW_MAX) {       /* Received 1. */
		elapsed_high = cecPoll (priv, 1, SAMPLE_PERIOD,
					CEC_BIT_MAX - elapsed_low);
		if (elapsed_high >= 0)
			result = 1;
		else
			result = -EBADMSG;
	}
	else if (elapsed_low <= CEC_0_LOW_MAX) {  /* Received 0. */
		elapsed_high = cecPoll (priv, 0, SAMPLE_PERIOD,
					CEC_BIT_MAX - elapsed_low);
		if (elapsed_high >= 0)
			result = 0;
		else
			result = -EBADMSG;
	}
	else                                      /* An error occurred. */
		result = -EBADMSG;
	/* FIXME: generate priv->irq */
	return result;
}

/* FIXME: acknowledgement bit */
static int cecReceiveBlock (struct cec_priv* const priv,
			    u8* const payload, u8* const eom)
{
	int result = 0;
	u8 i, bit = 0;
	for (i = 0; i < 9; i++) {   /* receive including the EOM bit */
		bit = cecReceiveBit (priv);
		if (bit == 0 || bit == 1)
			result = (result << 1) | bit;
		else if (bit < 0) { /* error code */
			result = bit;
			break;
		}
		else {              /* wrong return value */
			result = -ERANGE;
			break;
		}
	}
	if (result == 0 || result == 1) {
		cecSendAck (priv);
		*payload = result >> 1;
		*eom     = result & 1;
		result   = 0;
	}
	return result;
}

static int cecReceiveMessage (struct cec_priv* const priv,
			      u8* const data, u8* const len)
{
	int result = 0;
	struct CecBlock blk;
	u8 i, payload, eom;
	memset (msg, 0, sizeof (msg));
	result = cecExpectStart (priv);
	if (result == 0) {
		for (i = 0; i < CEC_MAX_MSG_LEN; i++) {
			result = cecReceiveBlock (priv, &payload, &eom);
			if (result < 0)          /* error code */
				break;
			data[i] = payload;
			if (eom)                 /* End of message. */
				break;
		}
		*len = i;
	}
	return result;
}

static irqreturn_t cecSunxiIsr (int irq, void *dev_id)
{
	struct cec_priv *priv = dev_id;
	int result = 0;
	u8 msg[CEC_MAX_MSG_LEN], len;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	result = cecReceiveMessage (priv, data, &len);
	spin_unlock_irqrestore(&priv->lock, flags);

	if (result >= 0)
		cec_receive_message(&priv->cec_drv, msg, len);
	else
		pr_err (PFX "CEC receive error %d\n", result);
	return IRQ_HANDLED;
}

static int cec_set_addr(struct cec_driver *drv, const u8 addr)
{
	struct cec_priv *priv =
		container_of(drv, struct cec_priv, cec_drv);

	priv->addr = addr;
	return 0;
}

static int cec_send(struct cec_driver *drv, const u8 *data, const u8 len)
{
	struct cec_priv *priv =
		container_of(drv, struct cec_priv, cec_drv);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	result = cecSendMessage (priv, data, len);
	spin_unlock_irqrestore(&priv->lock, flags);
	return result;
}

static int cec_reset(struct cec_driver *drv)
{
//	struct cec_priv *priv =
//		container_of(drv, struct cec_priv, cec_drv);

	pr_info(PFX "reset is not implemented\n");
	return 0;
}

static struct cec_driver_ops cec_ops = {
	.set_logical_address	= cec_set_addr,
	.send			= cec_send,
	.reset			= cec_reset,
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
	priv->cec_drv.ops = &cec_ops;

	ret = register_cec_driver(&priv->cec_drv);
	if (ret)
		goto out_iomem;

	ret = request_irq(irq, cecSunxiIsr, 0, pdev->name, priv);
	if (ret)
		goto out_drv;

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

static int cec_remove(struct platform_device *pdev)
{
	struct cec_priv *priv = platform_get_drvdata(pdev);

	free_irq(priv->irq, priv);
	unregister_cec_driver(&priv->cec_drv);
	iounmap(priv->regs);
	kfree(priv);

	return 0;
}

static struct platform_driver cec_driver = {
	.driver	= {
		.name	= "cec-sunxi",
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

module_init(cec_init);
module_exit(cec_exit);

MODULE_AUTHOR("Hutchison Technologies");
MODULE_DESCRIPTION("HDMI-CEC software emulation for Allwinner SunXi");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cec-sunxi");
