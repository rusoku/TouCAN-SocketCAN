// SPDX-License-Identifier: GPL-2.0-only
/*
 * CAN driver for Rusoku Technologies TouCAN adapter
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>
#include <linux/can.h>
#include <linux/can/dev.h>

/* driver constants */
#define MAX_RX_URBS			20
#define MAX_TX_URBS			20
#define RX_BUFFER_SIZE			64

struct toucan_usb_tx_urb_context {
	struct toucan_usb_priv *priv;

	u32 echo_index;
	u8 dlc;
};

struct toucan_usb_priv {
	struct can_priv can; /* must be the first member */

	struct sk_buff *echo_skb[MAX_TX_URBS];

	struct usb_device *udev;
	struct net_device *netdev;

	atomic_t active_tx_urbs;
	struct usb_anchor tx_submitted;
	struct toucan_usb_tx_urb_context tx_contexts[MAX_TX_URBS];

	struct usb_anchor rx_submitted;

	struct can_berr_counter bec;

	struct mutex toucan_cmd_lock;

	unsigned int ep_ctrl_in;
	unsigned int ep_ctrl_out;
	unsigned int ep_bulk_in;
	unsigned int ep_bulk_out;
	const struct toucan_usb_device *adapter;
};

struct toucan_usb_device {
	u8 ep_bulk_in;
	u8 ep_bulk_out;
	const struct can_bittiming_const *bittiming_const;
	void (*usb_rx_pkt_decode)(struct urb *urb);
	void (*usb_tx_pkt_encode)(struct sk_buff *skb, void *msg, u32 ctrlmode);
	__u32 base_clock;
};

#define TOUCAN_MSG_FLG_EXTID			0x01
#define TOUCAN_MSG_FLG_RTR			0x02
#define TOUCAN_MSG_FLG_ERR_FLAG			0x04

struct __packed toucan_usb_msg {
	u8 flags;
	__be32 id;
	u8 dlc;
	u8 data[8];
	__be32 timestamp;
};

/* Status message flags */
#define TOUCAN_STATE_MSG_OK			0x00  /* Normal condition. */
#define TOUCAN_STATE_MSG_OVERRUN		0x01  /* Overrun occurred when sending */
#define TOUCAN_STATE_MSG_BUSLIGHT		0x02  /* Error counter has reached 96 */
#define TOUCAN_STATE_MSG_BUSHEAVY		0x03  /* Error count. has reached 128 */
#define TOUCAN_STATE_MSG_BUSOFF		0x04  /* Device is in BUSOFF */
#define TOUCAN_STATE_MSG_STUFF			0x20  /* Stuff Error */
#define TOUCAN_STATE_MSG_FORM			0x21  /* Form Error */
#define TOUCAN_STATE_MSG_ACK			0x23  /* Ack Error */
#define TOUCAN_STATE_MSG_BIT0			0x24  /* Bit1 Error */
#define TOUCAN_STATE_MSG_BIT1			0x25  /* Bit0 Error */
#define TOUCAN_STATE_MSG_CRC			0x27  /* CRC Error */

struct __packed toucan_status_msg {
	u8 state;
	u8 rxerr;
	u8 txerr;
};

#define TOUCAN_INI_FLG_SILENT			0x01
#define TOUCAN_INI_FLG_LOOPBACK			0x02
#define TOUCAN_INI_FLG_DISABLE_AUTO_RESTRANS	0x04
#define TOUCAN_INI_FLG_STATUS_FRAME		0x100

struct __packed toucan_init_msg {
	u8 tseg1;
	u8 tseg2;
	u8 sjw;
	__be16 brp;
	__be32 flags;
};

#define TOUCAN_LASTERR_OK	0x0

enum toucan_usb_cmd {
	TOUCAN_CAN_INTERFACE_INIT = 0x01,
	TOUCAN_CAN_INTERFACE_DEINIT,
	TOUCAN_CAN_INTERFACE_START,
	TOUCAN_CAN_INTERFACE_STOP,

	TOUCAN_GET_HARDWARE_VERSION = 0x10,
	TOUCAN_GET_FIRMWARE_VERSION,
	TOUCAN_GET_BOOTLOADER_VERSION,
	TOUCAN_GET_SERIAL_NUMBER,

	TOUCAN_GET_LAST_ERROR_CODE = 0x20,
};

#define USB_RUSOKU_VENDOR_ID			0x16d0
#define USB_RUSOKU_TOUCAN_PRODUCT_ID		0x0eac

static const struct usb_device_id toucan_usb_table[] = {
	{ USB_DEVICE(USB_RUSOKU_VENDOR_ID, USB_RUSOKU_TOUCAN_PRODUCT_ID) },
	{ }				/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, toucan_usb_table);

static int __toucan_usb_cmd(struct toucan_usb_priv *priv, int direction, u8 cmd,
			    u16 len, void *msg)
{
	int flags;
	int ret;
	unsigned int pipe;

	switch (direction) {
	case USB_DIR_IN:
		flags = USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
		pipe = priv->ep_ctrl_in;
		break;
	case USB_DIR_OUT:
		flags = USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
		pipe = priv->ep_ctrl_out;
		break;
	default:
		return -EINVAL;
	}

	ret = usb_control_msg(priv->udev, pipe, cmd, flags, 0, 0, msg, len, 1000);

	if (ret < 0) {
		netdev_err(priv->netdev, "sending command message failed\n");
		return ret;
	}

	if (direction == USB_DIR_IN && ret != len)
		ret = -EPROTO;

	return ret;
}

static int toucan_usb_cmd(struct toucan_usb_priv *priv, int direction, u8 cmd,
			  u16 len, void *msg)
{
	int ret;
	void *buff_msg = NULL;
	void *buff_err = NULL;

	if (len > 0) {
		buff_msg = kmalloc(len, GFP_KERNEL);
		if (!buff_msg)
			return -ENOMEM;

		if (direction == USB_DIR_OUT)
			memcpy(buff_msg, msg, len);
	}

	buff_err = kmalloc(1, GFP_KERNEL);
	if (!buff_err) {
		ret = -ENOMEM;
		goto free_buf;
	}

	mutex_lock(&priv->toucan_cmd_lock);

	ret = __toucan_usb_cmd(priv, direction, cmd, len, buff_msg);
	if (ret < 0)
		goto cleanup;

	ret = __toucan_usb_cmd(priv, USB_DIR_IN, TOUCAN_GET_LAST_ERROR_CODE, 1, buff_err);
	if (ret < 0)
		goto cleanup;

	if (*((u8 *)buff_err) != TOUCAN_LASTERR_OK) {
		netdev_err(priv->netdev, "command 0x%x produced an error\n", cmd);
		ret = -EINVAL;
		goto cleanup;
	}

	if (direction == USB_DIR_IN && len > 0)
		memcpy(msg, buff_msg, len);

	ret = 0;

cleanup:
	mutex_unlock(&priv->toucan_cmd_lock);
free_buf:
	kfree(buff_msg);
	kfree(buff_err);
	return ret;
}

static void toucan_usb_unlink_all_urbs(struct toucan_usb_priv *priv)
{
	int i;

	usb_kill_anchored_urbs(&priv->rx_submitted);

	usb_kill_anchored_urbs(&priv->tx_submitted);
	atomic_set(&priv->active_tx_urbs, 0);

	for (i = 0; i < MAX_TX_URBS; i++)
		priv->tx_contexts[i].echo_index = MAX_TX_URBS;
}

static int toucan_usb_get_hw_version(struct toucan_usb_priv *priv, u32 *res)
{
	int ret = 0;
	__be32 ver;

	ret = toucan_usb_cmd(priv, USB_DIR_IN, TOUCAN_GET_HARDWARE_VERSION, 4, &ver);
	if (ret)
		return ret;

	*res = be32_to_cpu(ver);

	return ret;
}

static int toucan_usb_get_fw_version(struct toucan_usb_priv *priv, u32 *res)
{
	int ret = 0;
	__be32 ver;

	ret = toucan_usb_cmd(priv, USB_DIR_IN, TOUCAN_GET_FIRMWARE_VERSION, 4, &ver);
	if (ret)
		return ret;

	*res = be32_to_cpu(ver);

	return ret;
}

static int toucan_usb_cmd_init(struct toucan_usb_priv *priv)
{
	struct can_bittiming *bt = &priv->can.bittiming;
	struct toucan_init_msg msg;
	u32 ctrlmode = priv->can.ctrlmode;
	u32 flags = TOUCAN_INI_FLG_STATUS_FRAME; // enable error message reception

	memset(&msg, 0, sizeof(msg));

	msg.tseg1 = bt->prop_seg + bt->phase_seg1;
	msg.tseg2 = bt->phase_seg2;
	msg.sjw = bt->sjw;
	msg.brp = cpu_to_be16((u16)bt->brp);

	if (ctrlmode & CAN_CTRLMODE_LOOPBACK)
		flags |= TOUCAN_INI_FLG_LOOPBACK;
	if (ctrlmode & CAN_CTRLMODE_LISTENONLY)
		flags |= TOUCAN_INI_FLG_SILENT;
	if (ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		flags |= TOUCAN_INI_FLG_DISABLE_AUTO_RESTRANS;

	msg.flags = cpu_to_be32(flags);

	return toucan_usb_cmd(priv, USB_DIR_OUT, TOUCAN_CAN_INTERFACE_INIT,
			      sizeof(msg), &msg);
}

static int toucan_usb_cmd_deinit(struct toucan_usb_priv *priv)
{
	return toucan_usb_cmd(priv, USB_DIR_OUT, TOUCAN_CAN_INTERFACE_DEINIT, 0, NULL);
}

static int toucan_usb_cmd_start(struct toucan_usb_priv *priv)
{
	return toucan_usb_cmd(priv, USB_DIR_OUT, TOUCAN_CAN_INTERFACE_START, 0, NULL);
}

static int toucan_usb_cmd_stop(struct toucan_usb_priv *priv)
{
	return toucan_usb_cmd(priv, USB_DIR_OUT, TOUCAN_CAN_INTERFACE_STOP, 0, NULL);
}

static int toucan_usb_set_mode(struct net_device *netdev, enum can_mode mode)
{
	struct toucan_usb_priv *priv = netdev_priv(netdev);
	int err = 0;

	switch (mode) {
	case CAN_MODE_START:
		err = toucan_usb_cmd_stop(priv);
		if (err) {
			netdev_warn(netdev, "restart: couldn't stop device");
			break;
		}

		err = toucan_usb_cmd_deinit(priv);
		if (err) {
			netdev_warn(netdev, "restart: couldn't deinit device");
			break;
		}

		err = toucan_usb_cmd_init(priv);
		if (err) {
			netdev_warn(netdev, "restart: couldn't init device");
			break;
		}

		err = toucan_usb_cmd_start(priv);
		if (err) {
			netdev_warn(netdev, "couldn't start device");
			break;
		}

		break;
	default:
		return -EOPNOTSUPP;
	}

	return err;
}

static void toucan_usb_write_bulk_callback(struct urb *urb)
{
	struct toucan_usb_tx_urb_context *context = urb->context;
	struct toucan_usb_priv *priv;
	struct net_device *netdev;

	BUG_ON(!context);

	priv = context->priv;
	netdev = priv->netdev;

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0:
		netdev->stats.tx_packets++;
		netdev->stats.tx_bytes += context->dlc;
		can_get_echo_skb(netdev, context->echo_index);
		break;

	case -ENOENT:
	case -ECONNRESET:
	case -EPROTO:
	case -ESHUTDOWN:
		can_free_echo_skb(netdev, context->echo_index);
		break;

	default:
		if (net_ratelimit())
			netdev_err(netdev, "Tx URB aborted (%d)\n",
				   urb->status);
		can_free_echo_skb(netdev, context->echo_index);
		break;
	}

	/* Release context */
	context->echo_index = MAX_TX_URBS;
	atomic_dec(&priv->active_tx_urbs);

	if (!urb->status)
		netif_wake_queue(netdev);
}

static void toucan_rx_err_msg(struct toucan_usb_priv *priv,
			      struct toucan_usb_msg *msg)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &priv->netdev->stats;
	struct toucan_status_msg *status;
	int rx_errors = 0;
	int tx_errors = 0;

	status = (struct toucan_status_msg *)(&msg->data);

	skb = alloc_can_err_skb(priv->netdev, &cf);
	if (!skb)
		return;

	switch (status->state) {
	case TOUCAN_STATE_MSG_OK:
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
		cf->can_id |= CAN_ERR_PROT;
		cf->data[2] = CAN_ERR_PROT_ACTIVE;
		break;
	case TOUCAN_STATE_MSG_BUSOFF:
		priv->can.state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		priv->can.can_stats.bus_off++;
		can_bus_off(priv->netdev);
		break;
	case TOUCAN_STATE_MSG_OVERRUN:
	case TOUCAN_STATE_MSG_BUSLIGHT:
	case TOUCAN_STATE_MSG_BUSHEAVY:
		cf->can_id |= CAN_ERR_CRTL;
		break;
	default:
		priv->can.state = CAN_STATE_ERROR_WARNING;
		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
		priv->can.can_stats.bus_error++;
		break;
	}

	switch (status->state) {
	case TOUCAN_STATE_MSG_OK:
	case TOUCAN_STATE_MSG_BUSOFF:
		break;
	case TOUCAN_STATE_MSG_ACK:
		cf->can_id |= CAN_ERR_ACK;
		tx_errors = 1;
		break;
	case TOUCAN_STATE_MSG_CRC:
		cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
		rx_errors = 1;
		break;
	case TOUCAN_STATE_MSG_BIT0:
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		tx_errors = 1;
		break;
	case TOUCAN_STATE_MSG_BIT1:
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		tx_errors = 1;
		break;
	case TOUCAN_STATE_MSG_FORM:
		cf->data[2] |= CAN_ERR_PROT_FORM;
		rx_errors = 1;
		break;
	case TOUCAN_STATE_MSG_STUFF:
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		rx_errors = 1;
		break;
	case TOUCAN_STATE_MSG_OVERRUN:
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		rx_errors = 1;
		break;
	case TOUCAN_STATE_MSG_BUSLIGHT:
		priv->can.state = CAN_STATE_ERROR_WARNING;
		cf->data[1] = (status->txerr > status->rxerr) ?
			CAN_ERR_CRTL_TX_WARNING :
			CAN_ERR_CRTL_RX_WARNING;
		priv->can.can_stats.error_warning++;
		break;
	case TOUCAN_STATE_MSG_BUSHEAVY:
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		cf->data[1] = (status->txerr > status->rxerr) ?
			CAN_ERR_CRTL_TX_PASSIVE :
			CAN_ERR_CRTL_RX_PASSIVE;
		priv->can.can_stats.error_passive++;
		break;
	default:
		netdev_warn(priv->netdev,
			    "Unknown status/error message (%d)\n", status->state);
		break;
	}

	if (tx_errors) {
		cf->data[2] |= CAN_ERR_PROT_TX;
		stats->tx_errors++;
	}

	if (rx_errors)
		stats->rx_errors++;

	cf->data[6] = status->txerr;
	cf->data[7] = status->rxerr;

	priv->bec.txerr = status->txerr;
	priv->bec.rxerr = status->rxerr;

	stats->rx_packets++;
	stats->rx_bytes += cf->len;
	netif_rx(skb);
}

static void toucan_rx_can_msg(struct toucan_usb_priv *priv,
			      struct toucan_usb_msg *msg)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &priv->netdev->stats;

	if (msg->flags & TOUCAN_MSG_FLG_ERR_FLAG) {
		toucan_rx_err_msg(priv, msg);
	} else {
		skb = alloc_can_skb(priv->netdev, &cf);
		if (!skb)
			return;

		cf->can_id = be32_to_cpu(msg->id);
<<<<<<< HEAD
		can_frame_set_cc_len(cf, msg->dlc, priv->can.ctrlmode);
=======
		cf->len = can_get_cc_len(msg->dlc);
		cf->len8_dlc = can_get_len8_dlc(priv->can.ctrlmode,
						cf->len, msg->dlc);
>>>>>>> parent of 51f897e... Revert "toucan: add support for len8_dlc"

		if (msg->flags & TOUCAN_MSG_FLG_EXTID)
			cf->can_id |= CAN_EFF_FLAG;

		if (msg->flags & TOUCAN_MSG_FLG_RTR) {
			cf->can_id |= CAN_RTR_FLAG;
		} else {
			memcpy(cf->data, msg->data, cf->len);
			stats->rx_bytes += cf->len;
		}
		stats->rx_packets++;
<<<<<<< HEAD
		stats->rx_bytes += cf->len;
=======
>>>>>>> parent of 51f897e... Revert "toucan: add support for len8_dlc"
		netif_rx(skb);
	}
}

static void toucan_rx_usb_pkt(struct urb *urb)
{
	int pos = 0;
	struct toucan_usb_msg *msg;
	struct toucan_usb_priv *priv = urb->context;

	while (pos < urb->actual_length) {
		if (pos + sizeof(*msg) > urb->actual_length) {
			netdev_err(priv->netdev, "format error\n");
			break;
		}

		msg = (struct toucan_usb_msg *)(urb->transfer_buffer + pos);
		toucan_rx_can_msg(priv, msg);

		pos += sizeof(*msg);
	}
}

static void toucan_tx_usb_pkt(struct sk_buff *skb, void *buf, u32 ctrlmode)
{
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct toucan_usb_msg *msg = (struct toucan_usb_msg *)buf;

	memset(buf, 0, sizeof(*msg));

	if (cf->can_id & CAN_RTR_FLAG)
		msg->flags |= TOUCAN_MSG_FLG_RTR;

	if (cf->can_id & CAN_EFF_FLAG)
		msg->flags |= TOUCAN_MSG_FLG_EXTID;

	msg->id = cpu_to_be32(cf->can_id & CAN_ERR_MASK);
	msg->dlc = can_get_cc_dlc(ctrlmode, cf->len, cf->len8_dlc);
	memcpy(msg->data, cf->data, cf->len);
}

static void toucan_usb_read_bulk_callback(struct urb *urb)
{
	struct toucan_usb_priv *priv = urb->context;
	struct net_device *netdev;
	int retval;

	netdev = priv->netdev;

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0: /* success */
		break;

	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		return;

	default:
		if (net_ratelimit())
			netdev_err(netdev, "Rx URB aborted (%d)\n", urb->status);
		goto resubmit_urb;
	}

	priv->adapter->usb_rx_pkt_decode(urb);

resubmit_urb:
	usb_fill_bulk_urb(urb, priv->udev, priv->ep_bulk_in,
			  urb->transfer_buffer, RX_BUFFER_SIZE,
			  toucan_usb_read_bulk_callback, priv);

	retval = usb_submit_urb(urb, GFP_ATOMIC);

	if (retval == -ENODEV)
		netif_device_detach(netdev);
	else if (retval)
		netdev_err(netdev, "failed resubmitting read bulk urb: %d\n",
			   retval);
}

static netdev_tx_t toucan_usb_start_xmit(struct sk_buff *skb,
					 struct net_device *netdev)
{
	struct toucan_usb_priv *priv = netdev_priv(netdev);
	struct toucan_usb_tx_urb_context *context = NULL;
	struct net_device_stats *stats = &netdev->stats;
	struct can_frame *cf;
	struct urb *urb;
	int i, err;
	u8 *buf;
	size_t size = sizeof(struct toucan_usb_msg);

	if (can_dropped_invalid_skb(netdev, skb))
		return NETDEV_TX_OK;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto nomem;

	buf = usb_alloc_coherent(priv->udev, size, GFP_ATOMIC,
				 &urb->transfer_dma);
	if (!buf) {
		netdev_err(netdev, "No memory left for USB buffer\n");
		goto nomembuf;
	}

	priv->adapter->usb_tx_pkt_encode(skb, buf, priv->can.ctrlmode);

	for (i = 0; i < MAX_TX_URBS; i++) {
		if (priv->tx_contexts[i].echo_index == MAX_TX_URBS) {
			context = &priv->tx_contexts[i];
			context->echo_index = i;
			context->priv = priv;
			break;
		}
	}

	/* May never happen! When this happens we'd more URBs in flight as
	 * allowed (MAX_TX_URBS).
	 */
	if (!context)
		goto nofreecontext;

	cf = (struct can_frame *)skb->data;
	context->dlc = cf->len;

	usb_fill_bulk_urb(urb, priv->udev, priv->ep_bulk_out, buf,
			  size, toucan_usb_write_bulk_callback, context);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &priv->tx_submitted);

	can_put_echo_skb(skb, netdev, context->echo_index);

	atomic_inc(&priv->active_tx_urbs);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (unlikely(err))
		goto failed;
	else if (atomic_read(&priv->active_tx_urbs) >= MAX_TX_URBS)
		/* Slow down tx path */
		netif_stop_queue(netdev);

	/* Release our reference to this URB, the USB core will eventually free
	 * it entirely.
	 */
	usb_free_urb(urb);

	return NETDEV_TX_OK;

nofreecontext:
	usb_free_coherent(priv->udev, size, buf, urb->transfer_dma);
	usb_free_urb(urb);

	netdev_warn(netdev, "couldn't find free context");

	return NETDEV_TX_BUSY;

failed:
	can_free_echo_skb(netdev, context->echo_index);

	usb_unanchor_urb(urb);
	usb_free_coherent(priv->udev, size, buf, urb->transfer_dma);

	atomic_dec(&priv->active_tx_urbs);

	if (err == -ENODEV)
		netif_device_detach(netdev);
	else
		netdev_warn(netdev, "failed tx_urb %d\n", err);

nomembuf:
	usb_free_urb(urb);

nomem:
	dev_kfree_skb(skb);
	stats->tx_dropped++;

	return NETDEV_TX_OK;
}

static int toucan_usb_alloc_rx_urbs(struct toucan_usb_priv *priv)
{
	struct net_device *netdev = priv->netdev;
	int err = 0, i;

	for (i = 0; i < MAX_RX_URBS; i++) {
		struct urb *urb = NULL;
		u8 *buf;

		/* create a URB, and a buffer for it */
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			err = -ENOMEM;
			break;
		}

		buf = usb_alloc_coherent(priv->udev, RX_BUFFER_SIZE, GFP_KERNEL,
					 &urb->transfer_dma);
		if (!buf) {
			usb_free_urb(urb);
			err = -ENOMEM;
			break;
		}

		usb_fill_bulk_urb(urb, priv->udev, priv->ep_bulk_in, buf,
				  RX_BUFFER_SIZE, toucan_usb_read_bulk_callback,
				  priv);
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		usb_anchor_urb(urb, &priv->rx_submitted);

		err = usb_submit_urb(urb, GFP_KERNEL);
		if (err) {
			usb_unanchor_urb(urb);
			usb_free_coherent(priv->udev, RX_BUFFER_SIZE, buf,
					  urb->transfer_dma);
			usb_free_urb(urb);
			break;
		}

		/* Drop reference, USB core will take care of freeing it */
		usb_free_urb(urb);
	}

	/* Did we submit any URBs */
	if (i == 0) {
		netdev_warn(netdev, "couldn't setup read URBs\n");
		return err;
	}

	/* Warn if we've couldn't submit all the URBs */
	if (i < MAX_RX_URBS)
		netdev_warn(netdev, "rx performance may be slow\n");

	return err;
}

static int toucan_usb_open(struct net_device *netdev)
{
	struct toucan_usb_priv *priv = netdev_priv(netdev);
	int err;

	err = open_candev(netdev);
	if (err)
		return err;

	err = toucan_usb_alloc_rx_urbs(priv);
	if (err)
		goto cleanup_candev;

	err = toucan_usb_cmd_init(priv);
	if (err)
		goto cleanup_candev;

	err = toucan_usb_cmd_start(priv);
	if (err)
		goto cleanup_candev;

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	netif_start_queue(netdev);

	return 0;

cleanup_candev:
	close_candev(netdev);
	return err;
}

static int toucan_usb_close(struct net_device *netdev)
{
	struct toucan_usb_priv *priv = netdev_priv(netdev);
	int err = 0;

	netif_stop_queue(netdev);

	err = toucan_usb_cmd_stop(priv);
	err |= toucan_usb_cmd_deinit(priv);
	if (err)
		netdev_warn(netdev, "couldn't stop device");

	priv->can.state = CAN_STATE_STOPPED;

	/* Stop polling */
	toucan_usb_unlink_all_urbs(priv);

	close_candev(netdev);

	return err;
}

static int toucan_usb_get_berr_counter(const struct net_device *netdev,
				       struct can_berr_counter *bec)
{
	struct toucan_usb_priv *priv = netdev_priv(netdev);

	bec->txerr = priv->bec.txerr;
	bec->rxerr = priv->bec.rxerr;

	return 0;
}

static const struct net_device_ops toucan_usb_netdev_ops = {
	.ndo_open = toucan_usb_open,
	.ndo_stop = toucan_usb_close,
	.ndo_start_xmit = toucan_usb_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static const struct can_bittiming_const toucan_bittiming_const = {
	.name = "toucan",
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static const struct toucan_usb_device toucan = {
	.ep_bulk_in = 1,
	.ep_bulk_out = 1,
	.bittiming_const = &toucan_bittiming_const,
	.usb_rx_pkt_decode = toucan_rx_usb_pkt,
	.usb_tx_pkt_encode = toucan_tx_usb_pkt,
	.base_clock = 50000000,
};

static int toucan_usb_probe(struct usb_interface *intf,
			    const struct usb_device_id *id)
{
	int i, err;
	u8 *hw, *fw;
	unsigned int hw_ver, fw_ver;
	struct net_device *netdev;
	struct toucan_usb_priv *priv;
	const struct toucan_usb_device *adapter;
	struct usb_device *usbdev = interface_to_usbdev(intf);

	switch (id->idProduct) {
	case USB_RUSOKU_TOUCAN_PRODUCT_ID:
		adapter = &toucan;
		break;
	default:
		dev_err(&intf->dev, "Unknown device id:0x%x\n", id->idProduct);
		break;
	}

	netdev = alloc_candev(sizeof(struct toucan_usb_priv), MAX_TX_URBS);
	if (!netdev) {
		dev_err(&intf->dev, "Can't alloc candev\n");
		return -ENOMEM;
	}

	priv = netdev_priv(netdev);

	priv->udev = usbdev;
	priv->netdev = netdev;

	priv->can.state = CAN_STATE_STOPPED;
	priv->can.clock.freq = adapter->base_clock;
	priv->can.bittiming_const = adapter->bittiming_const;
	priv->can.do_set_mode = toucan_usb_set_mode;
	priv->can.do_get_berr_counter = toucan_usb_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
				       CAN_CTRLMODE_LISTENONLY |
				       CAN_CTRLMODE_ONE_SHOT |
				       CAN_CTRLMODE_CC_LEN8_DLC;

	priv->adapter = adapter;
	priv->ep_ctrl_in = usb_rcvctrlpipe(priv->udev, 0);
	priv->ep_ctrl_out = usb_sndctrlpipe(priv->udev, 0);
	priv->ep_bulk_in = usb_rcvbulkpipe(priv->udev, adapter->ep_bulk_in);
	priv->ep_bulk_out = usb_sndbulkpipe(priv->udev, adapter->ep_bulk_out);

	netdev->flags |= IFF_ECHO;
	netdev->netdev_ops = &toucan_usb_netdev_ops;

	init_usb_anchor(&priv->rx_submitted);
	init_usb_anchor(&priv->tx_submitted);
	atomic_set(&priv->active_tx_urbs, 0);

	for (i = 0; i < MAX_TX_URBS; i++)
		priv->tx_contexts[i].echo_index = MAX_TX_URBS;

	usb_set_intfdata(intf, priv);

	SET_NETDEV_DEV(netdev, &intf->dev);

	mutex_init(&priv->toucan_cmd_lock);

	err = register_candev(netdev);
	if (err) {
		dev_err(&intf->dev, "Failed to register CAN device\n");
		goto cleanup_candev;
	}

	err = toucan_usb_get_hw_version(priv, &hw_ver);
	if (err) {
		netdev_err(netdev, "can't get hardware version\n");
		goto cleanup_unregister_candev;
	}
	err = toucan_usb_get_fw_version(priv, &fw_ver);
	if (err) {
		netdev_err(netdev, "can't get firmware version\n");
		goto cleanup_unregister_candev;
	}

	hw = (u8 *)(&hw_ver);
	fw = (u8 *)(&fw_ver);
	netdev_info(netdev, "registered. HW ver: %d.%d.%d, FW ver: %d.%d.%d\n",
		    hw[3], hw[2], hw[1], fw[3], fw[2], fw[1]);

	return 0;

cleanup_unregister_candev:
	unregister_netdev(priv->netdev);

cleanup_candev:
	free_candev(netdev);
	return err;
}

static void toucan_usb_disconnect(struct usb_interface *intf)
{
	struct toucan_usb_priv *priv = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);

	if (priv) {
		netdev_info(priv->netdev, "device disconnected\n");

		unregister_netdev(priv->netdev);
		toucan_usb_unlink_all_urbs(priv);
		free_candev(priv->netdev);
	}
}

static struct usb_driver toucan_usb_driver = {
	.name =		"toucan",
	.probe =	toucan_usb_probe,
	.disconnect =	toucan_usb_disconnect,
	.id_table =	toucan_usb_table,
};

module_usb_driver(toucan_usb_driver);

MODULE_DESCRIPTION("CAN driver for Rusoku Technologies TouCAN adapters");
MODULE_LICENSE("GPL v2");
