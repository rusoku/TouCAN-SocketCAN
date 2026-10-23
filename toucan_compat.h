/* SPDX-License-Identifier: GPL-2.0 */
#ifndef TOUCAN_COMPAT_H
#define TOUCAN_COMPAT_H

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>

/* Linux 5.4-5.10: CAN FD helper names before the rename. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
#define can_fd_dlc2len(dlc) can_dlc2len(dlc)
#define can_fd_len2dlc(len) can_len2dlc(len)
#define can_cc_dlc2len(dlc) get_can_dlc(dlc)
#endif

/* Old CAN echo helpers lack the frame-length parameter.
 * The fourth argument to can_put_echo_skb() and the pointer argument
 * to can_get_echo_skb() are NOT equivalent to the payload length.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
#define can_put_echo_skb(skb, dev, idx, frame_len) \
	can_put_echo_skb(skb, dev, idx)
#define can_get_echo_skb(dev, idx, frame_len_ptr) can_get_echo_skb(dev, idx)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0)
#define can_free_echo_skb(dev, idx, frame_len_ptr) can_free_echo_skb(dev, idx)
#endif

/* Older kernels require the NAPI weight as a fourth argument. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
#define netif_napi_add_weight(dev, napi, poll, weight) \
	netif_napi_add(dev, napi, poll, weight)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
#define can_dev_dropped_skb(netdev, skb) can_dropped_invalid_skb(netdev, skb)
#endif

#endif /* TOUCAN_COMPAT_H */
