/*
 * Copyright (c) 2019-2020 Bilal Wasim
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT quectel_bg9x

#include <logging/log.h>
LOG_MODULE_REGISTER(modem_quectel_bg9x, CONFIG_MODEM_LOG_LEVEL);

/* --------------------------------------------------------------------------
 * Driver header files.
 * NOTE: Do not change the order of file inclusion.
 * -------------------------------------------------------------------------- */
#include "quectel-bg9x.h"
#include "modem_helper.h"

/* Func: modem_rx
 * Desc: Thread to process all messages received from the Modem. */
static void modem_rx(void)
{
	while (true)
	{
		k_sem_take(&mdata.iface_data.rx_sem, K_FOREVER);
		mctx.cmd_handler.process(&mctx.cmd_handler, &mctx.iface);
		k_yield();
	}
}

/* Func: modem_rssi_query_work
 * Desc: Routine to get Modem RSSI. */
static void modem_rssi_query_work(struct k_work *work)
{}

/* --------------------------------------------------------------------------
 * Everything beyond this point is just code to tie things up with the Zephyr
 * Modem offload stack.
 * -------------------------------------------------------------------------- */

static const struct socket_op_vtable offload_socket_fd_op_vtable = {
	.fd_vtable = {
		.read 	= NULL,
		.write 	= NULL,
		.close 	= NULL,
		.ioctl 	= NULL,
	},
	.bind 		= NULL,
	.connect 	= NULL,
	.sendto 	= NULL,
	.recvfrom 	= NULL,
	.listen 	= NULL,
	.accept 	= NULL,
	.sendmsg    = NULL,
	.getsockopt = NULL,
	.setsockopt = NULL,
};

static struct net_offload modem_net_offload = {
	.get = net_offload_dummy_get,
};

/* Setup the Modem NET Interface. */
static void modem_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct modem_data *data  = dev->data;

	/* Direct socket offload used instead of net offload: */
	iface->if_dev->offload = &modem_net_offload;
	net_if_set_link_addr(iface, modem_get_mac(dev),
			             sizeof(data->mac_addr),
			             NET_LINK_ETHERNET);
	data->net_iface = iface;
}

static struct net_if_api api_funcs = {
	.init = modem_net_iface_init,
};

static int modem_init(const struct device *dev)
{
	int ret; ARG_UNUSED(dev);

	k_sem_init(&mdata.sem_response, 0, 1);
	k_work_q_start(&modem_workq, modem_workq_stack,
		       	   K_KERNEL_STACK_SIZEOF(modem_workq_stack),
				   K_PRIO_COOP(7));

	/* socket config */
	mdata.socket_config.sockets         = &mdata.sockets[0];
	mdata.socket_config.sockets_len     = ARRAY_SIZE(mdata.sockets);
	mdata.socket_config.base_socket_num = MDM_BASE_SOCKET_NUM;
	ret = modem_socket_init(&mdata.socket_config,
							&offload_socket_fd_op_vtable);
	if (ret < 0)
		goto error;

	/* cmd handler */
#if 0
	mdata.cmd_handler_data.cmds[CMD_RESP]      = response_cmds;
	mdata.cmd_handler_data.cmds_len[CMD_RESP]  = ARRAY_SIZE(response_cmds);
	mdata.cmd_handler_data.cmds[CMD_UNSOL]     = unsol_cmds;
	mdata.cmd_handler_data.cmds_len[CMD_UNSOL] = ARRAY_SIZE(unsol_cmds);
#endif /* Will be added back once support is made available. */
	mdata.cmd_handler_data.read_buf            = &mdata.cmd_read_buf[0];
	mdata.cmd_handler_data.read_buf_len        = sizeof(mdata.cmd_read_buf);
	mdata.cmd_handler_data.match_buf           = &mdata.cmd_match_buf[0];
	mdata.cmd_handler_data.match_buf_len       = sizeof(mdata.cmd_match_buf);
	mdata.cmd_handler_data.buf_pool            = &mdm_recv_pool;
	mdata.cmd_handler_data.alloc_timeout       = BUF_ALLOC_TIMEOUT;
	mdata.cmd_handler_data.eol                 = "\r";
	ret = modem_cmd_handler_init(&mctx.cmd_handler,
				     	 	 	 &mdata.cmd_handler_data);
	if (ret < 0)
		goto error;

	/* modem interface */
	mdata.iface_data.isr_buf       = &mdata.iface_isr_buf[0];
	mdata.iface_data.isr_buf_len   = sizeof(mdata.iface_isr_buf);
	mdata.iface_data.rx_rb_buf     = &mdata.iface_rb_buf[0];
	mdata.iface_data.rx_rb_buf_len = sizeof(mdata.iface_rb_buf);
	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data,
				    			MDM_UART_DEV_NAME);
	if (ret < 0)
		goto error;

	/* modem data storage */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model        = mdata.mdm_model;
	mctx.data_revision     = mdata.mdm_revision;
	mctx.data_imei         = mdata.mdm_imei;

	/* pin setup */
	mctx.pins              = modem_pins;
	mctx.pins_len          = ARRAY_SIZE(modem_pins);
	mctx.driver_data       = &mdata;

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("Error registering modem context: %d", ret);
		goto error;
	}

	/* start RX thread */
	k_thread_create(&modem_rx_thread, modem_rx_stack,
					K_KERNEL_STACK_SIZEOF(modem_rx_stack),
					(k_thread_entry_t) modem_rx,
					NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	/* Init RSSI query */
	k_delayed_work_init(&mdata.rssi_query_work, modem_rssi_query_work);
	/* modem_reset(); */

error:
	return ret;
}

/* Register the device with the Networking stack. */
NET_DEVICE_OFFLOAD_INIT(modem_gb9x, CONFIG_MODEM_QUECTEL_BG9X_NAME,
			modem_init, device_pm_control_nop, &mdata, NULL,
			CONFIG_MODEM_QUECTEL_BG9X_INIT_PRIORITY, &api_funcs,
			MDM_MAX_DATA_LENGTH);
