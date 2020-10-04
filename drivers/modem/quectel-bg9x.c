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

/* Setup commands - Commands sent to the modem to set it up at boot time.
 * TODO: We assume that the SIM card is already connected, and so there is
 * no handling if the card is not connected - Not critical and will be added
 * after the driver is fully functional. */
static struct setup_cmd setup_cmds[] =
{
	/* Commands to read info from the modem (things like IMEI, Model etc). */
	SETUP_CMD("AT+CGMI", "", on_cmd_atcmdinfo_manufacturer, 0U, ""),
	SETUP_CMD("AT+CGMM", "", on_cmd_atcmdinfo_model, 0U, ""),
	SETUP_CMD("AT+CGMR", "", on_cmd_atcmdinfo_revision, 0U, ""),
	SETUP_CMD("AT+CGSN", "", on_cmd_atcmdinfo_imei, 0U, ""),
	SETUP_CMD("AT+CIMI", "", on_cmd_atcmdinfo_imsi, 0U, ""),

	/* Connect the Modem to APN. */
	SETUP_CMD_NOHANDLE("AT+CGDCONT=1"),
	SETUP_CMD_NOHANDLE("AT+CGDCONT=1,\"IP\",\"" MDM_APN "\""),
	SETUP_CMD_NOHANDLE("AT+QIACT=1"),
};

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
{
	struct modem_cmd cmd  = MODEM_CMD("+CSQ: ", on_cmd_atcmdinfo_rssi_csq, 2U, ",");
	static char *send_cmd = "AT+CSQ";
	int ret;

	/* query modem RSSI */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     	     &cmd, 1U, send_cmd, &mdata.sem_response,
						 MDM_CMD_TIMEOUT);
	if (ret < 0)
		LOG_ERR("AT+CSQ ret:%d", ret);

	/* Re-start RSSI query work */
	if (work)
	{
		k_delayed_work_submit_to_queue(&modem_workq,
					       	   	   	   &mdata.rssi_query_work,
									   K_SECONDS(RSSI_TIMEOUT_SECS));
	}
}

/* Func: pin_init
 * Desc: DT bindings of quectel,bg9x marks power / reset pins as mandatory,
 *       and these pins are necessary to setup to get the modem up & running. */
static void pin_init(void)
{
	LOG_INF("Setting Modem Pins");

	/* NOTE: Per the BG95 document, the Reset pin is internally connected to the
	 * Power key pin. */

	/* MDM_POWER -> 1 for 500-1000 msec. */
	modem_pin_write(&mctx, MDM_POWER, 1);
	k_sleep(K_MSEC(750));

	/* MDM_POWER -> 0 and wait for ~2secs as UART remains in "inactive" state
	 * for some time after the power signal is enabled. */
	modem_pin_write(&mctx, MDM_POWER, 0);
	k_sleep(K_SECONDS(2));

	LOG_INF("... Done!");
}

/* Func: modem_setup
 * Desc: This function is used to setup the modem from zero. The idea
 *       is that this function will be called right after the modem is
 *       powered on to do the stuff necessary to talk to the modem. */
static void modem_setup(void)
{
	int ret = 0, retry_count = 0, counter = 0;

restart:

	/* stop RSSI delay work */
	k_delayed_work_cancel(&mdata.rssi_query_work);

	/* Setup the pins to ensure that Modem is enabled. */
	pin_init();

	/* Let the modem respond. */
	LOG_INF("Waiting for modem to respond");
	ret = modem_at(&mctx, &mdata);
	if (ret < 0)
	{
		LOG_ERR("MODEM WAIT LOOP ERROR: %d", ret);
		goto error;
	}

	/* Run setup commands on the modem. */
	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler,
					   	   setup_cmds, ARRAY_SIZE(setup_cmds),
						   &mdata.sem_response, MDM_REGISTRATION_TIMEOUT);
	if (ret < 0)
		goto error;

	/* query modem RSSI */
	modem_rssi_query_work(NULL);
	k_sleep(MDM_WAIT_FOR_RSSI_DELAY);

	/* Keep trying to read RSSI until we get a valid value - Eventually, exit. */
	while (counter++ < MDM_WAIT_FOR_RSSI_COUNT &&
	      (mctx.data_rssi >= 0 || mctx.data_rssi <= -1000))
	{
		modem_rssi_query_work(NULL);
		k_sleep(MDM_WAIT_FOR_RSSI_DELAY);
	}

	/* Is the RSSI invalid ? */
	if (mctx.data_rssi >= 0 || mctx.data_rssi <= -1000)
	{
		retry_count ++;

		/* Exit if we've tried long enough and its still not working.
		 * Indicates a much deeper problem. */
		if (retry_count >= MDM_NETWORK_RETRY_COUNT)
		{
			LOG_ERR("Failed network init. Too many attempts!");
			ret = -ENETUNREACH;
			goto error;
		}

		/* Try again! */
		LOG_ERR("Failed network init. Restarting process.");
		goto restart;
	}

	/* Network is ready - Start RSSI work in the background to inform us if
	 * signal strength drops down. */
	LOG_INF("Network is ready.");
	k_delayed_work_submit_to_queue(&modem_workq,
				       &mdata.rssi_query_work,
				       K_SECONDS(RSSI_TIMEOUT_SECS));

error:
	return;
}

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
	mdata.cmd_handler_data.eol                 = "\r\n";
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
	modem_setup();

error:
	return ret;
}

/* Register the device with the Networking stack. */
NET_DEVICE_OFFLOAD_INIT(modem_gb9x, CONFIG_MODEM_QUECTEL_BG9X_NAME,
			modem_init, device_pm_control_nop, &mdata, NULL,
			CONFIG_MODEM_QUECTEL_BG9X_INIT_PRIORITY, &api_funcs,
			MDM_MAX_DATA_LENGTH);
