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

/* --------------------------------------------------------------------------
 * Implementation of Modem Offload API(s) which allow normal Zephyr apps to
 * connect, disconnect, send / recv data and other stuff to/from any host on
 * the internet.
 * -------------------------------------------------------------------------- */

/* Func: send_socket_data
 * Desc: This function will send "binary" data over the socket object. */
static ssize_t send_socket_data(struct modem_socket *sock,
								const struct sockaddr *dst_addr,
								struct modem_cmd *handler_cmds,
								size_t handler_cmds_len,
								const char *buf, size_t buf_len,
								k_timeout_t timeout)
{
	int      ret;
	char     send_buf[32];
	char     ctrlz = 0x1A;

	/* Binary and ASCII mode allows sending MDM_MAX_DATA_LENGTH bytes to
	 * the socket in one command */
	if (buf_len > MDM_MAX_DATA_LENGTH)
		buf_len = MDM_MAX_DATA_LENGTH;

	/* Create a buffer with the correct params - As always, we are hardcoding
	 * socket number 0 instead of using the sock_id. */
	mdata.sock_written = buf_len;
	snprintk(send_buf, sizeof(send_buf), "AT+QISEND=%d,%ld", 0, (long int) buf_len);

	/* Setup the locks correctly. */
	k_sem_take(&mdata.cmd_handler_data.sem_tx_lock, K_FOREVER);
	k_sem_reset(&mdata.sem_tx_ready);

	/* Send the Modem command. */
	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler,
				    			NULL, 0U, send_buf, NULL, K_NO_WAIT);
	EXIT_ON_ERROR(ret);

	/* set command handlers */
	ret = modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
					    				handler_cmds, handler_cmds_len,
										true);
	EXIT_ON_ERROR(ret);

	/* Wait for '>' */
	ret = k_sem_take(&mdata.sem_tx_ready, K_MSEC(5000));
	if (ret < 0)
	{
		/* Didn't get the data prompt sadly. Need to exit. */
		LOG_DBG("Timeout waiting for tx");
		goto exit;
	}

	/* Now that we have received the data prompt, write all data on the console.
	 * Then send CTRL+Z. */
	mctx.iface.write(&mctx.iface, buf, buf_len);
	mctx.iface.write(&mctx.iface, &ctrlz, 1);

	/* Wait for 'SEND OK' or 'SEND FAIL' */
	k_sem_reset(&mdata.sem_response);
	ret = k_sem_take(&mdata.sem_response, timeout);
	if (ret < 0)
	{
		LOG_DBG("No send response");
		goto exit;
	}

	ret = modem_cmd_handler_get_error(&mdata.cmd_handler_data);
	if (ret != 0)
		LOG_DBG("Failed to send data");

exit:
	/* unset handler commands and ignore any errors */
	(void)modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
					    NULL, 0U, false);
	k_sem_give(&mdata.cmd_handler_data.sem_tx_lock);

	if (ret < 0)
		return ret;

	/* Return the amount of data written on the socket. */
	return mdata.sock_written;
}

/* Func: offload_sendto
 * Desc: This function will send data on the socket object. */
static ssize_t offload_sendto(void *obj, const void *buf, size_t len,
			      	  	  	  int flags, const struct sockaddr *to,
							  socklen_t tolen)
{
	int                 ret;
	struct modem_socket *sock = (struct modem_socket *) obj;

	/* Here's how sending data works,
	 * -> We firstly send the "AT+QISEND" command on the given socket and
	 *    specify the length of data to be transferred.
	 * -> In response to "AT+QISEND" command, the modem may respond with a
	 *    data prompt (>) or not respond at all. If it doesn't respond, we
	 *    exit. If it does respond with a data prompt (>), we move forward.
	 * -> We plainly write all data on the UART and terminate by sending a
	 *    CTRL+Z. Once the modem receives CTRL+Z, it starts processing the
	 *    data and will respond with either "SEND OK", "SEND FAIL" or "ERROR".
	 *    Here we are registering handlers for the first two responses. We
	 *    already have a handler for the "generic" error response. */
	struct modem_cmd    cmd[] =
	{
		MODEM_CMD_DIRECT(">", on_cmd_tx_ready),
		MODEM_CMD("SEND OK", on_cmd_send_ok,   0, ","),
		MODEM_CMD("SEND FAIL", on_cmd_send_fail, 0, ","),
	};

	/* Ensure that valid parameters are passed. */
	if (!buf || len == 0)
	{
		errno = EINVAL;
		return -1;
	}

	if (!sock->is_connected && sock->ip_proto != IPPROTO_UDP)
	{
		errno = ENOTCONN;
		return -1;
	}

	if (!to && sock->ip_proto == IPPROTO_UDP)
		to = &sock->dst;

	ret = send_socket_data(sock, to, cmd, ARRAY_SIZE(cmd), buf, len,
			       	   	   MDM_CMD_TIMEOUT);
	if (ret < 0)
	{
		errno = -ret;
		return -1;
	}

	/* Data was written successfully. */
	errno = 0;
	return ret;
}

/* Func: offload_read
 * Desc: This function reads data from the given socket object. */
static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
#if 0
	return offload_recvfrom(obj, buffer, count, 0, NULL, 0);
#else
	return 0;
#endif
}

/* Func: offload_write
 * Desc: This function writes data to the given socket object. */
static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

/* Func: offload_poll
 * Desc: This function polls on a given socket object. */
static int offload_poll(struct zsock_pollfd *fds, int nfds, int msecs)
{
	int i;
	void *obj;

	/* Only accept modem sockets. */
	for (i = 0; i < nfds; i++)
	{
		if (fds[i].fd < 0)
			continue;

		/* If vtable matches, then it's modem socket. */
		obj = z_get_fd_obj(fds[i].fd,
				   (const struct fd_op_vtable *) &offload_socket_fd_op_vtable,
				   EINVAL);
		if (obj == NULL)
			return -1;
	}

	return modem_socket_poll(&mdata.socket_config, fds, nfds, msecs);
}

/* Func: offload_ioctl
 * Desc: Function call to handle various misc requests. */
static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	switch (request)
	{
		case ZFD_IOCTL_POLL_PREPARE:
			return -EXDEV;

		case ZFD_IOCTL_POLL_UPDATE:
			return -EOPNOTSUPP;

		case ZFD_IOCTL_POLL_OFFLOAD:
		{
			/* Poll on the given socket. */
			struct zsock_pollfd *fds;
			int nfds, timeout;

			fds = va_arg(args, struct zsock_pollfd *);
			nfds = va_arg(args, int);
			timeout = va_arg(args, int);

			return offload_poll(fds, nfds, timeout);
		}

		default:
			errno = EINVAL;
			return -1;
	}
}

/* Func: offload_connect
 * Desc: This function will connect with a provided TCP / UDP host. */
static int offload_connect(void *obj, const struct sockaddr *addr,
			   	   	   	   socklen_t addrlen)
{
	struct modem_socket *sock     = (struct modem_socket *) obj;
	uint16_t            dst_port  = 0;
	char                *protocol = "TCP", buf[64] = {0};
	int                 ret;

	if (sock->id < mdata.socket_config.base_socket_num - 1)
	{
		LOG_ERR("Invalid socket_id(%d) from fd:%d",
				sock->id, sock->sock_fd);
		errno = EINVAL;
		return -1;
	}

	/* Find the correct destination port. */
	if (addr->sa_family == AF_INET6)
		dst_port = ntohs(net_sin6(addr)->sin6_port);
	else if (addr->sa_family == AF_INET)
		dst_port = ntohs(net_sin(addr)->sin_port);

	/* Are we dealing with UDP or TCP? */
	if (sock->ip_proto == IPPROTO_UDP)
		protocol = "UDP";

	/* Formulate the complete string.
	 * TODO: We are always using "socket 0" instead of dynamically deciding which one
	 * to use. This is OK for most apps in general and especially OK during project dev,
	 * but we will eventually add support for all sockets. Shouldn't take more than
	 * 30 mins to implement. */
	snprintk(buf, sizeof(buf), "AT+QIOPEN=%d,%d,\"%s\",\"%s\",%d,0,0", 0, 0, protocol,
		     modem_context_sprint_ip_addr(addr), dst_port);

	/* Send out the command. */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     	 	 NULL, 0U, buf,
						 &mdata.sem_response, MDM_CMD_CONN_TIMEOUT);
	if (ret < 0)
	{
		LOG_ERR("%s ret:%d", log_strdup(buf), ret);
		errno = -ret;
		return -1;
	}

	/* Connected successfully. */
	sock->is_connected = true;
	errno = 0;
	return 0;
}

/* Func: offload_close
 * Desc: This function closes the connection with the remote client and
 *       frees the socket. */
static int offload_close(void *obj)
{
	struct modem_socket *sock = (struct modem_socket *) obj;
	char                buf[32];
	int                 ret;

	/* Make sure we assigned an id */
	if (sock->id < mdata.socket_config.base_socket_num)
		return 0;

	/* Close the socket only if it is connected. */
	if (sock->is_connected)
	{
		snprintk(buf, sizeof(buf), "AT+QICLOSE=%d,%d", 0, 3);

		/* Tell the modem to close the socket (as with the "connect" call, we hardcode
		 * the socket id to 0. */
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     	 	 NULL, 0U, buf,
							 &mdata.sem_response, MDM_CMD_TIMEOUT);
		if (ret < 0)
			LOG_ERR("%s ret:%d", log_strdup(buf), ret);
	}

	modem_socket_put(&mdata.socket_config, sock->sock_fd);
	return 0;
}

/* Func: offload_sendmsg
 * Desc: This function sends messages to the modem. */
static ssize_t offload_sendmsg(void *obj, const struct msghdr *msg, int flags)
{
	ssize_t sent = 0;
	int rc;

	LOG_DBG("msg_iovlen:%d flags:%d", msg->msg_iovlen, flags);

	for (int i = 0; i < msg->msg_iovlen; i++)
	{
		const char *buf = msg->msg_iov[i].iov_base;
		size_t len      = msg->msg_iov[i].iov_len;

		while (len > 0)
		{
			rc = offload_sendto(obj, buf, len, flags,
							    msg->msg_name, msg->msg_namelen);
			if (rc < 0)
			{
				if (rc == -EAGAIN)
				{
					k_sleep(MDM_SENDMSG_SLEEP);
				}
				else
				{
					sent = rc;
					break;
				}
			}
			else
			{
				sent += rc;
				buf += rc;
				len -= rc;
			}
		}
	}

	return (ssize_t) sent;
}

/* --------------------------------------------------------------------------
 * Implementation of Modem Offload API(s) ends here.
 * -------------------------------------------------------------------------- */

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
		.read 	= offload_read,
		.write 	= offload_write,
		.close 	= offload_close,
		.ioctl 	= offload_ioctl,
	},
	.bind 		= NULL,
	.connect 	= offload_connect,
	.sendto 	= offload_sendto,
	.recvfrom 	= NULL,
	.listen 	= NULL,
	.accept 	= NULL,
	.sendmsg    = offload_sendmsg,
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
	k_sem_init(&mdata.sem_tx_ready, 0, 1);
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
	mdata.cmd_handler_data.cmds[CMD_RESP]      = response_cmds;
	mdata.cmd_handler_data.cmds_len[CMD_RESP]  = ARRAY_SIZE(response_cmds);
#if 0
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
