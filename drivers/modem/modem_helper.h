/*
 * Copyright (c) 2019-2020 Bilal Wasim
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MODEM_HELPER_H
#define MODEM_HELPER_H

/* Helper routine to generate hash. */
static uint32_t hash32(char *str, int len)
{
#define HASH_MULTIPLIER		37

	uint32_t h = 0;
	int i;

	for (i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}

	return h;
}

/* Func: modem_get_mac
 * Desc: This function will generate the Modem MAC address from
 *       Modem IMEI. As the IMEI of this device will be unique
 *       and never change, each device will have its own unique
 *       MAC address. */
static inline uint8_t *modem_get_mac(const struct device *dev)
{
	struct modem_data *data = dev->data;
	uint32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	/* use IMEI for mac_addr */
	hash_value = hash32(mdata.mdm_imei, strlen(mdata.mdm_imei));

	UNALIGNED_PUT(hash_value, (uint32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

/* Func: net_offload_dummy_get
 * Desc: A dummy function to bypass lack of support for "NULL"
 *       net_offload in the Zephyr Networking stack. */
static int net_offload_dummy_get(sa_family_t family, enum net_sock_type type,
				 	 	 	 	 enum net_ip_protocol ip_proto, struct net_context **context)
{

	LOG_ERR("CONFIG_NET_SOCKETS_OFFLOAD must be enabled for this driver");
	return -ENOTSUP;
}

/* Func: modem_atoi
 * Desc: Convert string to long integer, but handle errors */
static int modem_atoi(const char *s, const int err_value,
		      	  	  const char *desc, const char *func)
{
	int   ret;
	char  *endptr;

	ret = (int)strtol(s, &endptr, 10);
	if (!endptr || *endptr != '\0')
	{
		LOG_ERR("bad %s '%s' in %s", log_strdup(s), log_strdup(desc),
				log_strdup(func));
		return err_value;
	}

	return ret;
}

/* Func: modem_at
 * Desc: Send "AT" command to the modem and wait for it to
 *       respond. Eventually as everything is "finite", we will give
 *       up and kill the driver. */
static int modem_at(struct modem_context *mctx, struct modem_data *mdata)
{
	int counter = 0, ret = -1;

	while (counter < 50 && ret < 0)
	{
		k_sleep(K_SECONDS(2));

		/* Send "AT" command to the modem. */
		ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler,
				     	 	 NULL, 0, "AT", &mdata->sem_response,
							 MDM_CMD_TIMEOUT);

		/* Check the response from the Modem. */
		if (ret < 0 && ret != -ETIMEDOUT)
			return ret;

		counter ++;
	}

	return ret;
}

/* Func: on_cmd_sockread_common
 * Desc: Function to successfully read data from the modem on a given socket. */
static int on_cmd_sockread_common(int socket_id,
				  struct modem_cmd_handler_data *data,
				  int socket_data_length, uint16_t len)
{
	struct modem_socket      *sock = NULL;
	struct socket_read_data  *sock_data;
	int                      ret;

	if (!len)
	{
		LOG_ERR("Invalid length, Aborting!");
		return -EAGAIN;
	}

	/*
	 * make sure we still have buf data and next char in the buffer is a
	 * quote.
	 */
	if (!data->rx_buf || *data->rx_buf->data != '\"') {
		LOG_ERR("Incorrect format! Ignoring data!");
		return -EINVAL;
	}

	/* No data available on the socket. */
	if (socket_data_length <= 0)
	{
		LOG_ERR("Length problem (%d).  Aborting!", socket_data_length);
		return -EAGAIN;
	}

	/* check to make sure we have all of the data (minus the initial 0x0A) */
	if ((net_buf_frags_len(data->rx_buf) - 1) < socket_data_length)
	{
		LOG_DBG("Not enough data -- wait!");
		return -EAGAIN;
	}

	/* Skip "0x0A" */
	len --;
	net_buf_pull_u8(data->rx_buf);
	if (!data->rx_buf->len)
	{
		data->rx_buf = net_buf_frag_del(NULL, data->rx_buf);
	}

	sock = modem_socket_from_id(&mdata.socket_config, socket_id);
	if (!sock)
	{
		LOG_ERR("Socket not found! (%d)", socket_id);
		ret = -EINVAL;
		goto exit;
	}

	sock_data = (struct socket_read_data *)sock->data;
	if (!sock_data)
	{
		LOG_ERR("Socket data not found! Skip handling (%d)", socket_id);
		ret = -EINVAL;
		goto exit;
	}

	ret = net_buf_linearize(sock_data->recv_buf, sock_data->recv_buf_len,
				data->rx_buf, 0, (uint16_t)socket_data_length);
	data->rx_buf = net_buf_skip(data->rx_buf, ret);
	sock_data->recv_read_len = ret;
	if (ret != socket_data_length)
	{
		LOG_ERR("Total copied data is different then received data!"
			" copied:%d vs. received:%d", ret, socket_data_length);
		ret = -EINVAL;
	}

exit:
	/* remove packet from list (ignore errors) */
	(void)modem_socket_packet_size_update(&mdata.socket_config, sock,
					      -socket_data_length);

	/* don't give back semaphore -- OK to follow */
	return ret;
}

/* --------------------------------------------------------------------------
 * Everything beyond this point are implementation of Modem command handlers.
 * Whenever we send a command to the modem (via modem_cmd_send), the modem has
 * to respond with some sort of message. As these responses can vary a lot, we
 * need different handlers to handle different kinds of responses.
 * -------------------------------------------------------------------------- */

/* Handler: OK */
MODEM_CMD_DEFINE(on_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: ERROR */
MODEM_CMD_DEFINE(on_cmd_error)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CME Error: <err>[0] */
MODEM_CMD_DEFINE(on_cmd_exterror)
{
	/* TODO: map extended error codes to values */
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CSQ: <signal_power>[0], <qual>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_rssi_csq)
{
	int rssi = ATOI(argv[0], 0, "signal_power");

	/* Check the RSSI value. */
	if (rssi == 31)
		mctx.data_rssi = -51;
	else if (rssi >= 0 && rssi <= 31)
		mctx.data_rssi = -114 + ((rssi * 2) + 1);
	else
		mctx.data_rssi = -1000;

	LOG_INF("RSSI: %d", mctx.data_rssi);
	return 0;
}

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len = net_buf_linearize(mdata.mdm_manufacturer,
				    		sizeof(mdata.mdm_manufacturer) - 1,
							data->rx_buf, 0, len);
	mdata.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", log_strdup(mdata.mdm_manufacturer));
	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len = net_buf_linearize(mdata.mdm_model,
				    		sizeof(mdata.mdm_model) - 1,
							data->rx_buf, 0, len);
	mdata.mdm_model[out_len] = '\0';

	/* Log the received information. */
	LOG_INF("Model: %s", log_strdup(mdata.mdm_model));
	return 0;
}

/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	 size_t out_len = net_buf_linearize(mdata.mdm_revision,
				    		sizeof(mdata.mdm_revision) - 1,
							data->rx_buf, 0, len);
	mdata.mdm_revision[out_len] = '\0';

	/* Log the received information. */
	LOG_INF("Revision: %s", log_strdup(mdata.mdm_revision));
	return 0;
}

/* Handler: <IMEI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei)
{
	size_t out_len = net_buf_linearize(mdata.mdm_imei,
							sizeof(mdata.mdm_imei) - 1,
				    		data->rx_buf, 0, len);
	mdata.mdm_imei[out_len] = '\0';

	/* Log the received information. */
	LOG_INF("IMEI: %s", log_strdup(mdata.mdm_imei));
	return 0;
}

/* Handler: <IMSI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imsi)
{
	size_t  out_len = net_buf_linearize(mdata.mdm_imsi,
							sizeof(mdata.mdm_imsi) - 1,
				    		data->rx_buf, 0, len);
	mdata.mdm_imsi[out_len] = '\0';

	/* Log the received information. */
	LOG_INF("IMSI: %s", log_strdup(mdata.mdm_imsi));
	return 0;
}

/* Handler: <ICCID> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_iccid)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_iccid, sizeof(mdata.mdm_iccid) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_iccid[out_len] = '\0';
	LOG_INF("ICCID: %s", log_strdup(mdata.mdm_iccid));

	return 0;
}

/* Handler: TX Ready */
MODEM_CMD_DIRECT_DEFINE(on_cmd_tx_ready)
{
	/* We are ready to receive data. */
	k_sem_give(&mdata.sem_tx_ready);
	return len;
}

/* Handler: SEND OK */
MODEM_CMD_DEFINE(on_cmd_send_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);

	return 0;
}

/* Handler: SEND FAIL */
MODEM_CMD_DEFINE(on_cmd_send_fail)
{
	mdata.sock_written = 0;
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);

	return 0;
}

/* Handler: Read data */
MODEM_CMD_DEFINE(on_cmd_sockread)
{
	return on_cmd_sockread_common(0, data, ATOI(argv[0], 0, "length"), len);
}

static struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", on_cmd_ok, 0U, ""),
	MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
	MODEM_CMD("+CME ERROR: ", on_cmd_exterror, 1U, ""),
};

#endif /* #ifndef MODEM_HELPER_H */
