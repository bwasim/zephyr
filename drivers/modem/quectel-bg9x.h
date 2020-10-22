/*
 * Copyright (c) 2020 Analog Life LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef QUECTEL_BG9X_H
#define QUECTEL_BG9X_H

#include <kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <device.h>
#include <init.h>

#include <net/net_if.h>
#include <net/net_offload.h>
#include <net/socket_offload.h>

#include "modem_context.h"
#include "modem_socket.h"
#include "modem_cmd_handler.h"
#include "modem_iface_uart.h"

#define MDM_UART_DEV_NAME		  DT_INST_BUS_LABEL(0)
#define MDM_CMD_TIMEOUT			  K_SECONDS(10)
#define MDM_CMD_CONN_TIMEOUT		  K_SECONDS(120)
#define MDM_REGISTRATION_TIMEOUT	  K_SECONDS(180)
#define MDM_SENDMSG_SLEEP		  K_MSEC(1)
#define MDM_MAX_DATA_LENGTH		  1024
#define MDM_RECV_MAX_BUF		  30
#define MDM_RECV_BUF_SIZE		  1024
#define MDM_MAX_SOCKETS			  5
#define MDM_BASE_SOCKET_NUM		  0
#define MDM_NETWORK_RETRY_COUNT		  10
#define MDM_WAIT_FOR_RSSI_COUNT		  10
#define MDM_WAIT_FOR_RSSI_DELAY		  K_SECONDS(2)
#define BUF_ALLOC_TIMEOUT		  K_SECONDS(1)

/* Default lengths of certain things. */
#define MDM_MANUFACTURER_LENGTH		  10
#define MDM_MODEL_LENGTH		  16
#define MDM_REVISION_LENGTH		  64
#define MDM_IMEI_LENGTH			  16
#define MDM_IMSI_LENGTH			  16
#define MDM_ICCID_LENGTH		  32
#define MDM_APN_LENGTH			  32
#define RSSI_TIMEOUT_SECS		  30

#define MDM_APN				  CONFIG_MODEM_QUECTEL_BG9X_APN
#define MDM_USERNAME			  CONFIG_MODEM_QUECTEL_BG9X_USERNAME
#define MDM_PASSWORD			  CONFIG_MODEM_QUECTEL_BG9X_PASSWORD

/* Modem ATOI routine. */
#define ATOI(s_, value_, desc_)   modem_atoi(s_, value_, desc_, __func__)

/* pin settings */
enum mdm_control_pins {
	MDM_POWER = 0,
	MDM_RESET,
#if DT_INST_NODE_HAS_PROP(0, mdm_dtr_gpios)
	MDM_DTR,
#endif
};

/* driver data */
struct modem_data {
	struct net_if *net_iface;
	uint8_t mac_addr[6];

	/* modem interface */
	struct modem_iface_uart_data iface_data;
	uint8_t iface_isr_buf[MDM_RECV_BUF_SIZE];
	uint8_t iface_rb_buf[MDM_MAX_DATA_LENGTH];

	/* modem cmds */
	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_read_buf[MDM_RECV_BUF_SIZE];
	uint8_t cmd_match_buf[MDM_RECV_BUF_SIZE + 1];

	/* socket data */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];

	/* RSSI work */
	struct k_delayed_work rssi_query_work;

	/* modem data */
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	char mdm_imsi[MDM_IMSI_LENGTH];
	char mdm_iccid[MDM_ICCID_LENGTH];
#endif /* #if defined(CONFIG_MODEM_SIM_NUMBERS) */

	/* bytes written to socket in last transaction */
	int sock_written;

	/* Socket from which we are currently reading data. */
	int sock_fd;

	/* Semaphore(s) */
	struct k_sem sem_response;
	struct k_sem sem_tx_ready;
	struct k_sem sem_sock_conn;
};

/* Socket read callback data */
struct socket_read_data {
	char             *recv_buf;
	size_t           recv_buf_len;
	struct sockaddr  *recv_addr;
	uint16_t         recv_read_len;
};

/* Modem pins - Power, Reset & others. */
static struct modem_pin modem_pins[] = {
	/* MDM_POWER */
	MODEM_PIN(DT_INST_GPIO_LABEL(0, mdm_power_gpios),
		  DT_INST_GPIO_PIN(0, mdm_power_gpios),
		  DT_INST_GPIO_FLAGS(0, mdm_power_gpios) | GPIO_OUTPUT_LOW),

	/* MDM_RESET */
	MODEM_PIN(DT_INST_GPIO_LABEL(0, mdm_reset_gpios),
		  DT_INST_GPIO_PIN(0, mdm_reset_gpios),
		  DT_INST_GPIO_FLAGS(0, mdm_reset_gpios) | GPIO_OUTPUT_LOW),

#if DT_INST_NODE_HAS_PROP(0, mdm_dtr_gpios)
	/* MDM_DTR */
	MODEM_PIN(DT_INST_GPIO_LABEL(0, mdm_dtr_gpios),
		  DT_INST_GPIO_PIN(0, mdm_dtr_gpios),
		  DT_INST_GPIO_FLAGS(0, mdm_dtr_gpios) | GPIO_OUTPUT_LOW),
#endif
};

static inline int digits(int n)
{
	int count = 0;

	while (n != 0) {
		n /= 10;
		++count;
	}

	return count;
}

static inline int find_len(char *data)
{
	char buf[10] = {0};
	int  i;

	for (i = 0; i < 10; i++) {
		if (data[i] == '\r')
			break;

		buf[i] = data[i];
	}

	return atoi(buf);
}

static inline uint32_t hash32(char *str, int len)
{
#define HASH_MULTIPLIER		37

	uint32_t h = 0;
	int i;

	for (i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}

	return h;
}

static int net_offload_dummy_get(sa_family_t family, enum net_sock_type type,
				 enum net_ip_protocol ip_proto,
				 struct net_context **context)
{

	LOG_ERR("CONFIG_NET_SOCKETS_OFFLOAD must be enabled for this driver");
	return -ENOTSUP;
}

#endif /* #ifndef QUECTEL_BG9X_H */
