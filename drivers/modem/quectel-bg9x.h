/*
 * Copyright (c) 2019-2020 Bilal Wasim
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
#define MDM_POWER_ENABLE		  1
#define MDM_POWER_DISABLE		  0
#define MDM_RESET_NOT_ASSERTED	  1
#define MDM_RESET_ASSERTED		  0
#define MDM_CMD_TIMEOUT			  K_SECONDS(10)
#define MDM_DNS_TIMEOUT			  K_SECONDS(70)
#define MDM_CMD_CONN_TIMEOUT	  K_SECONDS(120)
#define MDM_REGISTRATION_TIMEOUT  K_SECONDS(180)
#define MDM_PROMPT_CMD_DELAY	  K_MSEC(75)
#define MDM_SENDMSG_SLEEP         K_MSEC(1)
#define MDM_MAX_DATA_LENGTH		  1024
#define MDM_RECV_MAX_BUF		  30
#define MDM_RECV_BUF_SIZE		  128
#define MDM_MAX_SOCKETS			  6
#define MDM_BASE_SOCKET_NUM		  0
#define MDM_NETWORK_RETRY_COUNT	  3
#define MDM_WAIT_FOR_RSSI_COUNT	  10
#define MDM_WAIT_FOR_RSSI_DELAY	  K_SECONDS(2)
#define BUF_ALLOC_TIMEOUT		  K_SECONDS(1)

/* Default lengths of certain things. */
#define MDM_MANUFACTURER_LENGTH	  10
#define MDM_MODEL_LENGTH		  16
#define MDM_REVISION_LENGTH		  64
#define MDM_IMEI_LENGTH			  16
#define MDM_IMSI_LENGTH			  16
#define MDM_APN_LENGTH			  32
#define RSSI_TIMEOUT_SECS		  30

/* Modem ATOI routine. */
#define ATOI(s_, value_, desc_)   modem_atoi(s_, value_, desc_, __func__)

/* pin settings */
enum mdm_control_pins {
	MDM_POWER = 0,
	MDM_RESET,
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
	char mdm_imsi[MDM_IMSI_LENGTH];

	/* modem state */
	int ev_creg;

	/* bytes written to socket in last transaction */
	int sock_written;

	/* response semaphore */
	struct k_sem sem_response;
};

/* Allocating static memory for various routines / buffers. */
K_KERNEL_STACK_DEFINE(modem_rx_stack, CONFIG_MODEM_QUECTEL_BG9X_RX_STACK_SIZE);
K_KERNEL_STACK_DEFINE(modem_workq_stack, CONFIG_MODEM_QUECTEL_BG9X_RX_WORKQ_STACK_SIZE);
NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0, NULL);

/* Modem data structures. */
struct k_thread             modem_rx_thread;
static struct k_work_q      modem_workq;
static struct modem_data    mdata;
static struct modem_context mctx;

/* Modem pins - Power & Reset. */
static struct modem_pin modem_pins[] = {

	/* MDM_POWER */
	MODEM_PIN(DT_INST_GPIO_LABEL(0, mdm_power_gpios),
			  DT_INST_GPIO_PIN(0, mdm_power_gpios),
		      DT_INST_GPIO_FLAGS(0, mdm_power_gpios) | GPIO_OUTPUT),

	/* MDM_RESET */
	MODEM_PIN(DT_INST_GPIO_LABEL(0, mdm_reset_gpios),
			  DT_INST_GPIO_PIN(0, mdm_reset_gpios),
			  DT_INST_GPIO_FLAGS(0, mdm_reset_gpios) | GPIO_OUTPUT),
};

#endif /* #ifndef QUECTEL_BG9X_H */
