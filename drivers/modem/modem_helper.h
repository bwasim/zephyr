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

#endif /* #ifndef MODEM_HELPER_H */
