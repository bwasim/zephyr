# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SENSIRION_SPS30 sps30.c)
