# Copyright (c) 2020 Analog Life LLC
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BQ2589X bq2589x.c)
