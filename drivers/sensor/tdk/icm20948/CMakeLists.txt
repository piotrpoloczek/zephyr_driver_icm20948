# SPDX-License-Identifier: Apache-2.0


zephyr_library()

zephyr_library_sources(icm20948.c)
zephyr_library_sources(icm20948_i2c.c)
zephyr_library_sources_ifdef(CONFIG_ICM20948_TRIGGER icm20948_trig.c)