# SPDX-License-Identifier: Apache-2.0

set(ICM20948_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "ICM20948 module root directory")

add_subdirectory(drivers/sensor/tdk)

zephyr_include_directories(drivers/sensor/tdk)
