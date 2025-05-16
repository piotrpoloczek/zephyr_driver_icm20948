cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(icm20948)

add_subdirectory(drivers/sensor/icm20948)
