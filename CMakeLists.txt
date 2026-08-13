#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

# Include project-level device tree additions (Power_on pin, io-channels, etc.)
#set(EXTRA_DTC_OVERLAY_FILE ${CMAKE_SOURCE_DIR}/ContinuousBiosensing.dts)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/my_pbm.c
  src/hardware.c
  src/dac8831.c
)

# NORDIC SDK APP END
zephyr_library_include_directories(.)
