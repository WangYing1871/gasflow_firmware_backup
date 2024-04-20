CMAKE_MINIMUM_REQUIRED(VERSION 3.20.0)

FIND_PACKAGE(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

PROJECT(micro_flow_controller)

FILE(GLOB app_sources src/*.c)
                           
TARGET_SOURCES(app PRIVATE ${app_sources})
