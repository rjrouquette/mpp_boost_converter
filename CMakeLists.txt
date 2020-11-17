cmake_minimum_required(VERSION 2.8.4)

# Use gcc and g++
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

# don't test as they won't work
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

project("avr-mppbc")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -Wall -mmcu=atxmega16a4u -D__AVR_ATxmega16A4U__ -DF_CPU=32000000 -DF_USB=48000000 -DARCH=ARCH_XMEGA -DUSE_LUFA_CONFIG_HEADER")

include_directories("/usr/lib/avr/include")
include_directories("lufa")
include_directories("Config")

add_executable(
        mppbc

        main.c
        descriptors.c
        cli.c
        regulator.c

        lufa/LUFA/Drivers/USB/Class/Device/CDCClassDevice.c
)
