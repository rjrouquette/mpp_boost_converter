#!/bin/bash

# cleanup old files
rm mppbc.elf mppbc.hex

SRC_FILES="
main.c
descriptors.c
cli.c
regulator.c

lufa/LUFA/Drivers/USB/Class/Device/CDCClassDevice.c

lufa/LUFA/Drivers/USB/Core/ConfigDescriptors.c
lufa/LUFA/Drivers/USB/Core/DeviceStandardReq.c
lufa/LUFA/Drivers/USB/Core/Events.c
lufa/LUFA/Drivers/USB/Core/USBTask.c

lufa/LUFA/Drivers/USB/Core/XMEGA/Device_XMEGA.c
lufa/LUFA/Drivers/USB/Core/XMEGA/EndpointStream_XMEGA.c
lufa/LUFA/Drivers/USB/Core/XMEGA/Endpoint_XMEGA.c
lufa/LUFA/Drivers/USB/Core/XMEGA/USBController_XMEGA.c
lufa/LUFA/Drivers/USB/Core/XMEGA/USBInterrupt_XMEGA.c
"

avr-gcc -Os -mmcu=atxmega16a4u -DF_CPU=32000000 -DF_USB=48000000 -DARCH=ARCH_XMEGA -DUSE_LUFA_CONFIG_HEADER -Ilufa -IConfig -o mppbc.elf $SRC_FILES
if [ $? -ne 0 ]; then exit 1; fi

avr-objcopy -j .text -j .data -O ihex mppbc.elf mppbc.hex
if [ $? -ne 0 ]; then exit 1; fi

avrdude -v -c jtag3pdi -p ATxmega16A4U -P usb -u -Uflash:w:mppbc.hex
