#!/bin/bash
set -e
qemu-system-xtensa -nographic -M esp32 -m 4 -drive file=build/merged_qemu.bin,if=mtd,format=raw -nic user,model=open_eth,hostfwd=tcp::7654-:80 -global driver=timer.esp32.timg,property=wdt_disable,value=true $@
