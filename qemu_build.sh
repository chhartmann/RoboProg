#!/bin/bash
set -e
idf.py build -DBUILD_FOR_QEMU=1
(cd build && /opt/esp/python_env/idf4.4_py3.8_env/bin/python /opt/esp/idf/components/esptool_py/esptool/esptool.py --chip esp32 merge_bin -o merged_qemu.bin --flash_mode dout --flash_size 4MB --fill-flash-size 4MB 0x10000 main.bin 0x1000 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0xe000 ota_data_initial.bin 0x290000 storage.bin)