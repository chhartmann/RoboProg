from __future__ import print_function

import os

Import("env")

action = "python " + env['OBJCOPY'] # this expands to esptool.py; directly call esptool results in "python3\lib\site.py", line 178 file=sys.stderr"
action += " --chip esp32 merge_bin -o $BUILD_DIR/qemu_image.bin --flash_mode dout --flash_size 4MB --fill-flash-size 4MB"
action +=  " 0x10000 $BUILD_DIR/firmware.bin"
action += " 0x290000 $BUILD_DIR/spiffs.bin"
for bin in env['FLASH_EXTRA_IMAGES']:
    action += " " + bin[0] + " " + bin[1]

#def my_action(source, target, env):
#    print("hello world")

env.AddCustomTarget(
    name="qemu-image",
    dependencies=["$BUILD_DIR/firmware.bin", "$BUILD_DIR/partitions.bin"],
    actions=["platformio run -t buildfs", action], # dependency to spiffs is not working, so build it here
    title="Qemu Image",

    description="Image for Qemu")
