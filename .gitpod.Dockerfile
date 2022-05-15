FROM gitpod/workspace-full
                    
USER gitpod

RUN pip3 install -U platformio
RUN wget --no-verbose https://github.com/espressif/qemu/releases/download/esp-develop-20220203/qemu-esp-develop-20220203.tar.bz2
RUN tar xf qemu-esp-develop-20220203.tar.bz2
RUN rm qemu-esp-develop-20220203.tar.bz2
