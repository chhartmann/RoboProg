FROM gitpod/workspace-full

USER gitpod

#from https://github.com/espressif/esp-idf/blob/master/tools/docker/Dockerfile
ARG DEBIAN_FRONTEND=noninteractive

# We need libpython2.7 due to GDB tools
RUN : \
  && sudo apt-get update \
  && sudo apt-get install -y \
    apt-utils \
    bison \
    ca-certificates \
    ccache \
    check \
    curl \
    flex \
    git \
    gperf \
    lcov \
    libffi-dev \
    libncurses-dev \
    libpython2.7 \
    libusb-1.0-0-dev \
    make \
    ninja-build \
    python3 \
    python3-pip \
    unzip \
    wget \
    xz-utils \
    zip \
  && sudo apt-get autoremove -y \
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10 \
  && python -m pip install --upgrade \
    pip \
    virtualenv \
  && :

# To build the image for a branch or a tag of IDF, pass --build-arg IDF_CLONE_BRANCH_OR_TAG=name.
# To build the image with a specific commit ID of IDF, pass --build-arg IDF_CHECKOUT_REF=commit-id.
# It is possibe to combine both, e.g.:
#   IDF_CLONE_BRANCH_OR_TAG=release/vX.Y
#   IDF_CHECKOUT_REF=<some commit on release/vX.Y branch>.

ARG IDF_CLONE_URL=https://github.com/espressif/esp-idf.git
ARG IDF_CLONE_BRANCH_OR_TAG=release/v4.4
ARG IDF_CHECKOUT_REF=

ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp

RUN echo IDF_CHECKOUT_REF=$IDF_CHECKOUT_REF IDF_CLONE_BRANCH_OR_TAG=$IDF_CLONE_BRANCH_OR_TAG && \
    sudo git clone --recursive \
      ${IDF_CLONE_BRANCH_OR_TAG:+-b $IDF_CLONE_BRANCH_OR_TAG} \
      $IDF_CLONE_URL $IDF_PATH && \
    if [ -n "$IDF_CHECKOUT_REF" ]; then \
      cd $IDF_PATH && \
      sudo git checkout $IDF_CHECKOUT_REF && \
      sudo git submodule update --init --recursive; \
    fi

RUN sudo chown -R ${USER} /opt/esp

# Install all the required tools
RUN : \
  && sudo update-ca-certificates --fresh \
  && $IDF_PATH/tools/idf_tools.py --non-interactive install required \
  && $IDF_PATH/tools/idf_tools.py --non-interactive install cmake \
  && $IDF_PATH/tools/idf_tools.py --non-interactive install-python-env \
  && sudo rm -rf $IDF_TOOLS_PATH/dist \
  && :

# Ccache is installed, enable it by default
ENV IDF_CCACHE_ENABLE=1
#COPY entrypoint.sh /opt/esp/entrypoint.sh

#ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]
#CMD [ "/bin/bash" ]

# from https://github.com/espressif/vscode-esp-idf-extension/blob/master/templates/.devcontainer/Dockerfile

RUN sudo apt-get update \
  && sudo apt install -y -q \
  cmake \
  git \
  hwdata \
  libglib2.0-0 \
  libnuma1 \
  libpixman-1-0 \
  linux-tools-5.4.0-77-generic \
  && sudo rm -rf /var/lib/apt/lists/*

RUN sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/5.4.0-77-generic/usbip 20

# QEMU
ENV QEMU_REL=esp-develop-20220203
#ENV QEMU_SHA256=44c130226bdce9aff6abf0aeaab44f09fe4f2d71a5f9225ac1708d68e4852c02
ENV QEMU_DIST=qemu-${QEMU_REL}.tar.bz2
ENV QEMU_URL=https://github.com/espressif/qemu/releases/download/${QEMU_REL}/${QEMU_DIST}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN wget --no-verbose ${QEMU_URL} \
#  && echo "${QEMU_SHA256} *${QEMU_DIST}" | sha256sum --check --strict - \
  && sudo tar -xf $QEMU_DIST -C /opt \
  && rm ${QEMU_DIST}

ENV PATH=/opt/qemu/bin:${PATH}

RUN sudo chown -R ${USER} /opt/qemu

RUN echo $($IDF_PATH/tools/idf_tools.py export) >> $HOME/.bashrc

# for microros build
RUN pip install -U colcon-common-extensions
RUN pip install -U lark
RUN sudo apt-get update && sudo apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN sudo apt-get update && sudo apt-get install -y python3-rosdep2
RUN git clone -b galactic https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN rosdep update && rosdep install --from-paths src --ignore-src -y --rosdistro galactic
