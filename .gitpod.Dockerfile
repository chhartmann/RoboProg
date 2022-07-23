FROM gitpod/workspace-full

USER gitpod

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