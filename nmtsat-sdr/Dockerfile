FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential cmake pkg-config git \
    libusb-1.0-0-dev \
    libfftw3-dev \
    libcurl4-openssl-dev \
    libncurses-dev \
    python3 python3-pip python3-dev \
    usbutils \
    curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# RTL-SDR
RUN git clone https://github.com/librtlsdr/librtlsdr.git /tmp/rtl-sdr && \
    cd /tmp/rtl-sdr && mkdir build && cd build && \
    cmake .. -DINSTALL_UDEV_RULES=ON -DDETACH_KERNEL_DRIVER=ON && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/rtl-sdr

# BladeRF
RUN git clone https://github.com/Nuand/bladeRF.git /tmp/bladeRF && \
    cd /tmp/bladeRF/host && mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/bladeRF

# Python deps
RUN pip3 install --no-cache-dir numpy matplotlib

CMD ["cd /bench"]

CMD ["/bin/bash"]
