FROM debian:bookworm

WORKDIR /root

ENV ARCH=riscv
ENV CROSS_COMPILE=riscv32-linux-

# Install necessary packages
RUN apt-get update -y
RUN apt-get install -y --no-install-recommends device-tree-compiler git build-essential wget ca-certificates file \
        cpio unzip rsync bc flex curl python3
# Copy buildroot and linux config files
RUN mkdir -p $(pwd)/build/{buildroot,linux}
COPY config/buildroot_config build/buildroot/.config
COPY config/linux_config build/linux/.config
# Build buildroot - commit hash e889a1c9e983753dd0fa5062d3b9475a8cba6072
RUN git clone https://github.com/buildroot/buildroot.git && cd buildroot && git reset --hard e889a1c9e9 && cd ../
RUN make -C $(pwd)/buildroot -j$(nproc) O=$(pwd)/build/buildroot/
RUN rm -rf buildroot build/buildroot/build build/buildroot/staging
# Add cross compiler path
ENV PATH="/root/build/buildroot/host/bin:$PATH"
# Build linux
RUN git config --global http.version HTTP/1.1
RUN git clone --depth 1 --branch litex-rebase https://github.com/antmicro/linux.git
RUN make -C $(pwd)/linux -j$(nproc) O=$(pwd)/build/linux/
