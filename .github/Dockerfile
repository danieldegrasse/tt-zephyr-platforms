# Image to run tt-zephyr-platforms CI tests (nightly and per-PR)
# rebuilt nightly

ARG BASE_REV=main
FROM ghcr.io/zephyrproject-rtos/zephyr-build:${BASE_REV} AS base

USER root

# Patch APT urls to use https, since the container can't connect via HTTP
RUN sed -i -e 's/http/https/g' /etc/apt/sources.list && \
    apt-get update

# Enable virtual environments
RUN apt-get install -y python3-venv

# Clone the tt-zephyr-platforms repo
RUN pip install west && \
    west init -m https://github.com/danieldegrasse/tt-zephyr-platforms /tt-zephyr && \
    cd /tt-zephyr && \
    west update

# Install Zephyr Cmake package for root
RUN cp /home/user/.cmake/ ~ -r

# Start a new stage- the build artifacts are not needed in the final image
FROM base AS builder

# Pull in CI components to build
RUN cd /tt-zephyr && \
    west config manifest.group-filter -- +ci && \
    west update

# Install clang-17 to build metal (we need to manually patch the apt sources to use https)
RUN mkdir /tmp/llvm-build && \
    cd /tmp/llvm-build && \
    wget https://apt.llvm.org/llvm.sh && \
    sed -i "s/http:\/\/apt.llvm.org/https:\/\/apt.llvm.org/" -i llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh 17 all && \
    cd ~ && \
    rm -rf /tmp/llvm-build

# Install metal dependencies
RUN cd /tt-zephyr/ci/tt-metal && \
    ./install_dependencies.sh --docker --mode build

# Update cmake to 3.30.0 (not the latest, so we use apt-mark hold to prevent it from being updated)
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && \
    apt-get install -y --allow-downgrades cmake=3.30.8-0kitware1ubuntu22.04.1 cmake-data=3.30.8-0kitware1ubuntu22.04.1 && \
    apt-mark hold cmake cmake-data

# Build metal
RUN cd /tt-zephyr/ci/tt-metal && \
    ./build_metal.sh --build-tests --build-programming-examples

# Setup reusable metal python environment
RUN cd /tt-zephyr/ci/tt-metal && \
    PYTHON_ENV_DIR=/metal-env source ./create_venv.sh

# Build openocd revision with RTT support
RUN cd /tt-zephyr/ci/openocd && \
    ./bootstrap && \
    ./configure --prefix=/opt/openocd && \
    make -j$(nproc) && \
    make install

# Setup reusable Zephyr env
RUN python3 -m venv /zephyr-env && \
    source /zephyr-env/bin/activate && \
    pip install -r /tt-zephyr/tt-zephyr-platforms/scripts/requirements.txt && \
    pip install -r /tt-zephyr/zephyr/scripts/requirements.txt

# Install tt-flash
RUN source /zephyr-env/bin/activate && \
    pip install /tt-zephyr/ci/tt-flash

# Install pyluwen
RUN source /zephyr-env/bin/activate && \
    pip install maturin && \
    cd /tt-zephyr/ci/luwen/crates/pyluwen/ && \
    maturin build && \
    pip install /tt-zephyr/ci/luwen/target/wheels/pyluwen-* --force-reinstall

# Copy the build environments and metal to a new stage
# FROM base
#
# COPY --from=builder /metal-env /metal-env
# COPY --from=builder /zephyr-env /zephyr-env
# COPY --from=builder /opt/openocd /opt/openocd
# COPY --from=builder /tt-zephyr/ci/tt-metal /tt-zephyr/ci/tt-metal
#
# RUN cd /tt-zephyr/ci/tt-metal && \
#     ./install_dependencies.sh --docker --mode runtime
