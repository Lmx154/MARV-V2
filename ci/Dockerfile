FROM ubuntu:24.04@sha256:008173c23f95b170204355c12626cb5a965d779a7e1283b09e9cffbb1bf33ca3

ARG UV_VERSION=0.12.20
ARG UV_SHA256=6590717592ace991ff83a63fef799e3ad9d33ecc8f96c5d6bdd732496e79337f

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates=20260601~24.04.1 \
        curl=8.5.0-2ubuntu10.15 \
        g++=4:13.2.0-7ubuntu1 \
        g++-13=13.3.0-6ubuntu2~24.04.1 \
        gcc-arm-none-eabi=15:13.2.rel1-2 \
        libnewlib-arm-none-eabi=4.4.0.20231231-2 \
        libstdc++-arm-none-eabi-newlib=15:13.2.rel1-2+26 \
        git=1:2.43.0-1ubuntu7.3 \
        clang-tidy-18=1:18.1.3-1ubuntu1 \
        cmake=3.28.3-1build7 \
        ninja-build=1.11.1-2 \
        python3.12=3.12.3-1ubuntu0.17 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL -o /tmp/uv.tar.gz \
        "https://github.com/astral-sh/uv/releases/download/${UV_VERSION}/uv-x86_64-unknown-linux-gnu.tar.gz" \
    && echo "${UV_SHA256}  /tmp/uv.tar.gz" | sha256sum -c - \
    && tar -xzf /tmp/uv.tar.gz -C /usr/local/bin --strip-components=1 \
        uv-x86_64-unknown-linux-gnu/uv uv-x86_64-unknown-linux-gnu/uvx \
    && rm /tmp/uv.tar.gz \
    && uv --version

ENV UV_PYTHON=python3.12 \
    UV_PYTHON_DOWNLOADS=never

WORKDIR /src
