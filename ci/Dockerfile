FROM ubuntu:24.04@sha256:008173c23f95b170204355c12626cb5a965d779a7e1283b09e9cffbb1bf33ca3

ARG UV_VERSION=0.12.20
ARG UV_SHA256=6590717592ace991ff83a63fef799e3ad9d33ecc8f96c5d6bdd732496e79337f
# Node runs the GCS tests (gcs/: vitest, svelte-check; ci/run_ci.sh step gcs_tests). 22.x is the line the GCS is developed on
# (gcs/package.json devDependencies: @types/node 22, vite 8, vitest 5); the tarball is the official linux-x64 build, checked
# against the SHA-256 in https://nodejs.org/dist/v22.23.3/SHASUMS256.txt.
ARG NODE_VERSION=22.23.3
ARG NODE_SHA256=1084aa36196bba4c3a5e69a1ee388a6e4ff729dad09445fbcd434b28fe3c24af

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates=20260601~24.04.1 \
        curl=8.5.0-2ubuntu10.16 \
        g++=4:13.2.0-7ubuntu1 \
        g++-13=13.3.0-6ubuntu2~24.04.1 \
        gcc-arm-none-eabi=15:13.2.rel1-2 \
        libnewlib-arm-none-eabi=4.4.0.20231231-2 \
        libstdc++-arm-none-eabi-newlib=15:13.2.rel1-2+26 \
        git=1:2.43.0-1ubuntu7.3 \
        clang-tidy-18=1:18.1.3-1ubuntu1 \
        cmake=3.28.3-1build7 \
        ccache=4.9.1-1 \
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

RUN curl -fsSL -o /tmp/node.tar.gz \
        "https://nodejs.org/dist/v${NODE_VERSION}/node-v${NODE_VERSION}-linux-x64.tar.gz" \
    && echo "${NODE_SHA256}  /tmp/node.tar.gz" | sha256sum -c - \
    && tar -xzf /tmp/node.tar.gz -C /usr/local --strip-components=1 --no-same-owner \
        --exclude="node-v${NODE_VERSION}-linux-x64/CHANGELOG.md" --exclude="node-v${NODE_VERSION}-linux-x64/README.md" \
    && rm /tmp/node.tar.gz \
    && node --version \
    && npm --version

ENV UV_PYTHON=python3.12 \
    UV_PYTHON_DOWNLOADS=never

# Compiler cache (CMakeLists.txt uses ccache as the compiler launcher when it is found). The persistent directory is
# /cache, mounted by ci/local_ci.sh (the named volume marv-ci-cache) and by ci.yml (the actions/cache directory), with
# MARV_CACHE_DIR=/cache; the compiler cache is /cache/ccache.
# CCACHE_BASEDIR=/src: the clone is always at /src, so paths under it are rewritten relative to the compile directory
# for hashing, and the -I paths of the sibling build trees (build/host-debug, build/host-gz-l4, ...) hash alike.
# CCACHE_NOHASHDIR: the build directory is not hashed with -g, so Debug objects hit across build trees; the cost is a
# comp_dir in the debug info that names the tree that filled the cache, irrelevant to a CI build.
# CCACHE_COMPILERCHECK=content: the compiler is identified by the hash of its binary, not its mtime (every image build
# gives the files new mtimes).
# CCACHE_MAXSIZE=2G: a labelled choice, one job's cache directory, well below the 10 GB Actions cache quota that the
# five jobs share.
ENV MARV_CACHE_DIR=/cache \
    CCACHE_DIR=/cache/ccache \
    CCACHE_BASEDIR=/src \
    CCACHE_NOHASHDIR=true \
    CCACHE_COMPILERCHECK=content \
    CCACHE_MAXSIZE=2G

WORKDIR /src
