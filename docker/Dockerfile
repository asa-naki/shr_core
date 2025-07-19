# Dockerfile for ROS2 Robotics Kit ISO Builder
FROM ubuntu:22.04

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

# Install build dependencies
RUN apt-get update && apt-get install -y \
    # ISO building tools
    debootstrap \
    squashfs-tools \
    xorriso \
    isolinux \
    syslinux-efi \
    grub-pc-bin \
    grub-efi-amd64-bin \
    grub-efi-arm64-bin \
    mtools \
    dosfstools \
    # Cross-architecture support
    qemu-user-static \
    binfmt-support \
    # Network tools
    wget \
    curl \
    # Development tools
    git \
    python3 \
    python3-pip \
    # Cleanup
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Ansible
RUN pip3 install ansible ansible-lint yamllint

# Enable QEMU binary formats
RUN update-binfmts --enable

# Create working directories
RUN mkdir -p /workspace /output /tmp/iso-build
WORKDIR /workspace

# Copy build scripts
COPY scripts/ /workspace/scripts/
COPY ansible/ /workspace/ansible/
RUN chmod +x /workspace/scripts/*.sh

# Set build variables
ENV ROS2_DISTRO=humble
ENV UBUNTU_VERSION=22.04
ENV ARCHITECTURE=amd64

# Default command
CMD ["/bin/bash", "-c", "make build-dev"]

# Labels
LABEL maintainer="asa-naki"
LABEL description="Docker container for building ROS2 Robotics Kit ISO images"
LABEL version="1.0"
