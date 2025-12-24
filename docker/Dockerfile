# Autonomous Rover Docker Container
# Optimized for NVIDIA Jetson Orin Nano with CUDA 12.6 and TensorRT 10.3

# Base image with CUDA and cuDNN pre-installed
FROM nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3

# Maintainer
LABEL maintainer="autonomous-rover"
LABEL description="Autonomous rover with computer vision on Jetson Orin"

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-opencv \
    libopencv-dev \
    git \
    cmake \
    wget \
    curl \
    vim \
    nano \
    build-essential \
    pkg-config \
    # GStreamer for hardware acceleration
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    # Serial communication
    libserial-dev \
    # Additional tools
    htop \
    iotop \
    tmux \
    usbutils \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace/autonomous-rover

# Copy requirements
COPY ../requirements.txt .

# Install Python dependencies
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir -r requirements.txt

# Install jetson-stats for monitoring
RUN pip3 install --no-cache-dir jetson-stats

# Install additional Jetson-specific packages
RUN pip3 install --no-cache-dir \
    pycuda \
    Jetson.GPIO

# Copy application code
COPY ../src ./src
COPY ../config ./config
COPY ../models ./models

# Create directories for logs and data
RUN mkdir -p logs data

# Expose ports for telemetry and debugging
EXPOSE 8080 5000

# Set permissions
RUN chmod +x src/**/*.py 2>/dev/null || true

# Default command
CMD ["python3", "src/main.py"]
