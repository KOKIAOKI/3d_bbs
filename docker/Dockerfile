# Load base image
FROM nvidia/cuda:12.0.0-cudnn8-devel-ubuntu22.04

# Add maintainer info
LABEL maintainer="TaeYoung Kim <tyoung96@yonsei.ac.kr>"

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# Set essential packages
RUN apt-get update && apt-get install -y \
    libatlas-base-dev libgoogle-glog-dev \
    libsuitesparse-dev libglew-dev wget unzip git \
    python3-pip tzdata build-essential libboost-all-dev libeigen3-dev \
                         libflann-dev libvtk7-dev libqhull-dev libusb-dev \
                         libgtest-dev git-core freeglut3-dev pkg-config \
                         libxmu-dev libxi-dev libusb-1.0-0-dev libyaml-cpp-dev

# Install CMake 3.27
WORKDIR /root/tmp
RUN wget https://github.com/Kitware/CMake/archive/refs/tags/v3.27.9.zip && \
    unzip v3.27.9.zip && \
    cd CMake-3.27.9 && \
    ./bootstrap && \
    make && \
    make install

# Install Eigen 3.4.0
WORKDIR /root/tmp
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip && \
    unzip eigen-3.4.0.zip
WORKDIR /root/tmp/eigen-3.4.0/build
RUN cmake .. && make install

# Install PCL
RUN apt-get install -y libpcl-dev

WORKDIR /root/workspace

# Load ROS environment at each run
COPY ./entrypoint.sh /
RUN chmod 755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
