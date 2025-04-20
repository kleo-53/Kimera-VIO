FROM ubuntu:20.04
LABEL maintainer="mabate@mit.edu"

# To avoid tzdata asking for geographic location...
ENV DEBIAN_FRONTEND=noninteractive

ENV DIRPATH /root/
WORKDIR $DIRPATH

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && \
      apt-get install -y \
      build-essential \
      cmake \
      gfortran \
      git \
      libatlas-base-dev \
      libboost-all-dev \
      libeigen3-dev \
      libgflags-dev \
      libgoogle-glog-dev \
      libmetis-dev \
      libopencv-dev \
      libopencv-contrib-dev \
      libtbb-dev \
      pkg-config \
      xvfb \
      python3 \
      python3-dev \
      python3-pip \
      python3-tk && \
      apt-get clean && \
      rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/borglab/gtsam.git && \
      cd gtsam && \
      git checkout 4.2 && \
      mkdir build && \
      cd build && \
      cmake -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_UNSTABLE=ON \
      -DGTSAM_POSE3_EXPMAP=ON \
      -DGTSAM_ROT3_EXPMAP=ON \
      -DGTSAM_TANGENT_PREINTEGRATION=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_USE_SYSTEM_METIS=ON \
      .. && \
      make -j$(nproc) install

# Install Open_GV
RUN git clone https://github.com/marcusabate/opengv && \
      cd opengv && \
      git apply march_native_disable.patch && \
      mkdir build && \
      cd build && \
      cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      .. && \
      make -j$(nproc) install

# Install DBoW2
# COPY .patches/fix_vocab.patch dbow2.patch
RUN git clone https://github.com/dorian3d/DBoW2.git && \
    cd DBoW2 && \
    git checkout v1.1-free-enhancement-rmsalinas && \
    git reset --hard HEAD && \
    git apply ../dbow2.patch || echo "Patch not applicable, skipping" && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) install

# Install RobustPGO
ADD https://api.github.com/repos/MIT-SPARK/Kimera-RPGO/git/refs/heads/master version.json
RUN git clone https://github.com/MIT-SPARK/Kimera-RPGO.git
RUN cd Kimera-RPGO && \
      mkdir build && \
      cd build && \
      cmake .. && \
      make -j$(nproc) install

# Install Kimera-VIO
# RUN git clone https://github.com/MIT-SPARK/Kimera-VIO.git
# RUN cd Kimera-VIO && mkdir build && cd build && cmake .. && make -j$(nproc)
# RUN cd Kimera-VIO && \
#     mkdir -p build && \
#     cd build && \
#     cmake .. && \
#     make gnssMy -j$(nproc) && \
#     cp gnssMy /usr/local/bin/

# RUN git clone https://github.com/kleo-53/Kimera-VIO.git
# RUN cd Kimera-VIO && mkdir build && cd build && cmake .. && make -j$(nproc) && make GnssStereoVIO -j$(nproc) && cp GnssStereoVIO /usr/local/bin/
# RUN cd Kimera-VIO && mkdir build && cd build && cmake .. && make help | grep GnssStereoVIO && make -j$(nproc) && make GnssStereoVIO -j$(nproc) && cp GnssStereoVIO /usr/local/bin/

# # Копируем Kimera-VIO в контейнер
# COPY . /root/Kimera-VIO

# # Собираем Kimera-VIO
# RUN cd /root/Kimera-VIO && \
#     # Удаляем старый кэш CMake (если есть)
#     rm -rf build && \
#     mkdir -p build && cd build && \
#     # Запускаем CMake с нуля
#     cmake -DCMAKE_BUILD_TYPE=Release .. && \
#     make -j$(($(nproc) - 1)) && \
#     make install
# #     cmake .. && \
# #     make -j$(nproc) && \
# #     make install

# Собираем odometer (после основной сборки Kimera-VIO)
# RUN cd /root/Kimera-VIO/odometer && \
#     rm -rf build && \
#     mkdir -p build && cd build && \
#     cmake .. && \
#     make -j$(nproc)
    
# RUN cd Kimera-VIO && \
#     mkdir build && \
#     cd build && \
#     cmake .. && \
# #     make help && \
#     make -j$(nproc) || (echo "make failed" && exit 1)
#     make GnssStereoVIO -j$(nproc) || (echo "make GnssStereoVIO failed" && exit 1) && \
#     cp GnssStereoVIO /usr/local/bin/
# COPY ./odometer /root/odometer
# RUN cd /Kimera-VIO/odometer && \
#     mkdir -p build && cd build && \
#     cmake .. && \
#     make -j$(nproc)

# RUN cd Kimera-VIO && \
#     mkdir -p build && \
#     cd build && \
#     cmake .. && \
#     make -j$(nproc) install

# RUN cd /Kimera-VIO && \
#     rm -rf build && \
#     mkdir -p build && \
#     cd build && \
#     cmake .. && \
#     make gnssMy -j$(nproc) && \
#     cp gnssMy /usr/local/bin/