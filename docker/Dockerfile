# Use Ubuntu as the base image
FROM osrf/ros:humble-desktop-full

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    vim \
    gcc  g++  gfortran \
    git \
    patch \
    zip unzip \
    wget curl python3-pip \
    liblapack-dev  libmetis-dev  libblas-dev  libatlas-base-dev \
    libglfw3-dev \
    libglpk-dev \
    pkg-config \
    ipython3 \
    python3-dev  python3-tk\
    swig \
    doxygen doxygen-latex \
    liburdfdom-dev libassimp-dev \
    libboost-all-dev \
    ffmpeg \
    qtbase5-dev \
    --install-recommends && \
    python3 -m pip install --upgrade numpy && \
    pip install pybullet meshcat scipy matplotlib && \
    rm -rf /var/lib/apt/lists/*


# cmake
RUN wget https://cmake.org/files/v3.29/cmake-3.29.2.tar.gz
RUN tar -xzvf cmake-3.29.2.tar.gz
RUN cd cmake-3.29.2 && ./bootstrap && make -j$((`nproc`+1)) && make install


## Eigen 3.4.0
RUN cd /opt && \
git clone https://gitlab.com/libeigen/eigen.git && \
cd eigen && \
git checkout 3147391d && \ 
mkdir build && \
cd build && \
cmake .. && \
make install

# eigenpy
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
      -o /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] \
      http://robotpkg.openrobots.org/packages/debian/pub jammy robotpkg" \
      > /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && \
    apt-get install -y robotpkg-py310-eigenpy



# Create a non-root user
ARG USERNAME=docker
# ARG USER_UID=57088748
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir -p /home/$USERNAME/.config \
  && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Install sudo and set up access for non-root user
RUN apt-get update && apt-get install -y sudo \
&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
&& chmod 0440 /etc/sudoers.d/$USERNAME \
&& rm -rf /var/lib/apt/lists/*     



# Switch to the non-root user
USER $USERNAME
ENV HOME=/home/${USERNAME}

RUN mkdir -p /home/$USERNAME/repos
WORKDIR /home/$USERNAME/repos

# Source ROS setup in Dockerfile
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc



RUN mkdir -p /home/$USERNAME/repos
WORKDIR /home/$USERNAME/repos


RUN wget https://github.com/google-deepmind/mujoco/releases/download/3.2.6/mujoco-3.2.6-linux-x86_64.tar.gz
RUN tar -xvzf mujoco-3.2.6-linux-x86_64.tar.gz


#Clarabel.cpp
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/home/$USERNAME/.cargo/bin:${PATH}"
RUN git clone --recurse-submodules https://github.com/oxfordcontrol/Clarabel.cpp.git && \
    cd Clarabel.cpp && \
    mkdir build && \
    cd build && \
    cmake  -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build .   



# Pinocchio
RUN echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /home/$USERNAME/.bashrc

RUN git clone --recursive https://github.com/stack-of-tasks/pinocchio && \
cd pinocchio && \
# git checkout v3.3.1 && \
mkdir build && cd build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/opt/openrobots  && \
make -j4 && \
sudo make install


WORKDIR /home/$USERNAME

# # Add environment variables to .bashrc for user
# RUN echo "export PATH=/usr/local/bin:\$PATH" >> /home/$USERNAME/.bashrc \
#     && echo "export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:\$PKG_CONFIG_PATH" >> /home/$USERNAME/.bashrc \
#     && echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /home/$USERNAME/.bashrc \
#     && echo "export PYTHONPATH=/usr/local/lib/python3.10/site-packages:\$PYTHONPATH" >> /home/$USERNAME/.bashrc \
#     && echo "export CMAKE_PREFIX_PATH=/usr/local:\$CMAKE_PREFIX_PATH" >> /home/$USERNAME/.bashrc


# RUN echo "export PATH=$PATH:$HOME/.local/bin" >> /home/$USERNAME/.bashrc

# RUN echo "export LD_LIBRARY_PATH=$HOME/repos/mujoco-3.2.6/lib:$LD_LIBRARY_PATH" >> /home/$USERNAME/.bashrc \
#     && echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$USERNAME/repos/Clarabel.cpp/rust_wrapper/target/release" >> /home/$USERNAME/.bashrc

ENV PATH=/usr/local/bin:$PATH \
    PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH \
    LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
    PYTHONPATH=/usr/local/lib/python3.10/site-packages:$PYTHONPATH \
    CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH

ENV PATH=$PATH:$HOME/.local/bin
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$USERNAME/repos/mujoco-3.2.6/lib
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$USERNAME/repos/Clarabel.cpp/rust_wrapper/target/release
