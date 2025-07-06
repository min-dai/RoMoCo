# Use Ubuntu as the base image
FROM osrf/ros:noetic-desktop-full

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    vim \
    gcc  g++  gfortran \
    git \
    patch \
    zip unzip \
    wget  cmake  pip \
    liblapack-dev  libmetis-dev  libblas-dev  libatlas-base-dev \
    libglfw3-dev \
    libglpk-dev \
    pkg-config \
    ipython3 \
    python3-dev python3-catkin-tools python3-tk\
    swig \
    curl \
    doxygen doxygen-latex \
    liburdfdom-dev libassimp-dev \
    ros-noetic-eigenpy \
    libboost-all-dev \
    ffmpeg \
    qtcreator qtbase5-dev \
    --install-recommends && \
    python3 -m pip install --upgrade numpy && \
    pip install pybullet meshcat scipy matplotlib && \
    rm -rf /var/lib/apt/lists/*


## Eigen 3.4.0
RUN cd /opt && \
git clone https://gitlab.com/libeigen/eigen.git && \
cd eigen && \
git checkout 3147391d && \ 
mkdir build && \
cd build && \
cmake .. && \
make install


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

RUN mkdir -p /home/$USERNAME/repos
WORKDIR /home/$USERNAME/repos

# Source ROS setup in Dockerfile
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc









# # cmake
# RUN wget https://cmake.org/files/v3.29/cmake-3.29.2.tar.gz
# RUN tar -xzvf cmake-3.29.2.tar.gz
# RUN cd cmake-3.29.2 && ./bootstrap && make -j$((`nproc`+1)) && make install





# # Install mujoco
# WORKDIR /home/$USERNAME
# RUN wget https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz
# RUN tar -xvzf mujoco210-linux-x86_64.tar.gz
# RUN mkdir -p /home/$USERNAME/.mujoco
# RUN mv mujoco210 /home/$USERNAME/.mujoco/



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


# Source build Ipopt and its dependencies
RUN git clone https://github.com/coin-or-tools/ThirdParty-ASL.git && \
cd ThirdParty-ASL && \
./get.ASL && \
./configure --with-lapack-lflags="-L$HOME/lib -lf77blas -lcblas -latlas" && \
make && sudo make install 

RUN git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git && \
cd ThirdParty-Mumps && \
./get.Mumps && \
./configure && \  
make && sudo make install


RUN git clone https://github.com/coin-or/Ipopt.git && \
cd Ipopt && \
mkdir build && cd build && \
../configure && \
make && sudo make install


# Clone and build CasADi from source (if needed, skip this step for a binary install)
RUN git clone https://github.com/casadi/casadi.git -b main casadi && \
    cd casadi && \
    mkdir build && cd build && \
    cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON .. && \
    make && \
    sudo make install 



# HPP-FCL
# Set eigenpy_DIR to the directory containing eigenpy CMake files
ENV eigenpy_DIR="/opt/ros/noetic/share/eigenpy/cmake"


RUN git clone --recursive https://github.com/humanoid-path-planner/hpp-fcl && \
cd hpp-fcl && \
git checkout v2.4.5 && \
mkdir build && cd build && \
cmake .. -DCMAKE_BUILD_TYPE=Release && \
make -j4 && \
sudo make install

# Pinocchio
RUN git clone --recursive https://github.com/stack-of-tasks/pinocchio && \
cd pinocchio && \
git checkout v3.3.1 && \
mkdir build && cd build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_WITH_COLLISION_SUPPORT=ON -DBUILD_WITH_CASADI_SUPPORT=ON  && \
make -j4 && \
sudo make install


WORKDIR /home/$USERNAME

# Add environment variables to .bashrc for user
RUN echo "export PATH=/usr/local/bin:\$PATH" >> /home/$USERNAME/.bashrc \
    && echo "export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:\$PKG_CONFIG_PATH" >> /home/$USERNAME/.bashrc \
    && echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /home/$USERNAME/.bashrc \
    && echo "export PYTHONPATH=/usr/local/lib/python3.8/site-packages:\$PYTHONPATH" >> /home/$USERNAME/.bashrc \
    && echo "export CMAKE_PREFIX_PATH=/usr/local:\$CMAKE_PREFIX_PATH" >> /home/$USERNAME/.bashrc


RUN echo "export PATH=$PATH:$HOME/.local/bin" >> /home/$USERNAME/.bashrc


# Source ROS setup in Dockerfile
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc

