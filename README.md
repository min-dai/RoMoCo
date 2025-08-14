# RoMoCo



<div style="display: flex; justify-content: center; gap: 20px;">

<img src="img/qp.gif" alt="G1 walking" width="30%"/>
<img src="img/posik.gif" alt="G1 running TSCQP" width="30%"/>
<img src="img/qpm.gif" alt="G1 running hierarchical QP" width="30%"/>

</div>

<div style="display: flex; justify-content: center; gap: 20px;">

<img src="img/cassiemlip.gif" alt="Cassie MLIP" width="30%"/>

</div>


**RoMoCo** (Reduced-order Modeling and Control) is a C++ framework for simulation and control of bipedal and humanoid robots.
It provides modular components for planning, control, and simulation interfaces, with the goal of offering a unified, extensible framework for research and development.



## Features
- Modular design for walking, standing, and in-air robot control
- Pinocchio-based kinematics and dynamics
- Integration with MuJoCo for simulation
- Support for state-of-the-art reduced-order model planners (HLIP, DCM; ALIP/MLIP planned)
- Multiple whole-body control methods: TSCQP, position IK, velocity IK, inverse dynamics, hierarchical QP (WIP)
- Easy-to-extend base classes for new robots or controllers

## Overview
- biped_core: Core functionalities: robot kinematics, dynamics, and base classes.
- biped_planner: Reduced-order model planners
- biped_control: Output generation and embedding for whole-body control
- mujoco_interface: Interface for MuJoCo simulation
- torque_control: Different whole-body torque control methods
- biped_state_machine: Example state machine for bipedal locomotion
- biped_utils: Utility functions: Bezier, Yaml parsing, geometry, etc.
  
Robots: G1, Cassie, H1, more to come  

## Build & Run
Tested with ROS 2 Humble and Ubuntu 22.04 (Jammy).

### Run from Docker
```bash
./docker/build.sh
./docker/run.sh
```

### Build locally
Install the dependencies listed below then clone the repository and build it using `colcon build`.
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
```


### Build
Install the dependencies listed below then clone the repository and build it using `colcon build`.

### Run Tests
```bash
colcon test
```

### Launch
- ROS 2 (recommended):
  - Source the workspace: `. install/setup.bash`
  - Example launch commands (adjust names if different):
    - `ros2 launch g1_simulation g1mujoco.launch.xml` or `ros2 launch g1_simulation g1mujoco.launch.py`
    - `ros2 launch cassie_simulation cassiemujoco.launch.py` (includes a fake radio; toggle SB from 0→1 for stand-to-walk)


## License
This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE) file for details.

## Installation
```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential vim gcc g++ gfortran git patch wget cmake \
  python3-pip liblapack-dev libmetis-dev libblas-dev libatlas-base-dev \
  libglpk-dev pkg-config ipython3 python3-dev python3-tk swig \
  doxygen doxygen-latex liburdfdom-dev libassimp-dev libboost-all-dev \
  libglfw3 libglfw3-dev qtcreator qtbase5-dev lsb-release --install-recommends

python3 -m pip install --upgrade pip numpy

echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

### EigenPy
Repository: https://github.com/stack-of-tasks/eigenpy/tree/master

On Ubuntu 22.04 (Python 3.10):
```bash
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt-get update
sudo apt install robotpkg-py310-eigenpy
```
Also add its path to your `.bashrc`:
```bash
echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> ~/.bashrc
```

If your Python minor version differs, install the matching package (e.g., robotpkg-py39-eigenpy for Python 3.9).

### Pinocchio 
```bash
mkdir -p ~/repos && cd ~/repos
git clone --recursive https://github.com/stack-of-tasks/pinocchio
cd pinocchio
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_PREFIX_PATH="/opt/openrobots"
make -j4
sudo make install
```
Update your `.bashrc` (PYTHONPATH adapted dynamically to your Python version):
```bash
export PATH=/usr/local/bin:$PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python3.10/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
```


Eigen 3.4.0
```
cd ~/repos/
git clone https://gitlab.com/libeigen/eigen.git 
cd eigen 
git checkout 3147391d 
mkdir build 
cd build 
cmake .. 
sudo make install
```

### MuJoCo
```bash
mkdir -p ~/repos && cd ~/repos
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.6/mujoco-3.2.6-linux-x86_64.tar.gz
tar -xvzf mujoco-3.2.6-linux-x86_64.tar.gz
rm mujoco-3.2.6-linux-x86_64.tar.gz 


### QP (Clarabel.cpp)
```bash
mkdir -p ~/repos && cd ~/repos
git clone --recurse-submodules https://github.com/oxfordcontrol/Clarabel.cpp.git
cd Clarabel.cpp
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . 
```
If installing to a non-standard prefix, add it to `CMAKE_PREFIX_PATH`.

### For debugging (Eigen pretty printers)
```bash
mkdir -p ~/repos && cd ~/repos
git clone https://github.com/dmillard/eigengdb
cd eigengdb
sudo python3 setup.py install
python3 bin/eigengdb_register_printers
```



## Todo
- [ ] Hierarchical QP controller for humanoids (WIP)
- [ ] Add multi-domain planner MLIP and embedding (integration needed)
- [ ] Kalman filter used for Cassie (integration needed)


