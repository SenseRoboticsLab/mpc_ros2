# Install
## 1 install Ipopt-3.12.8
```bash
cd [Ipopt-3.12.8]
mkdir build && cd build
../configure
make FFLAGS+=' -fallow-argument-mismatch' -j8
make install
sudo cp -a include/* /usr/include/.  
sudo cp -a lib/* /usr/lib/. 
```
## 2 ros build
```bash
cd [workspace]
colcon build
source ./install/setup.bash
ros2 launch mpc_ros mpc_launch.py
```