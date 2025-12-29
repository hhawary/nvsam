# NVSAM

This project is for demo purposes

## Dependency
- zed_wrapper (humble-v4.0.8)

## Docker

### Build
```bash
cd nvsam/docker/
docker build . -f ./Dockerfile -t nvsam
```
## How to use?

### Compile nvsam package
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src

git clone https://github.com/stereolabs/zed-ros2-wrapper.git -b humble-v4.0.8 --recursive
git clone git@github.com:hhawary/nvsam.git -b zed_wrapper --recursive

cd ..
colcon build
```
### Run nvsam

#### Option 1: Online


*Default*
```
cd ros2_ws
source install/setup.bash
ros2 launch nvsam nvsam_launch.py
```

set max_distance (default 20.0 meter)

```
cd ros2_ws
source install/setup.bash
ros2 launch nvsam nvsam_launch.py max_distance:=10.0
```


#### Option 2: Offline

```
cd ros2_ws
source install/setup.bash
ros2 launch nvsam nvsam_launch.py svo_path:=<path to svo file> max_distance:=10.0 fps:=15.0
```